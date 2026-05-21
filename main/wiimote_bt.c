#include "wiimote_bt.h"
#include "wiimote_reports.h"
#include "wiimote_protocol.h"
#include "wiimote_state.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd_api.h"
#include "esp_sdp_api.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

#define WIIMOTE_DI_VENDOR_ID                (0x057E)
#define WIIMOTE_DI_PRODUCT_ID               (0x0306)
#define WIIMOTE_DI_PRODUCT_VERSION          (0x0100)
#define WIIMOTE_SYNC_DISCOVERABLE_WINDOW_MS (20000)

static const char *TAG = "wiimote_bt";
static const char  s_device_name[] = CONFIG_EXAMPLE_LOCAL_DEVICE_NAME;

static bool                 s_connected;
static esp_hidd_app_param_t s_app_param;
static esp_hidd_qos_param_t s_qos;
static esp_timer_handle_t   s_pairing_timer;

bool wiimote_bt_is_connected(void)
{
    return s_connected;
}

/* --- pairing window ------------------------------------------------------- */

/* A real Wii Remote is connectable + discoverable for ~20 s after the sync
 * button, then goes quiet. A one-shot timer reproduces that without a task. */
static void pairing_window_expired(void *arg)
{
    (void)arg;
    if (!s_connected) {
        ESP_LOGI(TAG, "[PAIR] window closed; reset the device to re-open it");
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    } else {
        ESP_LOGI(TAG, "[PAIR] HID link already active, window timer ignored");
    }
}

static void open_pairing_window(void)
{
    ESP_LOGI(TAG, "[PAIR] entering 20s sync-style discoverable window");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_LIMITED_DISCOVERABLE);

    if (s_pairing_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = pairing_window_expired,
            .name = "wiimote_pair",
        };
        if (esp_timer_create(&args, &s_pairing_timer) != ESP_OK) {
            ESP_LOGE(TAG, "failed to create pairing-window timer");
            return;
        }
    }
    esp_timer_start_once(s_pairing_timer,
                         (uint64_t)WIIMOTE_SYNC_DISCOVERABLE_WINDOW_MS * 1000);
}

/* --- SDP device-ID record ------------------------------------------------ */

static void create_di_record(void)
{
    esp_bluetooth_sdp_record_t di_record = {0};
    di_record.dip.hdr.type = ESP_SDP_TYPE_DIP_SERVER;
    di_record.dip.vendor = WIIMOTE_DI_VENDOR_ID;
    di_record.dip.vendor_id_source = ESP_SDP_VENDOR_ID_SRC_USB;
    di_record.dip.product = WIIMOTE_DI_PRODUCT_ID;
    di_record.dip.version = WIIMOTE_DI_PRODUCT_VERSION;
    di_record.dip.primary_record = true;

    esp_err_t ret = esp_sdp_create_record(&di_record);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_sdp_create_record failed: %s", esp_err_to_name(ret));
    }
}

static void esp_sdp_cb(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param)
{
    switch (event) {
    case ESP_SDP_INIT_EVT:
        if (param->init.status == ESP_SDP_SUCCESS) {
            ESP_LOGI(TAG, "SDP initialized; creating Nintendo DI record");
            create_di_record();
        } else {
            ESP_LOGE(TAG, "SDP init failed: %d", param->init.status);
        }
        break;
    case ESP_SDP_CREATE_RECORD_COMP_EVT:
        if (param->create_record.status == ESP_SDP_SUCCESS) {
            ESP_LOGI(TAG, "DI record created (vid=0x%04X pid=0x%04X)",
                     WIIMOTE_DI_VENDOR_ID, WIIMOTE_DI_PRODUCT_ID);
        } else {
            ESP_LOGE(TAG, "DI record create failed: %d", param->create_record.status);
        }
        break;
    default:
        break;
    }
}

/* --- GAP / legacy pairing ------------------------------------------------ */

/* Wii Remote legacy PIN = a 6-byte BD_ADDR written back-to-front. */
static void fill_reversed_bdaddr_pin(const esp_bd_addr_t source, esp_bt_pin_code_t pin)
{
    for (size_t i = 0; i < ESP_BD_ADDR_LEN; i++) {
        pin[i] = source[ESP_BD_ADDR_LEN - 1 - i];
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "[PAIR] authentication complete, device: %s",
                     param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "[PAIR] authentication failed, status: %d",
                     param->auth_cmpl.stat);
        }
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        esp_bt_pin_code_t pin_code = {0};
        ESP_LOGI(TAG, "[PAIR] legacy PIN request (Wii-style binary PIN)");
#if CONFIG_EXAMPLE_WIIMOTE_GUEST_PAIRING_MODE
        /* 1+2 guest pairing: PIN is our own BD_ADDR reversed. */
        esp_bd_addr_t local_bda = {0};
        memcpy(local_bda, esp_bt_dev_get_address(), ESP_BD_ADDR_LEN);
        fill_reversed_bdaddr_pin(local_bda, pin_code);
#else
        /* Sync-button bonding: PIN is the host (Wii) BD_ADDR reversed. */
        fill_reversed_bdaddr_pin(param->pin_req.bda, pin_code);
#endif
        esp_bt_gap_pin_reply(param->pin_req.bda, true, ESP_BD_ADDR_LEN, pin_code);
        break;
    }
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "GAP mode change: %d", param->mode_chg.mode);
        break;
    default:
        break;
    }
}

/* --- HID device profile -------------------------------------------------- */

static void on_hid_connected(const esp_bd_addr_t bda)
{
    s_connected = true;
    wiimote_state_reset();

    ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    /* Prime the host with a default data report and a status report. */
    wiimote_reports_send_data();
    wiimote_reports_send_status();

    /* Once connected, behave like a paired remote: stop advertising. */
    ESP_LOGI(TAG, "[PAIR] HID connected; disabling discoverability");
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
}

static void on_hid_disconnected(void)
{
    s_connected = false;
    ESP_LOGI(TAG, "disconnected; staying connectable for bonded reconnect");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
}

static void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event) {
    case ESP_HIDD_INIT_EVT:
        if (param->init.status == ESP_HIDD_SUCCESS) {
            esp_bt_hid_device_register_app(&s_app_param, &s_qos, &s_qos);
        } else {
            ESP_LOGE(TAG, "init hidd failed!");
        }
        break;
    case ESP_HIDD_REGISTER_APP_EVT:
        if (param->register_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "HID register success; opening pairing window");
            open_pairing_window();
            if (param->register_app.in_use) {
                ESP_LOGI(TAG, "known host found, attempting reconnect");
                esp_bt_hid_device_connect(param->register_app.bd_addr);
            }
        } else {
            ESP_LOGE(TAG, "HID register failed!");
        }
        break;
    case ESP_HIDD_OPEN_EVT:
        if (param->open.status != ESP_HIDD_SUCCESS) {
            ESP_LOGE(TAG, "HID open failed!");
        } else if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED) {
            on_hid_connected(param->open.bd_addr);
        } else if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTING) {
            ESP_LOGI(TAG, "connecting...");
        }
        break;
    case ESP_HIDD_CLOSE_EVT:
        if (param->close.status == ESP_HIDD_SUCCESS &&
            param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
            on_hid_disconnected();
        }
        break;
    case ESP_HIDD_VC_UNPLUG_EVT:
        if (param->vc_unplug.status == ESP_HIDD_SUCCESS &&
            param->vc_unplug.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
            on_hid_disconnected();
        }
        break;
    case ESP_HIDD_SEND_REPORT_EVT:
        if (param->send_report.status != ESP_HIDD_SUCCESS) {
            ESP_LOGE(TAG, "send report failed id:0x%02x reason:%d",
                     param->send_report.report_id, param->send_report.reason);
        }
        break;
    case ESP_HIDD_GET_REPORT_EVT:
        wiimote_reports_handle_get_report(param->get_report.report_type,
                                          param->get_report.report_id);
        break;
    case ESP_HIDD_SET_REPORT_EVT:
        if (param->set_report.report_type == ESP_HIDD_REPORT_TYPE_OUTPUT) {
            wiimote_protocol_handle_output_report(param->set_report.report_id,
                                                  param->set_report.data,
                                                  param->set_report.len);
        }
        break;
    case ESP_HIDD_INTR_DATA_EVT:
        wiimote_protocol_handle_output_report(param->intr_data.report_id,
                                              param->intr_data.data,
                                              param->intr_data.len);
        break;
    default:
        break;
    }
}

/* --- bring-up ------------------------------------------------------------ */

void wiimote_bt_init(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "controller init failed: %s", esp_err_to_name(ret));
        return;
    }
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Wii Remotes use legacy pairing only; SSP must be disabled. */
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bluedroid_cfg.ssp_en = false;
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }
    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "gap register failed: %s", esp_err_to_name(ret));
        return;
    }
    if ((ret = esp_sdp_register_callback(esp_sdp_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "sdp register failed: %s", esp_err_to_name(ret));
        return;
    }
    if ((ret = esp_sdp_init()) != ESP_OK) {
        ESP_LOGE(TAG, "sdp init failed: %s", esp_err_to_name(ret));
        return;
    }

    esp_bt_gap_set_device_name(s_device_name);

    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    cod.minor = ESP_BT_COD_MINOR_PERIPHERAL_JOYSTICK;
    cod.service = 0x01;   /* limited discoverable */
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL);

    /* Let the controller settle before bringing up the HID profile. */
    vTaskDelay(pdMS_TO_TICKS(2000));

    uint16_t descriptor_len = 0;
    s_app_param.name = s_device_name;
    s_app_param.description = s_device_name;
    s_app_param.provider = "Nintendo";
    s_app_param.subclass = ESP_HID_CLASS_JOS;
    s_app_param.desc_list = wiimote_reports_descriptor(&descriptor_len);
    s_app_param.desc_list_len = descriptor_len;
    memset(&s_qos, 0, sizeof(s_qos));

    esp_bt_hid_device_register_callback(esp_bt_hidd_cb);
    esp_bt_hid_device_init();

    /* Legacy pairing only; the PIN is supplied dynamically in the GAP cb. */
    esp_bt_pin_code_t pin_code = {0};
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, pin_code);

    ESP_LOGI(TAG, "Wii Remote identity ready: '%s' (VID=0x%04X PID=0x%04X)",
             s_device_name, WIIMOTE_DI_VENDOR_ID, WIIMOTE_DI_PRODUCT_ID);
}
