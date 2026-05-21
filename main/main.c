/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "esp_log.h"
#include "esp_hidd_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_gap_bt_api.h"
#include "esp_sdp_api.h"
#include <string.h>
#include <stdio.h>

#include "freertos/task.h"
#include "freertos/semphr.h"

#define WIIMOTE_MAX_REPORT_SIZE                (21)
#define WIIMOTE_MIN_REPORT_SIZE                (2)
#define REPORT_BUFFER_SIZE                     WIIMOTE_MAX_REPORT_SIZE
#define WIIMOTE_DI_VENDOR_ID                   (0x057E)
#define WIIMOTE_DI_PRODUCT_ID                  (0x0306)
#define WIIMOTE_DI_PRODUCT_VERSION             (0x0100)
#define WIIMOTE_SYNC_DISCOVERABLE_WINDOW_MS    (20000)

static const char local_device_name[] = CONFIG_EXAMPLE_LOCAL_DEVICE_NAME;

typedef struct {
    esp_hidd_app_param_t app_param;
    esp_hidd_qos_param_t both_qos;
    SemaphoreHandle_t report_mutex;
    uint8_t buffer[REPORT_BUFFER_SIZE];
} local_param_t;

static local_param_t s_local_param = {0};
static int s_di_record_handle = 0;
static bool s_hid_connected = false;
static TaskHandle_t s_pairing_window_task_hdl = NULL;

static void create_wiimote_di_record(void);
static void open_sync_pairing_window(void);

static void pairing_window_task(void *arg)
{
    static const char *TAG = "pairing_window";
    (void)arg;

    /*
     * Real Wii Remote sync-button behavior:
     * - Connectable + discoverable for ~20s
     * - Then stop being discoverable if no host completed pairing/connection
     */
    ESP_LOGI(TAG, "[PAIR][1/4] Entering 20s sync-style discoverable window");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_LIMITED_DISCOVERABLE);
    ESP_LOGI(TAG, "[PAIR][2/4] Waiting for host inquiry/authentication");
    vTaskDelay(pdMS_TO_TICKS(WIIMOTE_SYNC_DISCOVERABLE_WINDOW_MS));

    if (!s_hid_connected) {
        ESP_LOGI(TAG, "[PAIR][3/4] No HID connection within window, disabling discoverability");
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        ESP_LOGI(TAG, "[PAIR][4/4] Pairing window closed; restart device to re-open sync window");
    } else {
        ESP_LOGI(TAG, "[PAIR][3/4] HID link already active, pairing window timer ignored");
    }

    s_pairing_window_task_hdl = NULL;
    vTaskDelete(NULL);
}

static void open_sync_pairing_window(void)
{
    static const char *TAG = "open_pairing_window";

    if (s_pairing_window_task_hdl != NULL) {
        ESP_LOGW(TAG, "pairing window already active");
        return;
    }

    BaseType_t ok = xTaskCreate(pairing_window_task, "pairing_window_task", 3 * 1024,
                                NULL, configMAX_PRIORITIES - 4, &s_pairing_window_task_hdl);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create pairing window task");
    }
}

static void esp_sdp_cb(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param)
{
    static const char *TAG = "esp_sdp_cb";

    switch (event) {
    case ESP_SDP_INIT_EVT:
        if (param->init.status == ESP_SDP_SUCCESS) {
            ESP_LOGI(TAG, "SDP initialized; creating Nintendo DI record");
            create_wiimote_di_record();
        } else {
            ESP_LOGE(TAG, "SDP init failed: %d", param->init.status);
        }
        break;
    case ESP_SDP_CREATE_RECORD_COMP_EVT:
        if (param->create_record.status == ESP_SDP_SUCCESS) {
            s_di_record_handle = param->create_record.record_handle;
            ESP_LOGI(TAG, "DI record created (handle=%d, vid=0x%04X pid=0x%04X ver=0x%04X)",
                     s_di_record_handle, WIIMOTE_DI_VENDOR_ID, WIIMOTE_DI_PRODUCT_ID, WIIMOTE_DI_PRODUCT_VERSION);
        } else {
            ESP_LOGE(TAG, "DI record create failed: %d", param->create_record.status);
        }
        break;
    default:
        break;
    }
}

static void create_wiimote_di_record(void)
{
    static const char *TAG = "create_wiimote_di_record";
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
    } else {
        ESP_LOGI(TAG, "DI record request submitted");
    }
}

#define WIIMOTE_IN_REPORT(id, size) \
    0x85, (id), 0x75, 0x08, 0x95, (size), 0x81, 0x00

#define WIIMOTE_OUT_REPORT(id, size) \
    0x85, (id), 0x75, 0x08, 0x95, (size), 0x91, 0x00

/*
 * Wii Remote-style descriptor: vendor-defined payloads with explicit report IDs/sizes.
 * This mirrors Wiibrew/xwiimote report lengths instead of standard mouse usages.
 */
static uint8_t hid_wiimote_descriptor[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xA1, 0x01,                    // COLLECTION (Application)

    // Output reports (host -> Wiimote)
    WIIMOTE_OUT_REPORT(0x10, 1),   // Rumble
    WIIMOTE_OUT_REPORT(0x11, 1),   // Player LEDs
    WIIMOTE_OUT_REPORT(0x12, 2),   // Data reporting mode
    WIIMOTE_OUT_REPORT(0x13, 1),   // IR enable
    WIIMOTE_OUT_REPORT(0x14, 1),   // Speaker enable
    WIIMOTE_OUT_REPORT(0x15, 1),   // Status request
    WIIMOTE_OUT_REPORT(0x16, 21),  // Write memory/registers
    WIIMOTE_OUT_REPORT(0x17, 6),   // Read memory/registers
    WIIMOTE_OUT_REPORT(0x18, 21),  // Speaker data
    WIIMOTE_OUT_REPORT(0x19, 1),   // Speaker mute
    WIIMOTE_OUT_REPORT(0x1A, 1),   // IR enable 2

    // Input reports (Wiimote -> host)
    WIIMOTE_IN_REPORT(0x20, 6),    // Status
    WIIMOTE_IN_REPORT(0x21, 21),   // Read memory/register data
    WIIMOTE_IN_REPORT(0x22, 4),    // Ack result
    WIIMOTE_IN_REPORT(0x30, 2),    // Buttons
    WIIMOTE_IN_REPORT(0x31, 5),    // Buttons + accel
    WIIMOTE_IN_REPORT(0x32, 10),   // Buttons + extension
    WIIMOTE_IN_REPORT(0x33, 17),   // Buttons + accel + IR
    WIIMOTE_IN_REPORT(0x34, 21),   // Buttons + extension
    WIIMOTE_IN_REPORT(0x35, 21),   // Buttons + accel + extension
    WIIMOTE_IN_REPORT(0x36, 21),   // Buttons + IR + extension
    WIIMOTE_IN_REPORT(0x37, 21),   // Buttons + accel + IR + extension
    WIIMOTE_IN_REPORT(0x3D, 21),   // Extension only
    WIIMOTE_IN_REPORT(0x3E, 21),   // Interleaved mode 1
    WIIMOTE_IN_REPORT(0x3F, 21),   // Interleaved mode 2

    0xC0                           // END_COLLECTION
};

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static const uint16_t hid_wiimote_descriptor_len = sizeof(hid_wiimote_descriptor);

static uint8_t get_input_report_size(uint8_t report_id)
{
    switch (report_id) {
    case 0x20: return 6;
    case 0x21: return 21;
    case 0x22: return 4;
    case 0x30: return 2;
    case 0x31: return 5;
    case 0x32: return 10;
    case 0x33: return 17;
    case 0x34: return 21;
    case 0x35: return 21;
    case 0x36: return 21;
    case 0x37: return 21;
    case 0x3D: return 21;
    case 0x3E: return 21;
    case 0x3F: return 21;
    default:
        return 0;
    }
}

static void fill_reversed_bdaddr_pin(const esp_bd_addr_t source_addr, esp_bt_pin_code_t pin_code)
{
    for (size_t i = 0; i < ESP_BD_ADDR_LEN; i++) {
        pin_code[i] = source_addr[ESP_BD_ADDR_LEN - 1 - i];
    }
}

static void send_default_input_report(void)
{
    const uint8_t report_id = 0x30; // default DRM after power-up
    const uint8_t report_size = get_input_report_size(report_id);

    xSemaphoreTake(s_local_param.report_mutex, portMAX_DELAY);
    memset(s_local_param.buffer, 0, report_size);
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, report_id, report_size, s_local_param.buffer);
    xSemaphoreGive(s_local_param.report_mutex);
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "esp_bt_gap_cb";
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "[PAIR] Authentication complete (legacy mode), device: %s", param->auth_cmpl.device_name);
            ESP_LOG_BUFFER_HEX(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG, "[PAIR] Authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        esp_bt_pin_code_t pin_code = {0};
        uint8_t pin_len = ESP_BD_ADDR_LEN;
        esp_bd_addr_t local_bda = {0};
        memcpy(local_bda, esp_bt_dev_get_address(), ESP_BD_ADDR_LEN);
        ESP_LOGI(TAG, "[PAIR] Legacy PIN request received (Wii-style binary PIN), min_16_digit=%d",
                 param->pin_req.min_16_digit);
#if CONFIG_EXAMPLE_WIIMOTE_GUEST_PAIRING_MODE
        // 1+2 guest pairing: PIN is reversed Wiimote BD_ADDR (our local adapter).
        fill_reversed_bdaddr_pin(local_bda, pin_code);
        ESP_LOGI(TAG, "[PAIR] Using guest PIN source: reverse local BD_ADDR");
#else
        // Sync-button bonding: PIN is reversed host BD_ADDR (requesting peer).
        fill_reversed_bdaddr_pin(param->pin_req.bda, pin_code);
        ESP_LOGI(TAG, "[PAIR] Using sync PIN source: reverse host BD_ADDR");
#endif
        ESP_LOG_BUFFER_HEX(TAG, pin_code, ESP_BD_ADDR_LEN);
        esp_bt_gap_pin_reply(param->pin_req.bda, true, pin_len, pin_code);
        break;
    }
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;
    default:
        ESP_LOGI(TAG, "event: %d", event);
        break;
    }
    return;
}

void bt_app_task_start_up(void)
{
    s_local_param.report_mutex = xSemaphoreCreateMutex();
    memset(s_local_param.buffer, 0, REPORT_BUFFER_SIZE);
}

void bt_app_task_shut_down(void)
{
    if (s_local_param.report_mutex) {
        vSemaphoreDelete(s_local_param.report_mutex);
        s_local_param.report_mutex = NULL;
    }
}

void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    static const char *TAG = "esp_bt_hidd_cb";
    switch (event) {
    case ESP_HIDD_INIT_EVT:
        if (param->init.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "setting hid parameters");
            esp_bt_hid_device_register_app(&s_local_param.app_param, &s_local_param.both_qos, &s_local_param.both_qos);
        } else {
            ESP_LOGE(TAG, "init hidd failed!");
        }
        break;
    case ESP_HIDD_DEINIT_EVT:
        break;
    case ESP_HIDD_REGISTER_APP_EVT:
        if (param->register_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "HID register success; opening sync-style pairing window");
            open_sync_pairing_window();
            if (param->register_app.in_use) {
                ESP_LOGI(TAG, "known virtual cable host found, attempting reconnect");
                esp_bt_hid_device_connect(param->register_app.bd_addr);
            }
        } else {
            ESP_LOGE(TAG, "setting hid parameters failed!");
        }
        break;
    case ESP_HIDD_UNREGISTER_APP_EVT:
        if (param->unregister_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "unregister app success!");
        } else {
            ESP_LOGE(TAG, "unregister app failed!");
        }
        break;
    case ESP_HIDD_OPEN_EVT:
        if (param->open.status == ESP_HIDD_SUCCESS) {
            if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTING) {
                ESP_LOGI(TAG, "connecting...");
            } else if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED) {
                s_hid_connected = true;
                ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0],
                         param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4],
                         param->open.bd_addr[5]);
                bt_app_task_start_up();
                send_default_input_report();
                /*
                 * Once connected, stop inquiry/page scans just like a paired remote that is
                 * no longer accepting new pairing attempts while actively connected.
                 */
                ESP_LOGI(TAG, "[PAIR] HID connected; disabling discoverability/connectability");
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "open failed!");
        }
        break;
    case ESP_HIDD_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_CLOSE_EVT");
        if (param->close.status == ESP_HIDD_SUCCESS) {
            if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTING) {
                ESP_LOGI(TAG, "disconnecting...");
            } else if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                s_hid_connected = false;
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                /*
                 * After a bonded disconnect, remain connectable but non-discoverable.
                 * A fresh sync pairing window is intentionally only opened on boot/reset.
                 */
                ESP_LOGI(TAG, "[PAIR] Disconnected; staying connectable/non-discoverable for bonded host reconnect");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    case ESP_HIDD_SEND_REPORT_EVT:
        if (param->send_report.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d", param->send_report.report_id,
                     param->send_report.report_type);
        } else {
            ESP_LOGE(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d, status:%d, reason:%d",
                     param->send_report.report_id, param->send_report.report_type, param->send_report.status,
                     param->send_report.reason);
        }
        break;
    case ESP_HIDD_REPORT_ERR_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_REPORT_ERR_EVT");
        break;
    case ESP_HIDD_GET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_GET_REPORT_EVT id:0x%02x, type:%d, size:%d", param->get_report.report_id,
                 param->get_report.report_type, param->get_report.buffer_size);
        if (param->get_report.report_type != ESP_HIDD_REPORT_TYPE_INPUT) {
            esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM);
            break;
        }
        {
            const uint8_t report_id = param->get_report.report_id;
            const uint8_t report_len = get_input_report_size(report_id);
            if (report_len < WIIMOTE_MIN_REPORT_SIZE || report_len > WIIMOTE_MAX_REPORT_SIZE) {
                ESP_LOGE(TAG, "unsupported report id:0x%02x", report_id);
                esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID);
                break;
            }
            xSemaphoreTake(s_local_param.report_mutex, portMAX_DELAY);
            memset(s_local_param.buffer, 0, report_len);
            esp_bt_hid_device_send_report(param->get_report.report_type, report_id, report_len, s_local_param.buffer);
            xSemaphoreGive(s_local_param.report_mutex);
        }
        break;
    case ESP_HIDD_SET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_REPORT_EVT id:0x%02x, type:%d, len:%d",
                 param->set_report.report_id, param->set_report.report_type, param->set_report.len);
        if (param->set_report.report_type == ESP_HIDD_REPORT_TYPE_OUTPUT &&
            param->set_report.report_id == 0x15) {
            // Host status request; send basic status report with zero payload.
            const uint8_t status_report_id = 0x20;
            const uint8_t status_report_len = get_input_report_size(status_report_id);
            xSemaphoreTake(s_local_param.report_mutex, portMAX_DELAY);
            memset(s_local_param.buffer, 0, status_report_len);
            esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, status_report_id,
                                          status_report_len, s_local_param.buffer);
            xSemaphoreGive(s_local_param.report_mutex);
        } else if (param->set_report.report_type == ESP_HIDD_REPORT_TYPE_OUTPUT) {
            // Generic acknowledgement with success result for supported output path.
            const uint8_t ack_report_id = 0x22;
            const uint8_t ack_report_len = get_input_report_size(ack_report_id);
            xSemaphoreTake(s_local_param.report_mutex, portMAX_DELAY);
            memset(s_local_param.buffer, 0, ack_report_len);
            s_local_param.buffer[2] = param->set_report.report_id;
            s_local_param.buffer[3] = 0x00;
            esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, ack_report_id,
                                          ack_report_len, s_local_param.buffer);
            xSemaphoreGive(s_local_param.report_mutex);
        }
        break;
    case ESP_HIDD_SET_PROTOCOL_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_PROTOCOL_EVT mode:%d (ignored for Wii profile)",
                 param->set_protocol.protocol_mode);
        break;
    case ESP_HIDD_INTR_DATA_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_INTR_DATA_EVT");
        break;
    case ESP_HIDD_VC_UNPLUG_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_VC_UNPLUG_EVT");
        if (param->vc_unplug.status == ESP_HIDD_SUCCESS) {
            if (param->vc_unplug.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                s_hid_connected = false;
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                ESP_LOGI(TAG, "[PAIR] Virtual cable unplug; staying connectable/non-discoverable");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    default:
        break;
    }
}

void app_main(void)
{
    const char *TAG = "app_main";
    esp_err_t ret;
    char bda_str[18] = {0};

    ESP_LOGI(TAG, "=== Wii Remote emulation boot ===");
    ESP_LOGI(TAG, "Target identity: name='%s', VID=0x%04X, PID=0x%04X",
             local_device_name, WIIMOTE_DI_VENDOR_ID, WIIMOTE_DI_PRODUCT_ID);

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "initialize controller failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "enable controller failed: %s", esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    // Wii Remote protocol requires legacy pairing; SSP is not supported.
    bluedroid_cfg.ssp_en = false;
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "enable bluedroid failed: %s", esp_err_to_name(ret));
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

    ESP_LOGI(TAG, "setting device name");
    esp_bt_gap_set_device_name(local_device_name);

    ESP_LOGI(TAG, "setting Wii Remote class of device (0x002504)");
    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    cod.minor = ESP_BT_COD_MINOR_PERIPHERAL_JOYSTICK;
    cod.service = 0x01; // limited discoverable service bit
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL);

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Initialize HID SDP information and L2CAP parameters.
    // to be used in the call of `esp_bt_hid_device_register_app` after profile initialization finishes
    do {
        s_local_param.app_param.name = local_device_name;
        s_local_param.app_param.description = local_device_name;
        s_local_param.app_param.provider = "Nintendo";
        s_local_param.app_param.subclass = ESP_HID_CLASS_JOS;
        s_local_param.app_param.desc_list = hid_wiimote_descriptor;
        s_local_param.app_param.desc_list_len = hid_wiimote_descriptor_len;

        memset(&s_local_param.both_qos, 0, sizeof(esp_hidd_qos_param_t)); // don't set the qos parameters
    } while (0);

    ESP_LOGI(TAG, "register hid device callback");
    esp_bt_hid_device_register_callback(esp_bt_hidd_cb);

    ESP_LOGI(TAG, "starting hid device");
    esp_bt_hid_device_init();

    /*
     * Legacy pairing only; PIN supplied dynamically in GAP PIN_REQ callback
     * according to Wii sync/guest pairing behavior.
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code = {0};
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
    ESP_LOGI(TAG, "Pairing behavior: sync-style window opens after HID register success");
    ESP_LOGI(TAG, "Use reset/reboot to simulate pressing red sync on a physical Wiimote");
}
