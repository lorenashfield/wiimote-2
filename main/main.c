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
#include "driver/uart.h"
 #include <string.h>
 #include <stdio.h>
 #include <inttypes.h>
 
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 
 #define WIIMOTE_MAX_REPORT_SIZE                (21)
 #define WIIMOTE_MIN_REPORT_SIZE                (2)
 #define REPORT_BUFFER_SIZE                     WIIMOTE_MAX_REPORT_SIZE
 #define WIIMOTE_DI_VENDOR_ID                   (0x057E)
 #define WIIMOTE_DI_PRODUCT_ID                  (0x0306)
 #define WIIMOTE_DI_PRODUCT_VERSION             (0x0100)
 #define WIIMOTE_SYNC_DISCOVERABLE_WINDOW_MS    (20000)
 #define WIIMOTE_EEPROM_SIZE                    (0x1700)
 #define WIIMOTE_REGISTER_BLOCK_SIZE            (0x100)
#define WIIMOTE_UART_PORT                      (UART_NUM_0)
#define WIIMOTE_UART_PULSE_MS                  (120)

// Hardcoded debug toggles.
#define WIIMOTE_ENABLE_IO_DEBUG                (0)
#define WIIMOTE_ENABLE_UART_BUTTON_DEBUG       (1)
 
 #define WIIMOTE_REG_BASE_SPEAKER               (0xA20000)
 #define WIIMOTE_REG_BASE_EXTENSION             (0xA40000)
 #define WIIMOTE_REG_BASE_MOTION_PLUS           (0xA60000)
 #define WIIMOTE_REG_BASE_IR                    (0xB00000)
 
 #define WIIMOTE_READ_ERROR_SUCCESS             (0x00)
 #define WIIMOTE_READ_ERROR_NACK                (0x07)
 #define WIIMOTE_READ_ERROR_INVALID_ADDRESS     (0x08)

#define WIIMOTE_BTN_LEFT_MASK                  (0x0100)
#define WIIMOTE_BTN_RIGHT_MASK                 (0x0200)
#define WIIMOTE_BTN_DOWN_MASK                  (0x0400)
#define WIIMOTE_BTN_UP_MASK                    (0x0800)
#define WIIMOTE_BTN_HOME_MASK                  (0x0080)
 
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
 static TaskHandle_t s_data_report_task_hdl = NULL;
static TaskHandle_t s_uart_input_task_hdl = NULL;
 static uint8_t s_status_flags = 0x00;
 static const uint8_t s_full_battery_level = 0xFF;
#if WIIMOTE_ENABLE_IO_DEBUG
static const char *WIIMOTE_PACKET_TAG = "wiimote_pkt";
#endif
 static uint8_t s_emulated_eeprom[WIIMOTE_EEPROM_SIZE] = {0};
 static uint8_t s_reg_speaker[WIIMOTE_REGISTER_BLOCK_SIZE] = {0};
 static uint8_t s_reg_extension[WIIMOTE_REGISTER_BLOCK_SIZE] = {0};
 static uint8_t s_reg_motion_plus[WIIMOTE_REGISTER_BLOCK_SIZE] = {0};
 static uint8_t s_reg_ir[WIIMOTE_REGISTER_BLOCK_SIZE] = {0};
 static uint8_t s_reporting_mode = 0x30;
 static bool s_reporting_continuous = false;
 static bool s_rumble_enabled = false;
static uint16_t s_button_state = 0x0000;
 
 static void create_wiimote_di_record(void);
 static void open_sync_pairing_window(void);
 static void send_status_report(void);
 static void update_status_from_output_report(uint8_t report_id, const uint8_t *payload, uint16_t len);
 static bool output_report_requests_ack(const uint8_t *payload, uint16_t len);
 static void send_ack_report(uint8_t output_report_id);
 static void send_ack_report_with_error(uint8_t output_report_id, uint8_t error_code);
 static void log_hid_packet(const char *direction, const char *context, esp_hidd_report_type_t report_type,
                            uint8_t report_id, uint16_t len, const uint8_t *payload);
 static void send_input_report_with_trace(esp_hidd_report_type_t report_type, uint8_t report_id,
                                          uint8_t len, uint8_t *payload, const char *context);
 static void init_emulated_memory(void);
 static bool read_emulated_memory(uint32_t offset, bool register_space, uint8_t *out, uint16_t len, uint8_t *error);
 static bool write_emulated_memory(uint32_t offset, bool register_space, const uint8_t *in, uint16_t len, uint8_t *error);
 static void send_read_memory_data_report(uint16_t offset_low16, uint8_t chunk_len, uint8_t error, const uint8_t *data);
 static void handle_read_memory_request(const uint8_t *payload, uint16_t len);
 static uint8_t handle_write_memory_request(const uint8_t *payload, uint16_t len);
 static void send_report_for_mode(uint8_t report_id, const char *context);
 static void handle_output_report_command(uint8_t report_id, const uint8_t *payload, uint16_t len, const char *source);
 static void data_report_task(void *arg);
static void uart_input_task(void *arg);
static void send_button_report_pulse(uint16_t button_mask, const char *button_name);
static void set_button_state_mask(uint16_t mask, bool pressed);
static uint16_t get_button_state(void);
 
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
 
 static void init_emulated_memory(void)
 {
     static const uint8_t calibration_block[] = {
         0xA1, 0xAA, 0x8B, 0x99, 0xAE, 0x9E, 0x78, 0x30, 0xA7, 0x74, 0xD3, 0xA1, 0xAA, 0x8B, 0x99, 0xAE,
         0x9E, 0x78, 0x30, 0xA7, 0x74, 0xD3, 0x82, 0x82, 0x82, 0x15, 0x9C, 0x9C, 0x9E, 0x38, 0x40, 0x3E,
         0x82, 0x82, 0x82, 0x15, 0x9C, 0x9C, 0x9E, 0x38, 0x40, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
     };
     static const uint8_t unknown_tail[] = {
         0x00, 0x00, 0x00, 0xFF, 0x11, 0xEE, 0x00, 0x00, 0x33, 0xCC, 0x44, 0xBB, 0x00, 0x00, 0x66, 0x99,
         0x77, 0x88, 0x00, 0x00, 0x2B, 0x01, 0xE8, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
     };
 
     memset(s_emulated_eeprom, 0, sizeof(s_emulated_eeprom));
     memset(s_reg_speaker, 0, sizeof(s_reg_speaker));
     memset(s_reg_extension, 0, sizeof(s_reg_extension));
     memset(s_reg_motion_plus, 0, sizeof(s_reg_motion_plus));
     memset(s_reg_ir, 0, sizeof(s_reg_ir));
 
     memcpy(&s_emulated_eeprom[0x0000], calibration_block, sizeof(calibration_block));
     memcpy(&s_emulated_eeprom[0x16D0], unknown_tail, sizeof(unknown_tail));
 }
 
 static bool read_emulated_memory(uint32_t offset, bool register_space, uint8_t *out, uint16_t len, uint8_t *error)
 {
     const uint16_t low16 = (uint16_t)(offset & 0xFFFF);
 
     if (out == NULL || len == 0 || error == NULL) {
         return false;
     }
 
     *error = WIIMOTE_READ_ERROR_SUCCESS;
 
     if (!register_space) {
         if ((uint32_t)low16 + len > WIIMOTE_EEPROM_SIZE) {
             *error = WIIMOTE_READ_ERROR_INVALID_ADDRESS;
             memset(out, 0, len);
             return false;
         }
         memcpy(out, &s_emulated_eeprom[low16], len);
         return true;
     }
 
     uint8_t *reg_block = NULL;
     switch (offset & 0xFF0000) {
     case WIIMOTE_REG_BASE_SPEAKER:
         reg_block = s_reg_speaker;
         break;
     case WIIMOTE_REG_BASE_EXTENSION:
         reg_block = s_reg_extension;
         break;
     case WIIMOTE_REG_BASE_MOTION_PLUS:
         reg_block = s_reg_motion_plus;
         break;
     case WIIMOTE_REG_BASE_IR:
         reg_block = s_reg_ir;
         break;
     default:
         *error = WIIMOTE_READ_ERROR_INVALID_ADDRESS;
         memset(out, 0, len);
         return false;
     }
 
     const uint16_t reg_offset = (uint16_t)(offset & 0x00FF);
     if ((uint32_t)reg_offset + len > WIIMOTE_REGISTER_BLOCK_SIZE) {
         *error = WIIMOTE_READ_ERROR_NACK;
         memset(out, 0, len);
         return false;
     }
 
     memcpy(out, &reg_block[reg_offset], len);
     return true;
 }
 
 static bool write_emulated_memory(uint32_t offset, bool register_space, const uint8_t *in, uint16_t len, uint8_t *error)
 {
     const uint16_t low16 = (uint16_t)(offset & 0xFFFF);
 
     if (in == NULL || len == 0 || error == NULL) {
         return false;
     }
 
     *error = WIIMOTE_READ_ERROR_SUCCESS;
 
     if (!register_space) {
         if ((uint32_t)low16 + len > WIIMOTE_EEPROM_SIZE) {
             *error = WIIMOTE_READ_ERROR_INVALID_ADDRESS;
             return false;
         }
         memcpy(&s_emulated_eeprom[low16], in, len);
         return true;
     }
 
     uint8_t *reg_block = NULL;
     switch (offset & 0xFF0000) {
     case WIIMOTE_REG_BASE_SPEAKER:
         reg_block = s_reg_speaker;
         break;
     case WIIMOTE_REG_BASE_EXTENSION:
         reg_block = s_reg_extension;
         break;
     case WIIMOTE_REG_BASE_MOTION_PLUS:
         reg_block = s_reg_motion_plus;
         break;
     case WIIMOTE_REG_BASE_IR:
         reg_block = s_reg_ir;
         break;
     default:
         *error = WIIMOTE_READ_ERROR_INVALID_ADDRESS;
         return false;
     }
 
     const uint16_t reg_offset = (uint16_t)(offset & 0x00FF);
     if ((uint32_t)reg_offset + len > WIIMOTE_REGISTER_BLOCK_SIZE) {
         *error = WIIMOTE_READ_ERROR_NACK;
         return false;
     }
 
     memcpy(&reg_block[reg_offset], in, len);
     return true;
 }

static void set_button_state_mask(uint16_t mask, bool pressed)
{
    if (pressed) {
        s_button_state |= mask;
    } else {
        s_button_state &= (uint16_t)~mask;
    }
}

static uint16_t get_button_state(void)
{
    return s_button_state;
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
     send_input_report_with_trace(ESP_HIDD_REPORT_TYPE_INTRDATA, report_id, report_size, s_local_param.buffer,
                                  "default_data_report");
     xSemaphoreGive(s_local_param.report_mutex);
 }
 
 static void log_hid_packet(const char *direction, const char *context, esp_hidd_report_type_t report_type,
                            uint8_t report_id, uint16_t len, const uint8_t *payload)
 {
#if !WIIMOTE_ENABLE_IO_DEBUG
    (void)direction;
    (void)context;
    (void)report_type;
    (void)report_id;
    (void)len;
    (void)payload;
    return;
#else
     ESP_LOGI(WIIMOTE_PACKET_TAG,
              "[%s] ctx=%s type=%d id=0x%02X len=%u",
              direction,
              (context != NULL) ? context : "n/a",
              report_type,
              report_id,
              (unsigned int)len);
 
     if (payload != NULL && len > 0) {
         ESP_LOG_BUFFER_HEX_LEVEL(WIIMOTE_PACKET_TAG, payload, len, ESP_LOG_INFO);
     } else {
         ESP_LOGI(WIIMOTE_PACKET_TAG, "[%s] no payload bytes", direction);
     }
#endif
 }
 
 static void send_input_report_with_trace(esp_hidd_report_type_t report_type, uint8_t report_id,
                                          uint8_t len, uint8_t *payload, const char *context)
 {
     log_hid_packet("TX", context, report_type, report_id, len, payload);
     esp_bt_hid_device_send_report(report_type, report_id, len, payload);
 }
 
 static void build_status_report_payload(uint8_t *payload, size_t len)
 {
     if (payload == NULL || len < 6) {
         return;
     }
 
     memset(payload, 0, len);
    const uint16_t buttons = get_button_state();
    payload[0] = (uint8_t)((buttons >> 8) & 0xFF);
    payload[1] = (uint8_t)(buttons & 0xFF);
     payload[2] = s_status_flags;
     payload[5] = s_full_battery_level;
 }
 
 static void send_status_report(void)
 {
     const uint8_t status_report_id = 0x20;
     const uint8_t status_report_len = get_input_report_size(status_report_id);
 
     xSemaphoreTake(s_local_param.report_mutex, portMAX_DELAY);
     build_status_report_payload(s_local_param.buffer, status_report_len);
     send_input_report_with_trace(ESP_HIDD_REPORT_TYPE_INTRDATA, status_report_id,
                                  status_report_len, s_local_param.buffer, "status_request_response");
     xSemaphoreGive(s_local_param.report_mutex);
 }
 
 static bool output_report_requests_ack(const uint8_t *payload, uint16_t len)
 {
     if (payload == NULL || len == 0) {
         return false;
     }
 
     return (payload[0] & 0x02) != 0;
 }
 
 static void send_ack_report(uint8_t output_report_id)
 {
     send_ack_report_with_error(output_report_id, 0x00);
 }
 
 static void send_ack_report_with_error(uint8_t output_report_id, uint8_t error_code)
 {
     const uint8_t ack_report_id = 0x22;
     const uint8_t ack_report_len = get_input_report_size(ack_report_id);
 
     xSemaphoreTake(s_local_param.report_mutex, portMAX_DELAY);
     memset(s_local_param.buffer, 0, ack_report_len);
    const uint16_t buttons = get_button_state();
    s_local_param.buffer[0] = (uint8_t)((buttons >> 8) & 0xFF);
    s_local_param.buffer[1] = (uint8_t)(buttons & 0xFF);
     s_local_param.buffer[2] = output_report_id;
     s_local_param.buffer[3] = error_code;
     send_input_report_with_trace(ESP_HIDD_REPORT_TYPE_INTRDATA, ack_report_id,
                                  ack_report_len, s_local_param.buffer, "output_ack");
     xSemaphoreGive(s_local_param.report_mutex);
 }
 
 static void send_read_memory_data_report(uint16_t offset_low16, uint8_t chunk_len, uint8_t error, const uint8_t *data)
 {
     const uint8_t read_report_id = 0x21;
     const uint8_t read_report_len = get_input_report_size(read_report_id);
 
     if (chunk_len == 0 || chunk_len > 16 || data == NULL) {
         return;
     }
 
     xSemaphoreTake(s_local_param.report_mutex, portMAX_DELAY);
     memset(s_local_param.buffer, 0, read_report_len);
    const uint16_t buttons = get_button_state();
    s_local_param.buffer[0] = (uint8_t)((buttons >> 8) & 0xFF);
    s_local_param.buffer[1] = (uint8_t)(buttons & 0xFF);
     // High nibble stores (size - 1), low nibble stores error code.
     s_local_param.buffer[2] = (uint8_t)(((chunk_len - 1U) << 4) | (error & 0x0F));
     s_local_param.buffer[3] = (uint8_t)((offset_low16 >> 8) & 0xFF);
     s_local_param.buffer[4] = (uint8_t)(offset_low16 & 0xFF);
     memcpy(&s_local_param.buffer[5], data, chunk_len);
     send_input_report_with_trace(ESP_HIDD_REPORT_TYPE_INTRDATA, read_report_id,
                                  read_report_len, s_local_param.buffer, "read_memory_response");
     xSemaphoreGive(s_local_param.report_mutex);
 }
 
 static void handle_read_memory_request(const uint8_t *payload, uint16_t len)
 {
     static const char *TAG = "wiimote_read";
    (void)TAG;
     uint8_t read_data[16] = {0};
 
     if (payload == NULL || len < 6) {
         ESP_LOGW(TAG, "invalid read-memory request len=%u", (unsigned int)len);
         return;
     }
 
     const uint8_t control = payload[0];
     const bool reg_space_bit2 = (control & 0x04) != 0;
     const bool reg_space_bit3 = (control & 0x08) != 0;
     const bool register_space = reg_space_bit2 || reg_space_bit3;
     const uint32_t offset = ((uint32_t)payload[1] << 16) | ((uint32_t)payload[2] << 8) | (uint32_t)payload[3];
     uint16_t remaining = ((uint16_t)payload[4] << 8) | (uint16_t)payload[5];
     uint32_t current_offset = offset;
     uint8_t error = WIIMOTE_READ_ERROR_SUCCESS;
 
     if (remaining == 0) {
        #if WIIMOTE_ENABLE_IO_DEBUG
        ESP_LOGW(TAG, "read request with zero size ignored");
        #endif
         return;
     }
 
     if (reg_space_bit2 && reg_space_bit3) {
         error = WIIMOTE_READ_ERROR_INVALID_ADDRESS;
         memset(read_data, 0, sizeof(read_data));
         send_read_memory_data_report((uint16_t)(offset & 0xFFFF), 16, error, read_data);
        #if WIIMOTE_ENABLE_IO_DEBUG
        ESP_LOGW(TAG, "invalid read control flags=0x%02x", control);
        #endif
         return;
     }
 
    #if WIIMOTE_ENABLE_IO_DEBUG
    ESP_LOGI(TAG, "read request offset=0x%06" PRIX32 " size=%u space=%s", offset, (unsigned int)remaining,
             register_space ? "register" : "eeprom");
    #endif
 
     while (remaining > 0) {
         const uint8_t chunk_len = (remaining > 16U) ? 16U : (uint8_t)remaining;
         memset(read_data, 0, sizeof(read_data));
         if (!read_emulated_memory(current_offset, register_space, read_data, chunk_len, &error)) {
             send_read_memory_data_report((uint16_t)(current_offset & 0xFFFF), 16, error, read_data);
            #if WIIMOTE_ENABLE_IO_DEBUG
            ESP_LOGW(TAG, "read error at offset=0x%06" PRIX32 " error=0x%02x", current_offset, error);
            #endif
             return;
         }
 
         send_read_memory_data_report((uint16_t)(current_offset & 0xFFFF), chunk_len, WIIMOTE_READ_ERROR_SUCCESS,
                                      read_data);
         current_offset += chunk_len;
         remaining = (uint16_t)(remaining - chunk_len);
     }
 }
 
 static uint8_t handle_write_memory_request(const uint8_t *payload, uint16_t len)
 {
     static const char *TAG = "wiimote_write";
    (void)TAG;
 
     if (payload == NULL || len < 5) {
        #if WIIMOTE_ENABLE_IO_DEBUG
        ESP_LOGW(TAG, "invalid write-memory request len=%u", (unsigned int)len);
        #endif
         return WIIMOTE_READ_ERROR_NACK;
     }
 
     const uint8_t control = payload[0];
     const bool reg_space_bit2 = (control & 0x04) != 0;
     const bool reg_space_bit3 = (control & 0x08) != 0;
     const bool register_space = reg_space_bit2 || reg_space_bit3;
     const uint32_t offset = ((uint32_t)payload[1] << 16) | ((uint32_t)payload[2] << 8) | (uint32_t)payload[3];
     const uint8_t write_size = payload[4];
     uint8_t error = WIIMOTE_READ_ERROR_SUCCESS;
 
     if (reg_space_bit2 && reg_space_bit3) {
        #if WIIMOTE_ENABLE_IO_DEBUG
        ESP_LOGW(TAG, "invalid write control flags=0x%02x", control);
        #endif
         return WIIMOTE_READ_ERROR_INVALID_ADDRESS;
     }
     if (write_size == 0 || write_size > 16) {
        #if WIIMOTE_ENABLE_IO_DEBUG
        ESP_LOGW(TAG, "invalid write size=%u ignored", write_size);
        #endif
         return WIIMOTE_READ_ERROR_NACK;
     }
     if ((uint32_t)len < (uint32_t)5 + write_size) {
        #if WIIMOTE_ENABLE_IO_DEBUG
        ESP_LOGW(TAG, "short write payload len=%u size=%u", (unsigned int)len, write_size);
        #endif
         return WIIMOTE_READ_ERROR_NACK;
     }
 
     if (!write_emulated_memory(offset, register_space, &payload[5], write_size, &error)) {
        #if WIIMOTE_ENABLE_IO_DEBUG
        ESP_LOGW(TAG, "write error offset=0x%06" PRIX32 " size=%u error=0x%02x", offset, write_size, error);
        #endif
         return error;
     }
 
    #if WIIMOTE_ENABLE_IO_DEBUG
    ESP_LOGI(TAG, "write request offset=0x%06" PRIX32 " size=%u space=%s", offset, write_size,
             register_space ? "register" : "eeprom");
    #endif
     return WIIMOTE_READ_ERROR_SUCCESS;
 }
 
 static void send_report_for_mode(uint8_t report_id, const char *context)
 {
     const uint8_t report_size = get_input_report_size(report_id);
     if (report_size < WIIMOTE_MIN_REPORT_SIZE || report_size > WIIMOTE_MAX_REPORT_SIZE) {
         return;
     }
 
     xSemaphoreTake(s_local_param.report_mutex, portMAX_DELAY);
     memset(s_local_param.buffer, 0, report_size);
    const uint16_t buttons = get_button_state();
    if (report_size >= 2) {
        s_local_param.buffer[0] = (uint8_t)((buttons >> 8) & 0xFF);
        s_local_param.buffer[1] = (uint8_t)(buttons & 0xFF);
    }
     send_input_report_with_trace(ESP_HIDD_REPORT_TYPE_INTRDATA, report_id, report_size, s_local_param.buffer, context);
     xSemaphoreGive(s_local_param.report_mutex);
 }
 
 static void data_report_task(void *arg)
 {
     (void)arg;
     while (s_hid_connected) {
         /*
          * Real Wiimote behavior is change-driven unless continuous mode is enabled.
          * For compatibility with some Wii init paths, send a steady heartbeat in the
          * selected reporting mode while connected.
          */
         send_report_for_mode(s_reporting_mode, "streaming_data_report");
         vTaskDelay(pdMS_TO_TICKS(100));
     }
 
     s_data_report_task_hdl = NULL;
     vTaskDelete(NULL);
 }

static void send_button_report_pulse(uint16_t button_mask, const char *button_name)
{
    if (!s_hid_connected) {
        return;
    }

    set_button_state_mask(button_mask, true);
    send_report_for_mode(s_reporting_mode, "uart_button_press");
#if WIIMOTE_ENABLE_UART_BUTTON_DEBUG
    ESP_LOGI("wiimote_uart", "button press: %s", button_name);
#endif
    vTaskDelay(pdMS_TO_TICKS(WIIMOTE_UART_PULSE_MS));

    set_button_state_mask(button_mask, false);
    send_report_for_mode(s_reporting_mode, "uart_button_release");
#if WIIMOTE_ENABLE_UART_BUTTON_DEBUG
    ESP_LOGI("wiimote_uart", "button release: %s", button_name);
#endif
}

static void uart_input_task(void *arg)
{
    (void)arg;
    uint8_t rx_byte = 0;
    uint8_t esc_state = 0; // 0: none, 1: got ESC, 2: got ESC[

    while (s_hid_connected) {
        int read = uart_read_bytes(WIIMOTE_UART_PORT, &rx_byte, 1, pdMS_TO_TICKS(50));
        if (read <= 0) {
            continue;
        }

        if (esc_state == 0) {
            if (rx_byte == 0x1B) {
                esc_state = 1;
                continue;
            }
            if (rx_byte == 'h' || rx_byte == 'H') {
                send_button_report_pulse(WIIMOTE_BTN_HOME_MASK, "HOME");
            }
            continue;
        }

        if (esc_state == 1) {
            esc_state = (rx_byte == '[') ? 2 : 0;
            continue;
        }

        // esc_state == 2, expect arrow final byte.
        switch (rx_byte) {
        case 'A':
            send_button_report_pulse(WIIMOTE_BTN_UP_MASK, "DPAD_UP");
            break;
        case 'B':
            send_button_report_pulse(WIIMOTE_BTN_DOWN_MASK, "DPAD_DOWN");
            break;
        case 'C':
            send_button_report_pulse(WIIMOTE_BTN_RIGHT_MASK, "DPAD_RIGHT");
            break;
        case 'D':
            send_button_report_pulse(WIIMOTE_BTN_LEFT_MASK, "DPAD_LEFT");
            break;
        default:
            break;
        }
        esc_state = 0;
    }

    s_uart_input_task_hdl = NULL;
    vTaskDelete(NULL);
}
 
 static void handle_output_report_command(uint8_t report_id, const uint8_t *payload, uint16_t len, const char *source)
 {
     static const char *TAG = "wiimote_cmd";
    (void)TAG;
 
     update_status_from_output_report(report_id, payload, len);
 
     switch (report_id) {
     case 0x10:
         s_rumble_enabled = (payload != NULL && len > 0) ? ((payload[0] & 0x01) != 0) : s_rumble_enabled;
         break;
     case 0x12:
         if (payload != NULL && len >= 2) {
             s_reporting_continuous = (payload[0] & 0x04) != 0;
             s_reporting_mode = payload[1];
             send_report_for_mode(s_reporting_mode, "report_mode_change");
         }
         break;
     case 0x15:
         // Host status request: return current flags + full battery level.
         send_status_report();
         break;
     case 0x16:
         send_ack_report_with_error(report_id, handle_write_memory_request(payload, len));
         break;
     case 0x17:
         // Host read-memory request (used heavily during Wii-side probe/init).
         handle_read_memory_request(payload, len);
         break;
     default:
         break;
     }
 
     if (report_id != 0x16 && output_report_requests_ack(payload, len)) {
         send_ack_report(report_id);
     }
 
    #if WIIMOTE_ENABLE_IO_DEBUG
    ESP_LOGI(TAG, "handled output report 0x%02x from %s (len=%u)",
             report_id, (source != NULL) ? source : "unknown", (unsigned int)len);
    ESP_LOGI(TAG, "state: mode=0x%02x continuous=%d rumble=%d flags=0x%02x",
             s_reporting_mode, s_reporting_continuous, s_rumble_enabled, s_status_flags);
    #endif
 }
 
 static void update_status_from_output_report(uint8_t report_id, const uint8_t *payload, uint16_t len)
 {
     if (payload == NULL || len == 0) {
         return;
     }
 
     const uint8_t control = payload[0];
     const bool feature_enabled = (control & 0x04) != 0;
 
     switch (report_id) {
     case 0x11: // Player LEDs are encoded in the high nibble.
         s_status_flags = (s_status_flags & 0x0F) | (control & 0xF0);
         break;
     case 0x13: // IR enable
     case 0x1A: // IR enable 2
         if (feature_enabled) {
             s_status_flags |= 0x08;
         } else {
             s_status_flags &= (uint8_t)~0x08;
         }
         break;
     case 0x14: // Speaker enable
         if (feature_enabled) {
             s_status_flags |= 0x04;
         } else {
             s_status_flags &= (uint8_t)~0x04;
         }
         break;
     default:
         break;
     }
 
     // Keep extension and low-battery bits clear in this emulator.
     s_status_flags &= (uint8_t)~0x03;
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
     if (s_data_report_task_hdl != NULL) {
         vTaskDelete(s_data_report_task_hdl);
         s_data_report_task_hdl = NULL;
     }
    if (s_uart_input_task_hdl != NULL) {
        vTaskDelete(s_uart_input_task_hdl);
        s_uart_input_task_hdl = NULL;
    }
 
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
                 s_reporting_mode = 0x30;
                 s_reporting_continuous = false;
                s_button_state = 0x0000;
                 ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0],
                          param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4],
                          param->open.bd_addr[5]);
                 bt_app_task_start_up();
                 send_default_input_report();
                 send_status_report();
                 if (s_data_report_task_hdl == NULL) {
                     BaseType_t ok = xTaskCreate(data_report_task, "data_report_task", 3 * 1024, NULL,
                                                 configMAX_PRIORITIES - 5, &s_data_report_task_hdl);
                     if (ok != pdPASS) {
                         ESP_LOGE(TAG, "failed to create data report task");
                         s_data_report_task_hdl = NULL;
                     }
                 }
                if (s_uart_input_task_hdl == NULL) {
                    BaseType_t ok = xTaskCreate(uart_input_task, "uart_input_task", 3 * 1024, NULL,
                                                configMAX_PRIORITIES - 6, &s_uart_input_task_hdl);
                    if (ok != pdPASS) {
                        ESP_LOGE(TAG, "failed to create uart input task");
                        s_uart_input_task_hdl = NULL;
                    }
                }
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
        #if WIIMOTE_ENABLE_IO_DEBUG
        ESP_LOGI(TAG, "ESP_HIDD_GET_REPORT_EVT id:0x%02x, type:%d, size:%d", param->get_report.report_id,
                 param->get_report.report_type, param->get_report.buffer_size);
        #endif
         log_hid_packet("RX", "get_report_request", param->get_report.report_type,
                        param->get_report.report_id, 0, NULL);
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
             if (report_id == 0x20) {
                 build_status_report_payload(s_local_param.buffer, report_len);
             } else {
                 memset(s_local_param.buffer, 0, report_len);
             }
             send_input_report_with_trace(param->get_report.report_type, report_id, report_len,
                                          s_local_param.buffer, "get_report_response");
             xSemaphoreGive(s_local_param.report_mutex);
         }
         break;
     case ESP_HIDD_SET_REPORT_EVT:
        #if WIIMOTE_ENABLE_IO_DEBUG
        ESP_LOGI(TAG, "ESP_HIDD_SET_REPORT_EVT id:0x%02x, type:%d, len:%d",
                 param->set_report.report_id, param->set_report.report_type, param->set_report.len);
        #endif
         log_hid_packet("RX", "set_report_request", param->set_report.report_type, param->set_report.report_id,
                        param->set_report.len, param->set_report.data);
         if (param->set_report.report_type == ESP_HIDD_REPORT_TYPE_OUTPUT) {
             handle_output_report_command(param->set_report.report_id, param->set_report.data,
                                          param->set_report.len, "set_report_evt");
         }
         break;
     case ESP_HIDD_SET_PROTOCOL_EVT:
         ESP_LOGI(TAG, "ESP_HIDD_SET_PROTOCOL_EVT mode:%d (ignored for Wii profile)",
                  param->set_protocol.protocol_mode);
         break;
     case ESP_HIDD_INTR_DATA_EVT:
        #if WIIMOTE_ENABLE_IO_DEBUG
        ESP_LOGI(TAG, "ESP_HIDD_INTR_DATA_EVT id:0x%02x, len:%d",
                 param->intr_data.report_id, param->intr_data.len);
        #endif
         log_hid_packet("RX", "intr_data", ESP_HIDD_REPORT_TYPE_INTRDATA, param->intr_data.report_id,
                        param->intr_data.len, param->intr_data.data);
         handle_output_report_command(param->intr_data.report_id, param->intr_data.data,
                                      param->intr_data.len, "intr_data_evt");
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
     init_emulated_memory();

    esp_err_t uart_ret = uart_driver_install(WIIMOTE_UART_PORT, 1024, 0, 0, NULL, 0);
    if (uart_ret != ESP_OK && uart_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(uart_ret));
    } else {
#if WIIMOTE_ENABLE_UART_BUTTON_DEBUG
        ESP_LOGI("wiimote_uart", "UART button input active on UART0 (arrows + h)");
#endif
    }
 
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
 