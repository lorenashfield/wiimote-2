/*
 * Wii Remote Bluetooth emulator for the ESP32.
 *
 * app_main only wires the modules together; each subsystem owns its own file:
 *   wiimote_memory  - emulated EEPROM + control registers
 *   wiimote_state   - central whole-device state model
 *   wiimote_reports - HID descriptor + device-to-host report assembler
 *   wiimote_protocol- host-to-host output-report command handling
 *   wiimote_uart_input - detachable UART button input source
 *   wiimote_runtime - the polling loop that samples inputs and emits reports
 *   wiimote_bt      - Bluetooth bring-up, pairing, and the HID profile
 */

#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "wiimote_memory.h"
#include "wiimote_state.h"
#include "wiimote_reports.h"
#include "wiimote_uart_input.h"
#include "wiimote_runtime.h"
#include "wiimote_bt.h"

void app_main(void)
{
    static const char *TAG = "app_main";
    ESP_LOGI(TAG, "=== Wii Remote emulation boot ===");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wiimote_memory_init();      /* emulated EEPROM + control registers   */
    wiimote_state_init();       /* central device state                  */
    wiimote_reports_init();     /* HID report transport                  */
    wiimote_uart_input_init();  /* detachable: UART button input source  */
    wiimote_runtime_start();    /* polling loop (idles until connected)   */
    wiimote_bt_init();          /* Bluetooth + pairing + HID profile     */
}
