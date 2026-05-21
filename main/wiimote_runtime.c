#include "wiimote_runtime.h"
#include "wiimote_state.h"
#include "wiimote_reports.h"
#include "wiimote_bt.h"
#include "wiimote_uart_input.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <string.h>

/* Report cadence. A real Wii Remote polls at ~200 Hz (5 ms); 10 ms (100 Hz)
 * is plenty for gameplay and lighter on the radio. */
#define WIIMOTE_REPORT_INTERVAL_MS  (10)
#define WIIMOTE_IDLE_INTERVAL_MS    (100)

static const char *TAG = "wiimote_runtime";
static TaskHandle_t s_task;

/* True when an input the host can see in a data report has changed. */
static bool inputs_changed(const wiimote_state_t *now, const wiimote_state_t *prev)
{
    return now->buttons        != prev->buttons
        || now->reporting_mode != prev->reporting_mode
        || memcmp(now->accel, prev->accel, sizeof(now->accel)) != 0
        || memcmp(now->ir, prev->ir, sizeof(now->ir)) != 0
        || memcmp(now->extension, prev->extension, sizeof(now->extension)) != 0;
}

static void runtime_task(void *arg)
{
    (void)arg;
    wiimote_state_t now;
    wiimote_state_t prev;
    bool have_prev = false;

    for (;;) {
        if (!wiimote_bt_is_connected()) {
            have_prev = false;
            vTaskDelay(pdMS_TO_TICKS(WIIMOTE_IDLE_INTERVAL_MS));
            continue;
        }

        /* --- sample input sources -------------------------------------- *
         * Detach point: remove the UART poll when GPIO buttons replace it;
         * future accel/IR sampling is added here as additional calls.      */
        wiimote_uart_input_poll();

        /* --- emit the data report -------------------------------------- */
        wiimote_state_snapshot(&now);
        const bool pending = wiimote_state_take_report_pending();
        const bool changed = !have_prev || inputs_changed(&now, &prev);

        if (now.reporting_continuous || changed || pending) {
            wiimote_reports_send_data();
        }
        prev = now;
        have_prev = true;

        /* --- apply output devices -------------------------------------- *
         * Attach point: future rumble / LED drivers act on now.rumble and
         * now.led_mask here.                                               */

        vTaskDelay(pdMS_TO_TICKS(WIIMOTE_REPORT_INTERVAL_MS));
    }
}

void wiimote_runtime_start(void)
{
    if (s_task != NULL) {
        return;
    }
    BaseType_t ok = xTaskCreate(runtime_task, "wiimote_runtime", 3 * 1024, NULL,
                                configMAX_PRIORITIES - 5, &s_task);
    if (ok != pdPASS) {
        s_task = NULL;
        ESP_LOGE(TAG, "failed to create runtime task");
    }
}
