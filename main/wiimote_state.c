#include "wiimote_state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static wiimote_state_t s_state;
static bool            s_report_pending;
static portMUX_TYPE    s_lock = portMUX_INITIALIZER_UNLOCKED;

/* Caller must hold s_lock. */
static void apply_defaults(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_state.accel[0] = WIIMOTE_ACCEL_NEUTRAL;
    s_state.accel[1] = WIIMOTE_ACCEL_NEUTRAL;
    s_state.accel[2] = WIIMOTE_ACCEL_NEUTRAL;
    s_state.reporting_mode = WIIMOTE_DEFAULT_MODE;
    s_state.battery_level  = 0xFF;
    s_report_pending = true;
}

void wiimote_state_init(void)
{
    taskENTER_CRITICAL(&s_lock);
    apply_defaults();
    taskEXIT_CRITICAL(&s_lock);
}

void wiimote_state_reset(void)
{
    taskENTER_CRITICAL(&s_lock);
    apply_defaults();
    taskEXIT_CRITICAL(&s_lock);
}

void wiimote_state_snapshot(wiimote_state_t *out)
{
    if (out == NULL) {
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    *out = s_state;
    taskEXIT_CRITICAL(&s_lock);
}

void wiimote_state_set_buttons(uint16_t mask, bool pressed)
{
    taskENTER_CRITICAL(&s_lock);
    if (pressed) {
        s_state.buttons |= mask;
    } else {
        s_state.buttons &= (uint16_t)~mask;
    }
    taskEXIT_CRITICAL(&s_lock);
}

uint16_t wiimote_state_get_buttons(void)
{
    taskENTER_CRITICAL(&s_lock);
    uint16_t value = s_state.buttons;
    taskEXIT_CRITICAL(&s_lock);
    return value;
}

void wiimote_state_set_reporting(uint8_t mode, bool continuous)
{
    taskENTER_CRITICAL(&s_lock);
    s_state.reporting_mode = mode;
    s_state.reporting_continuous = continuous;
    s_report_pending = true;
    taskEXIT_CRITICAL(&s_lock);
}

uint8_t wiimote_state_get_reporting_mode(void)
{
    taskENTER_CRITICAL(&s_lock);
    uint8_t mode = s_state.reporting_mode;
    taskEXIT_CRITICAL(&s_lock);
    return mode;
}

void wiimote_state_set_rumble(bool on)
{
    taskENTER_CRITICAL(&s_lock);
    s_state.rumble = on;
    taskEXIT_CRITICAL(&s_lock);
}

void wiimote_state_set_led_mask(uint8_t led_mask)
{
    taskENTER_CRITICAL(&s_lock);
    s_state.led_mask = led_mask;
    taskEXIT_CRITICAL(&s_lock);
}

void wiimote_state_set_ir_enabled(bool on)
{
    taskENTER_CRITICAL(&s_lock);
    s_state.ir_enabled = on;
    taskEXIT_CRITICAL(&s_lock);
}

void wiimote_state_set_speaker_enabled(bool on)
{
    taskENTER_CRITICAL(&s_lock);
    s_state.speaker_enabled = on;
    taskEXIT_CRITICAL(&s_lock);
}

void wiimote_state_set_speaker_muted(bool muted)
{
    taskENTER_CRITICAL(&s_lock);
    s_state.speaker_muted = muted;
    taskEXIT_CRITICAL(&s_lock);
}

void wiimote_state_set_battery(uint8_t level, bool low)
{
    taskENTER_CRITICAL(&s_lock);
    s_state.battery_level = level;
    s_state.battery_low = low;
    taskEXIT_CRITICAL(&s_lock);
}

void wiimote_state_request_report(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_report_pending = true;
    taskEXIT_CRITICAL(&s_lock);
}

bool wiimote_state_take_report_pending(void)
{
    taskENTER_CRITICAL(&s_lock);
    bool pending = s_report_pending;
    s_report_pending = false;
    taskEXIT_CRITICAL(&s_lock);
    return pending;
}
