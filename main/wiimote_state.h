/*
 * Central whole-device state for the emulated Wii Remote.
 *
 * One thread-safe model holds every input (buttons, accelerometer, IR, the
 * extension port) and every host-controlled device setting (reporting mode,
 * player LEDs, rumble, speaker, battery). Input drivers write their slice,
 * the protocol layer writes device settings from host output reports, and the
 * report assembler reads a consistent snapshot. Adding a hardware driver later
 * means filling an existing field -- the struct does not change.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define WIIMOTE_IR_OBJECT_COUNT  (4)
#define WIIMOTE_EXTENSION_BYTES  (21)
#define WIIMOTE_DEFAULT_MODE     (0x30)
#define WIIMOTE_ACCEL_NEUTRAL    (0x80)

/* One IR camera object. Driven by the future IR module; visible == false
 * until then, which the assembler emits as "no object". */
typedef struct {
    uint16_t x;        /* 0..1023 */
    uint16_t y;        /* 0..767  */
    uint8_t  size;     /* 0..15   */
    bool     visible;
} wiimote_ir_object_t;

/* Full device state. Copied out as a unit by wiimote_state_snapshot(). */
typedef struct {
    /* --- inputs --- */
    uint16_t buttons;                                /* core button bitmask */
    uint8_t  accel[3];                               /* X,Y,Z; 0x80 at rest */
    wiimote_ir_object_t ir[WIIMOTE_IR_OBJECT_COUNT];
    uint8_t  extension[WIIMOTE_EXTENSION_BYTES];
    /* --- host-controlled device state --- */
    uint8_t  reporting_mode;                         /* 0x30..0x3f */
    bool     reporting_continuous;
    bool     rumble;
    uint8_t  led_mask;                               /* player LEDs, high nibble */
    bool     ir_enabled;
    bool     speaker_enabled;
    bool     speaker_muted;
    uint8_t  battery_level;                          /* 0x00..0xff */
    bool     battery_low;
} wiimote_state_t;

void wiimote_state_init(void);
void wiimote_state_reset(void);                      /* restore power-on defaults */

void wiimote_state_snapshot(wiimote_state_t *out);

void     wiimote_state_set_buttons(uint16_t mask, bool pressed);
uint16_t wiimote_state_get_buttons(void);

void    wiimote_state_set_reporting(uint8_t mode, bool continuous);
uint8_t wiimote_state_get_reporting_mode(void);

void wiimote_state_set_rumble(bool on);
void wiimote_state_set_led_mask(uint8_t led_mask);
void wiimote_state_set_ir_enabled(bool on);
void wiimote_state_set_speaker_enabled(bool on);
void wiimote_state_set_speaker_muted(bool muted);
void wiimote_state_set_battery(uint8_t level, bool low);

/* One-shot "send a report next tick" request (used on connect / mode change). */
void wiimote_state_request_report(void);
bool wiimote_state_take_report_pending(void);
