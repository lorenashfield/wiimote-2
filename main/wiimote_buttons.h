/*
 * Core Wii Remote button bit-mask definitions.
 *
 * This is the permanent button layer: the 16-bit core button bitmask is sent
 * verbatim to the Wii in every data report (byte0 << 8 | byte1, per Wiibrew).
 * Input sources (the UART module today, GPIO buttons later) map their own
 * events onto these masks and feed them into wiimote_state. The masks never
 * change when the input source is swapped.
 */
#pragma once

/* High byte (report byte 0) */
#define WIIMOTE_BTN_LEFT   (0x0100)
#define WIIMOTE_BTN_RIGHT  (0x0200)
#define WIIMOTE_BTN_DOWN   (0x0400)
#define WIIMOTE_BTN_UP     (0x0800)
#define WIIMOTE_BTN_PLUS   (0x1000)

/* Low byte (report byte 1) */
#define WIIMOTE_BTN_TWO    (0x0001)
#define WIIMOTE_BTN_ONE    (0x0002)
#define WIIMOTE_BTN_B      (0x0004)
#define WIIMOTE_BTN_A      (0x0008)
#define WIIMOTE_BTN_MINUS  (0x0010)
#define WIIMOTE_BTN_HOME   (0x0080)
