/*
 * DETACHABLE button input source: UART.
 *
 * Reads the two-byte "+key" / "-key" frame protocol from UART0 and feeds
 * presses/releases into wiimote_state. This module is temporary scaffolding
 * for bring-up; when real GPIO buttons are wired it is removed wholesale --
 * delete this file pair, drop it from CMakeLists.txt, and remove the two
 * call sites (init in main.c, poll in wiimote_runtime). Nothing else depends
 * on it: the button bitmask lives in wiimote_buttons.h / wiimote_state.
 */
#pragma once

/* Install the UART driver. Call once at startup. */
void wiimote_uart_input_init(void);

/* Drain any buffered UART bytes and apply button presses/releases.
 * Non-blocking; called once per runtime tick. */
void wiimote_uart_input_poll(void);
