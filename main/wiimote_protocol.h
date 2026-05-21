/*
 * Host-to-device output reports (0x10-0x1a): rumble, player LEDs, data
 * reporting mode, IR/speaker enable, status requests, and memory access.
 *
 * Each report is parsed into wiimote_state; the report layer reflects that
 * state back to the Wii. This is the single place host commands are decoded.
 */
#pragma once

#include <stdint.h>

void wiimote_protocol_handle_output_report(uint8_t report_id,
                                           const uint8_t *payload, uint16_t len);
