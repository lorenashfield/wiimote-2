/*
 * Device-to-host HID reports: the report descriptor and every input report
 * the Wii Remote sends.
 *
 * Data reports 0x30-0x3f are produced by a table-driven assembler: each mode
 * has a fixed byte layout (buttons / accelerometer / IR / extension), and the
 * assembler packs a wiimote_state snapshot into it. Accelerometer and IR
 * regions read neutral until their hardware drivers fill wiimote_state.
 */
#pragma once

#include <stdint.h>

#include "esp_hidd_api.h"

void wiimote_reports_init(void);

/* HID report descriptor for esp_bt_hid_device_register_app(). */
uint8_t *wiimote_reports_descriptor(uint16_t *len_out);

/* Assemble + send the data report for the current reporting mode. */
void wiimote_reports_send_data(void);

/* Status report 0x20 (buttons, flags, battery). */
void wiimote_reports_send_status(void);

/* Acknowledge an output report: report 0x22. */
void wiimote_reports_send_ack(uint8_t output_report_id, uint8_t error);

/* Read-memory response: report 0x21 (chunk_len is 1..16). */
void wiimote_reports_send_read_data(uint16_t offset16, uint8_t chunk_len,
                                    uint8_t error, const uint8_t *data);

/* Respond to a host GET_REPORT request. */
void wiimote_reports_handle_get_report(esp_hidd_report_type_t report_type,
                                       uint8_t report_id);
