/*
 * Bluetooth layer: classic-BT + Bluedroid bring-up, the Wii sync-button
 * pairing model (legacy binary PIN), the Nintendo SDP device-ID record, the
 * HID-device profile callback, and the HID connection state.
 */
#pragma once

#include <stdbool.h>

/* Bring up the controller, Bluedroid, SDP, and the HID device profile, then
 * open the sync-style pairing window. Call once after NVS is ready. */
void wiimote_bt_init(void);

/* True while an HID connection to the Wii is open. */
bool wiimote_bt_is_connected(void);
