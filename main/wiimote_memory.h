/*
 * Emulated Wii Remote memory: the EEPROM (calibration / user data) and the
 * peripheral control-register blocks (speaker, extension, MotionPlus, IR).
 *
 * The Wii probes this memory during connection setup via read/write-memory
 * output reports. The register blocks are also the seam for future hardware:
 * the IR camera is configured through 0xB00000 and the MotionPlus/gyro lives
 * at 0xA60000.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define WIIMOTE_READ_ERROR_SUCCESS          (0x00)
#define WIIMOTE_READ_ERROR_NACK             (0x07)
#define WIIMOTE_READ_ERROR_INVALID_ADDRESS  (0x08)

void wiimote_memory_init(void);

/* Read/write EEPROM (register_space == false) or a control-register block
 * (register_space == true). Returns false and sets *error on failure. */
bool wiimote_memory_read(uint32_t offset, bool register_space,
                         uint8_t *out, uint16_t len, uint8_t *error);
bool wiimote_memory_write(uint32_t offset, bool register_space,
                          const uint8_t *in, uint16_t len, uint8_t *error);
