#include "wiimote_memory.h"

#include <string.h>

#define WIIMOTE_EEPROM_SIZE          (0x1700)
#define WIIMOTE_REGISTER_BLOCK_SIZE  (0x100)

#define WIIMOTE_REG_BASE_SPEAKER     (0xA20000)
#define WIIMOTE_REG_BASE_EXTENSION   (0xA40000)
#define WIIMOTE_REG_BASE_MOTION_PLUS (0xA60000)
#define WIIMOTE_REG_BASE_IR          (0xB00000)

static uint8_t s_eeprom[WIIMOTE_EEPROM_SIZE];
static uint8_t s_reg_speaker[WIIMOTE_REGISTER_BLOCK_SIZE];
static uint8_t s_reg_extension[WIIMOTE_REGISTER_BLOCK_SIZE];
static uint8_t s_reg_motion_plus[WIIMOTE_REGISTER_BLOCK_SIZE];
static uint8_t s_reg_ir[WIIMOTE_REGISTER_BLOCK_SIZE];

static uint8_t *register_block_for(uint32_t offset)
{
    switch (offset & 0xFF0000) {
    case WIIMOTE_REG_BASE_SPEAKER:     return s_reg_speaker;
    case WIIMOTE_REG_BASE_EXTENSION:   return s_reg_extension;
    case WIIMOTE_REG_BASE_MOTION_PLUS: return s_reg_motion_plus;
    case WIIMOTE_REG_BASE_IR:          return s_reg_ir;
    default:                           return NULL;
    }
}

void wiimote_memory_init(void)
{
    /* Accelerometer/IR calibration block the Wii reads back during init. */
    static const uint8_t calibration_block[] = {
        0xA1, 0xAA, 0x8B, 0x99, 0xAE, 0x9E, 0x78, 0x30, 0xA7, 0x74, 0xD3, 0xA1, 0xAA, 0x8B, 0x99, 0xAE,
        0x9E, 0x78, 0x30, 0xA7, 0x74, 0xD3, 0x82, 0x82, 0x82, 0x15, 0x9C, 0x9C, 0x9E, 0x38, 0x40, 0x3E,
        0x82, 0x82, 0x82, 0x15, 0x9C, 0x9C, 0x9E, 0x38, 0x40, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    static const uint8_t unknown_tail[] = {
        0x00, 0x00, 0x00, 0xFF, 0x11, 0xEE, 0x00, 0x00, 0x33, 0xCC, 0x44, 0xBB, 0x00, 0x00, 0x66, 0x99,
        0x77, 0x88, 0x00, 0x00, 0x2B, 0x01, 0xE8, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    memset(s_eeprom, 0, sizeof(s_eeprom));
    memset(s_reg_speaker, 0, sizeof(s_reg_speaker));
    memset(s_reg_extension, 0, sizeof(s_reg_extension));
    memset(s_reg_motion_plus, 0, sizeof(s_reg_motion_plus));
    memset(s_reg_ir, 0, sizeof(s_reg_ir));

    memcpy(&s_eeprom[0x0000], calibration_block, sizeof(calibration_block));
    memcpy(&s_eeprom[0x16D0], unknown_tail, sizeof(unknown_tail));
}

bool wiimote_memory_read(uint32_t offset, bool register_space,
                         uint8_t *out, uint16_t len, uint8_t *error)
{
    if (out == NULL || len == 0 || error == NULL) {
        return false;
    }
    *error = WIIMOTE_READ_ERROR_SUCCESS;

    if (!register_space) {
        const uint16_t low16 = (uint16_t)(offset & 0xFFFF);
        if ((uint32_t)low16 + len > WIIMOTE_EEPROM_SIZE) {
            *error = WIIMOTE_READ_ERROR_INVALID_ADDRESS;
            memset(out, 0, len);
            return false;
        }
        memcpy(out, &s_eeprom[low16], len);
        return true;
    }

    uint8_t *reg_block = register_block_for(offset);
    if (reg_block == NULL) {
        *error = WIIMOTE_READ_ERROR_INVALID_ADDRESS;
        memset(out, 0, len);
        return false;
    }

    const uint16_t reg_offset = (uint16_t)(offset & 0x00FF);
    if ((uint32_t)reg_offset + len > WIIMOTE_REGISTER_BLOCK_SIZE) {
        *error = WIIMOTE_READ_ERROR_NACK;
        memset(out, 0, len);
        return false;
    }
    memcpy(out, &reg_block[reg_offset], len);
    return true;
}

bool wiimote_memory_write(uint32_t offset, bool register_space,
                          const uint8_t *in, uint16_t len, uint8_t *error)
{
    if (in == NULL || len == 0 || error == NULL) {
        return false;
    }
    *error = WIIMOTE_READ_ERROR_SUCCESS;

    if (!register_space) {
        const uint16_t low16 = (uint16_t)(offset & 0xFFFF);
        if ((uint32_t)low16 + len > WIIMOTE_EEPROM_SIZE) {
            *error = WIIMOTE_READ_ERROR_INVALID_ADDRESS;
            return false;
        }
        memcpy(&s_eeprom[low16], in, len);
        return true;
    }

    uint8_t *reg_block = register_block_for(offset);
    if (reg_block == NULL) {
        *error = WIIMOTE_READ_ERROR_INVALID_ADDRESS;
        return false;
    }

    const uint16_t reg_offset = (uint16_t)(offset & 0x00FF);
    if ((uint32_t)reg_offset + len > WIIMOTE_REGISTER_BLOCK_SIZE) {
        *error = WIIMOTE_READ_ERROR_NACK;
        return false;
    }
    memcpy(&reg_block[reg_offset], in, len);
    return true;
}
