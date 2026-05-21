#include "wiimote_protocol.h"
#include "wiimote_state.h"
#include "wiimote_reports.h"
#include "wiimote_memory.h"

#include <string.h>

/* Decode the address-space selector shared by read/write-memory requests.
 * Returns false (and sets *invalid) when both selector bits are set. */
static bool decode_address_space(uint8_t control, bool *invalid)
{
    const bool bit2 = (control & 0x04) != 0;
    const bool bit3 = (control & 0x08) != 0;
    *invalid = bit2 && bit3;
    return bit2 || bit3;   /* true => control-register space, false => EEPROM */
}

/* Output report 0x17: stream the requested memory back in 0x21 chunks. */
static void handle_read_memory(const uint8_t *payload, uint16_t len)
{
    if (payload == NULL || len < 6) {
        return;
    }

    bool invalid = false;
    const bool register_space = decode_address_space(payload[0], &invalid);
    const uint32_t offset = ((uint32_t)payload[1] << 16) |
                            ((uint32_t)payload[2] << 8) |
                            (uint32_t)payload[3];
    uint16_t remaining = ((uint16_t)payload[4] << 8) | (uint16_t)payload[5];

    uint8_t data[16] = {0};
    if (remaining == 0) {
        return;
    }
    if (invalid) {
        wiimote_reports_send_read_data((uint16_t)(offset & 0xFFFF), 16,
                                       WIIMOTE_READ_ERROR_INVALID_ADDRESS, data);
        return;
    }

    uint32_t current = offset;
    while (remaining > 0) {
        const uint8_t chunk = (remaining > 16U) ? 16U : (uint8_t)remaining;
        uint8_t error = WIIMOTE_READ_ERROR_SUCCESS;
        memset(data, 0, sizeof(data));

        if (!wiimote_memory_read(current, register_space, data, chunk, &error)) {
            wiimote_reports_send_read_data((uint16_t)(current & 0xFFFF), 16, error, data);
            return;
        }
        wiimote_reports_send_read_data((uint16_t)(current & 0xFFFF), chunk,
                                       WIIMOTE_READ_ERROR_SUCCESS, data);
        current += chunk;
        remaining = (uint16_t)(remaining - chunk);
    }
}

/* Output report 0x16: apply a memory write and return the result code. */
static uint8_t handle_write_memory(const uint8_t *payload, uint16_t len)
{
    if (payload == NULL || len < 5) {
        return WIIMOTE_READ_ERROR_NACK;
    }

    bool invalid = false;
    const bool register_space = decode_address_space(payload[0], &invalid);
    if (invalid) {
        return WIIMOTE_READ_ERROR_INVALID_ADDRESS;
    }

    const uint32_t offset = ((uint32_t)payload[1] << 16) |
                            ((uint32_t)payload[2] << 8) |
                            (uint32_t)payload[3];
    const uint8_t write_size = payload[4];
    if (write_size == 0 || write_size > 16) {
        return WIIMOTE_READ_ERROR_NACK;
    }
    if ((uint32_t)len < (uint32_t)5 + write_size) {
        return WIIMOTE_READ_ERROR_NACK;
    }

    uint8_t error = WIIMOTE_READ_ERROR_SUCCESS;
    if (!wiimote_memory_write(offset, register_space, &payload[5], write_size, &error)) {
        return error;
    }
    return WIIMOTE_READ_ERROR_SUCCESS;
}

void wiimote_protocol_handle_output_report(uint8_t report_id,
                                           const uint8_t *payload, uint16_t len)
{
    if (payload == NULL || len == 0) {
        return;
    }

    const uint8_t control = payload[0];

    /* Bit 0 of every output report carries the live rumble state. */
    wiimote_state_set_rumble((control & 0x01) != 0);

    switch (report_id) {
    case 0x10:  /* rumble only -- applied above */
        break;
    case 0x11:  /* player LEDs (high nibble) */
        wiimote_state_set_led_mask((uint8_t)(control & 0xF0));
        break;
    case 0x12:  /* data reporting mode */
        if (len >= 2) {
            wiimote_state_set_reporting(payload[1], (control & 0x04) != 0);
            wiimote_reports_send_data();
        }
        break;
    case 0x13:  /* IR pixel-clock enable */
    case 0x1A:  /* IR logic enable */
        wiimote_state_set_ir_enabled((control & 0x04) != 0);
        break;
    case 0x14:  /* speaker enable */
        wiimote_state_set_speaker_enabled((control & 0x04) != 0);
        break;
    case 0x15:  /* host status request */
        wiimote_reports_send_status();
        break;
    case 0x16:  /* write memory -- sends its own ack with the result */
        wiimote_reports_send_ack(report_id, handle_write_memory(payload, len));
        break;
    case 0x17:  /* read memory */
        handle_read_memory(payload, len);
        break;
    case 0x18:  /* speaker data -- a future speaker driver consumes the payload */
        break;
    case 0x19:  /* speaker mute */
        wiimote_state_set_speaker_muted((control & 0x04) != 0);
        break;
    default:
        break;
    }

    /* Bit 1 requests an explicit ack; 0x16 already acked with its result. */
    if (report_id != 0x16 && (control & 0x02) != 0) {
        wiimote_reports_send_ack(report_id, 0x00);
    }
}
