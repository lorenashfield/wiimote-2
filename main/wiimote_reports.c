#include "wiimote_reports.h"
#include "wiimote_state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <string.h>

#define WIIMOTE_MAX_REPORT_SIZE  (21)
#define WIIMOTE_MIN_REPORT_SIZE  (2)

static const char *TAG = "wiimote_reports";

/* --- HID report descriptor ------------------------------------------------ */

#define WIIMOTE_IN_REPORT(id, size) \
    0x85, (id), 0x75, 0x08, 0x95, (size), 0x81, 0x00
#define WIIMOTE_OUT_REPORT(id, size) \
    0x85, (id), 0x75, 0x08, 0x95, (size), 0x91, 0x00

/* Vendor-defined payloads with explicit report IDs/sizes, mirroring the
 * report lengths a real Nintendo RVL-CNT-01 advertises (see Wiibrew). */
static uint8_t s_descriptor[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xA1, 0x01,                    // COLLECTION (Application)

    WIIMOTE_OUT_REPORT(0x10, 1),   // Rumble
    WIIMOTE_OUT_REPORT(0x11, 1),   // Player LEDs
    WIIMOTE_OUT_REPORT(0x12, 2),   // Data reporting mode
    WIIMOTE_OUT_REPORT(0x13, 1),   // IR enable
    WIIMOTE_OUT_REPORT(0x14, 1),   // Speaker enable
    WIIMOTE_OUT_REPORT(0x15, 1),   // Status request
    WIIMOTE_OUT_REPORT(0x16, 21),  // Write memory/registers
    WIIMOTE_OUT_REPORT(0x17, 6),   // Read memory/registers
    WIIMOTE_OUT_REPORT(0x18, 21),  // Speaker data
    WIIMOTE_OUT_REPORT(0x19, 1),   // Speaker mute
    WIIMOTE_OUT_REPORT(0x1A, 1),   // IR enable 2

    WIIMOTE_IN_REPORT(0x20, 6),    // Status
    WIIMOTE_IN_REPORT(0x21, 21),   // Read memory/register data
    WIIMOTE_IN_REPORT(0x22, 4),    // Ack result
    WIIMOTE_IN_REPORT(0x30, 2),    // Buttons
    WIIMOTE_IN_REPORT(0x31, 5),    // Buttons + accel
    WIIMOTE_IN_REPORT(0x32, 10),   // Buttons + extension
    WIIMOTE_IN_REPORT(0x33, 17),   // Buttons + accel + IR
    WIIMOTE_IN_REPORT(0x34, 21),   // Buttons + extension
    WIIMOTE_IN_REPORT(0x35, 21),   // Buttons + accel + extension
    WIIMOTE_IN_REPORT(0x36, 21),   // Buttons + IR + extension
    WIIMOTE_IN_REPORT(0x37, 21),   // Buttons + accel + IR + extension
    WIIMOTE_IN_REPORT(0x3D, 21),   // Extension only
    WIIMOTE_IN_REPORT(0x3E, 21),   // Interleaved mode 1
    WIIMOTE_IN_REPORT(0x3F, 21),   // Interleaved mode 2

    0xC0                           // END_COLLECTION
};

/* --- data-report layout table -------------------------------------------- */

#define NA  0xFF  /* field not present in this mode */

/* Byte layout of each data report 0x30-0x3f (Wiibrew "Data Reporting"). */
typedef struct {
    uint8_t size;
    uint8_t buttons_off;
    uint8_t accel_off;
    uint8_t ir_off;
    uint8_t ir_len;
    uint8_t ext_off;
    uint8_t ext_len;
} report_layout_t;

static const report_layout_t k_layouts[16] = {
    /* 0x30 */ {  2,  0, NA, NA,  0, NA,  0 },
    /* 0x31 */ {  5,  0,  2, NA,  0, NA,  0 },
    /* 0x32 */ { 10,  0, NA, NA,  0,  2,  8 },
    /* 0x33 */ { 17,  0,  2,  5, 12, NA,  0 },
    /* 0x34 */ { 21,  0, NA, NA,  0,  2, 19 },
    /* 0x35 */ { 21,  0,  2, NA,  0,  5, 16 },
    /* 0x36 */ { 21,  0, NA,  2, 10, 12,  9 },
    /* 0x37 */ { 21,  0,  2,  5, 10, 15,  6 },
    /* 0x38 */ {  0, NA, NA, NA,  0, NA,  0 },  /* unused */
    /* 0x39 */ {  0, NA, NA, NA,  0, NA,  0 },  /* unused */
    /* 0x3a */ {  0, NA, NA, NA,  0, NA,  0 },  /* unused */
    /* 0x3b */ {  0, NA, NA, NA,  0, NA,  0 },  /* unused */
    /* 0x3c */ {  0, NA, NA, NA,  0, NA,  0 },  /* unused */
    /* 0x3d */ { 21, NA, NA, NA,  0,  0, 21 },  /* extension only */
    /* 0x3e */ { 21,  0, NA,  2, 19, NA,  0 },  /* interleaved (approx.) */
    /* 0x3f */ { 21,  0, NA,  2, 19, NA,  0 },  /* interleaved (approx.) */
};

/* --- report buffer / transport ------------------------------------------- */

static SemaphoreHandle_t s_mutex;
static uint8_t           s_buffer[WIIMOTE_MAX_REPORT_SIZE];

void wiimote_reports_init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
}

/* Send s_buffer. Caller holds s_mutex. */
static void transmit(uint8_t report_id, uint8_t len)
{
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, report_id, len, s_buffer);
}

static uint8_t input_report_size(uint8_t report_id)
{
    switch (report_id) {
    case 0x20: return 6;
    case 0x21: return 21;
    case 0x22: return 4;
    default:
        if (report_id >= 0x30 && report_id <= 0x3f) {
            return k_layouts[report_id - 0x30].size;
        }
        return 0;
    }
}

uint8_t *wiimote_reports_descriptor(uint16_t *len_out)
{
    if (len_out != NULL) {
        *len_out = (uint16_t)sizeof(s_descriptor);
    }
    return s_descriptor;
}

/* --- assemblers ----------------------------------------------------------- */

static uint8_t compute_status_flags(const wiimote_state_t *st)
{
    uint8_t flags = (uint8_t)(st->led_mask & 0xF0);
    if (st->ir_enabled)      flags |= 0x08;
    if (st->speaker_enabled) flags |= 0x04;
    if (st->battery_low)     flags |= 0x01;
    /* bit 0x02 (extension attached) stays clear: no extension emulated. */
    return flags;
}

/* Pack the current reporting mode's data report into buf; returns its size. */
static uint8_t assemble_data_report(const wiimote_state_t *st, uint8_t *buf)
{
    const uint8_t mode = st->reporting_mode;
    if (mode < 0x30 || mode > 0x3f) {
        return 0;
    }
    const report_layout_t *layout = &k_layouts[mode - 0x30];
    if (layout->size == 0) {
        return 0;
    }

    memset(buf, 0, layout->size);

    if (layout->buttons_off != NA) {
        buf[layout->buttons_off]     = (uint8_t)(st->buttons >> 8);
        buf[layout->buttons_off + 1] = (uint8_t)(st->buttons & 0xFF);
    }
    if (layout->accel_off != NA) {
        buf[layout->accel_off]     = st->accel[0];
        buf[layout->accel_off + 1] = st->accel[1];
        buf[layout->accel_off + 2] = st->accel[2];
        /* A 10-bit accelerometer driver would pack the 2 extra LSBs into the
         * spare button-byte bits here; 8-bit-precision data needs nothing. */
    }
    if (layout->ir_off != NA && layout->ir_len > 0) {
        /* No camera yet: 0xFF marks every IR object absent. A future IR
         * driver packs st->ir[] per the mode's IR format at this offset. */
        memset(&buf[layout->ir_off], 0xFF, layout->ir_len);
    }
    if (layout->ext_off != NA && layout->ext_len > 0) {
        uint8_t n = layout->ext_len;
        if (n > WIIMOTE_EXTENSION_BYTES) {
            n = WIIMOTE_EXTENSION_BYTES;
        }
        memcpy(&buf[layout->ext_off], st->extension, n);
    }
    return layout->size;
}

/* --- public senders ------------------------------------------------------- */

void wiimote_reports_send_data(void)
{
    wiimote_state_t st;
    wiimote_state_snapshot(&st);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint8_t size = assemble_data_report(&st, s_buffer);
    if (size >= WIIMOTE_MIN_REPORT_SIZE) {
        transmit(st.reporting_mode, size);
    }
    xSemaphoreGive(s_mutex);
}

void wiimote_reports_send_status(void)
{
    wiimote_state_t st;
    wiimote_state_snapshot(&st);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(s_buffer, 0, 6);
    s_buffer[0] = (uint8_t)(st.buttons >> 8);
    s_buffer[1] = (uint8_t)(st.buttons & 0xFF);
    s_buffer[2] = compute_status_flags(&st);
    s_buffer[5] = st.battery_level;
    transmit(0x20, 6);
    xSemaphoreGive(s_mutex);
}

void wiimote_reports_send_ack(uint8_t output_report_id, uint8_t error)
{
    const uint16_t buttons = wiimote_state_get_buttons();

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(s_buffer, 0, 4);
    s_buffer[0] = (uint8_t)(buttons >> 8);
    s_buffer[1] = (uint8_t)(buttons & 0xFF);
    s_buffer[2] = output_report_id;
    s_buffer[3] = error;
    transmit(0x22, 4);
    xSemaphoreGive(s_mutex);
}

void wiimote_reports_send_read_data(uint16_t offset16, uint8_t chunk_len,
                                    uint8_t error, const uint8_t *data)
{
    if (chunk_len == 0 || chunk_len > 16 || data == NULL) {
        return;
    }
    const uint16_t buttons = wiimote_state_get_buttons();

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(s_buffer, 0, 21);
    s_buffer[0] = (uint8_t)(buttons >> 8);
    s_buffer[1] = (uint8_t)(buttons & 0xFF);
    /* High nibble = (size - 1), low nibble = error code. */
    s_buffer[2] = (uint8_t)(((chunk_len - 1U) << 4) | (error & 0x0F));
    s_buffer[3] = (uint8_t)((offset16 >> 8) & 0xFF);
    s_buffer[4] = (uint8_t)(offset16 & 0xFF);
    memcpy(&s_buffer[5], data, chunk_len);
    transmit(0x21, 21);
    xSemaphoreGive(s_mutex);
}

void wiimote_reports_handle_get_report(esp_hidd_report_type_t report_type,
                                       uint8_t report_id)
{
    if (report_type != ESP_HIDD_REPORT_TYPE_INPUT) {
        esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM);
        return;
    }

    const uint8_t size = input_report_size(report_id);
    if (size < WIIMOTE_MIN_REPORT_SIZE || size > WIIMOTE_MAX_REPORT_SIZE) {
        ESP_LOGW(TAG, "unsupported GET_REPORT id 0x%02x", report_id);
        esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID);
        return;
    }

    wiimote_state_t st;
    wiimote_state_snapshot(&st);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(s_buffer, 0, size);
    if (report_id == 0x20) {
        s_buffer[0] = (uint8_t)(st.buttons >> 8);
        s_buffer[1] = (uint8_t)(st.buttons & 0xFF);
        s_buffer[2] = compute_status_flags(&st);
        s_buffer[5] = st.battery_level;
    } else if (report_id >= 0x30 && report_id <= 0x3f) {
        st.reporting_mode = report_id;
        assemble_data_report(&st, s_buffer);
    }
    esp_bt_hid_device_send_report(report_type, report_id, size, s_buffer);
    xSemaphoreGive(s_mutex);
}
