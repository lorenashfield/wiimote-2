#include "wiimote_uart_input.h"
#include "wiimote_buttons.h"
#include "wiimote_state.h"

#include "driver/uart.h"
#include "esp_log.h"

#define WIIMOTE_UART_PORT             (UART_NUM_0)
#define WIIMOTE_UART_RX_BUFFER        (1024)
#define WIIMOTE_ENABLE_BUTTON_DEBUG   (1)

static const char *TAG = "wiimote_uart";

/* Persisted across poll() calls: the two-byte frame state machine. */
static enum {
    FRAME_IDLE = 0,   /* waiting for a '+' or '-' prefix */
    FRAME_PRESS,
    FRAME_RELEASE
} s_frame_state = FRAME_IDLE;

/* Map a UART key byte to a Wii button mask. */
static bool decode_key(uint8_t key, uint16_t *mask, const char **name)
{
    switch (key) {
    case 'w': case 'W': *mask = WIIMOTE_BTN_RIGHT; *name = "DPAD_RIGHT"; return true;
    case 'd': case 'D': *mask = WIIMOTE_BTN_DOWN;  *name = "DPAD_DOWN";  return true;
    case 's': case 'S': *mask = WIIMOTE_BTN_LEFT;  *name = "DPAD_LEFT";  return true;
    case 'a': case 'A': *mask = WIIMOTE_BTN_UP;    *name = "DPAD_UP";    return true;
    case ' ':           *mask = WIIMOTE_BTN_TWO;   *name = "BUTTON_2";   return true;
    case '/':           *mask = WIIMOTE_BTN_ONE;   *name = "BUTTON_1";   return true;
    case 'h': case 'H': *mask = WIIMOTE_BTN_HOME;  *name = "HOME";       return true;
    default:            return false;
    }
}

/*
 * UART wire protocol: every button event is a two-byte frame -- a '+' (press)
 * or '-' (release) prefix followed by a button key. Button state is sticky,
 * so a pressed button stays held until its matching release arrives and any
 * number of buttons can be held at once. A key byte received without a prefix
 * is dropped, which resyncs the framing after a lost/corrupted byte.
 */
static void process_byte(uint8_t byte)
{
    if (byte == '+') {
        s_frame_state = FRAME_PRESS;
        return;
    }
    if (byte == '-') {
        s_frame_state = FRAME_RELEASE;
        return;
    }
    if (s_frame_state == FRAME_IDLE) {
        return;
    }

    const bool press = (s_frame_state == FRAME_PRESS);
    s_frame_state = FRAME_IDLE;

    uint16_t mask = 0;
    const char *name = NULL;
    if (!decode_key(byte, &mask, &name)) {
        return;
    }

    wiimote_state_set_buttons(mask, press);
#if WIIMOTE_ENABLE_BUTTON_DEBUG
    ESP_LOGI(TAG, "button %s: %s", press ? "press" : "release", name);
#endif
}

void wiimote_uart_input_init(void)
{
    esp_err_t err = uart_driver_install(WIIMOTE_UART_PORT, WIIMOTE_UART_RX_BUFFER,
                                        0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return;
    }
#if WIIMOTE_ENABLE_BUTTON_DEBUG
    ESP_LOGI(TAG, "UART button input on UART0: press '+<key>', release '-<key>'; "
                  "keys [W/A/S/D=DPAD, space=2, /=1, h=HOME]");
#endif
}

void wiimote_uart_input_poll(void)
{
    uint8_t buf[32];
    int n;
    while ((n = uart_read_bytes(WIIMOTE_UART_PORT, buf, sizeof(buf), 0)) > 0) {
        for (int i = 0; i < n; i++) {
            process_byte(buf[i]);
        }
        if (n < (int)sizeof(buf)) {
            break;   /* RX buffer drained */
        }
    }
}
