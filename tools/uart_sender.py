#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = [
#     "pyserial>=3.5",
#     "pynput>=1.7.6",
# ]
# ///
"""
UART button sender for the ESP32 Wii Remote emulator (wiimote-emu).

Captures real keyboard key-down / key-up events and forwards them to the
firmware over its UART button protocol:

    +<key>   press and hold a button
    -<key>   release a button

Every press and release is sent explicitly, so a button stays held until it
is physically released, and any number of buttons can be held at the same
time. This is what a plain serial terminal cannot do: a terminal only emits
characters on key-down (with OS auto-repeat) and never reports key-up.

Firmware key bytes (see decode_uart_button_key in main/main.c):
    w -> DPAD_RIGHT   a -> DPAD_UP   s -> DPAD_LEFT   d -> DPAD_DOWN
    space -> BUTTON_2   / -> BUTTON_1   h -> HOME

Dependencies are declared in the inline script metadata above and installed
automatically by uv -- just run `uv run uart_sender.py`.

Run this INSTEAD of `idf.py monitor` -- only one process can own the serial
port. Firmware log output is echoed here so you can still see it.

macOS: grant your terminal app "Input Monitoring" (and "Accessibility") under
System Settings > Privacy & Security, otherwise key events are not delivered.
"""

import argparse
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("Missing dependency 'pyserial'.  Install with: pip install pyserial")

try:
    from pynput import keyboard
except ImportError:
    sys.exit("Missing dependency 'pynput'.  Install with: pip install pynput")


DEFAULT_BAUD = 115200

# Normalized key id -> (firmware key byte, button name).
# Both WASD and the arrow keys drive the D-pad; the reference-counted active
# set in ButtonSender keeps overlapping keys for one button consistent.
KEY_MAP = {
    "w": (b"w", "DPAD_RIGHT"),
    "a": (b"a", "DPAD_UP"),
    "s": (b"s", "DPAD_LEFT"),
    "d": (b"d", "DPAD_DOWN"),
    "/": (b"/", "BUTTON_1"),
    "h": (b"h", "HOME"),
    " ": (b" ", "BUTTON_2"),
    keyboard.Key.space: (b" ", "BUTTON_2"),
    keyboard.Key.up:    (b"a", "DPAD_UP"),
    keyboard.Key.down:  (b"d", "DPAD_DOWN"),
    keyboard.Key.left:  (b"s", "DPAD_LEFT"),
    keyboard.Key.right: (b"w", "DPAD_RIGHT"),
}

PORT_HINTS = ("usbserial", "usbmodem", "slab", "wchusb", "ttyusb", "ttyacm")

CONTROLS = """
Controls (captured globally -- this window does not need focus):
    W A S D ......... D-pad   (firmware: W=right  A=up  S=left  D=down)
    Arrow keys ...... D-pad   (mapped to the matching direction)
    Space ........... Button 2
    /  .............. Button 1
    H  .............. HOME
    Esc ............. release all buttons and quit
"""


def key_id(key):
    """Return a stable, hashable identifier for a pynput key event."""
    char = getattr(key, "char", None)
    if char is not None:
        return char.lower()
    return key


class ButtonSender:
    """Translates physical key events into +/- UART frames."""

    def __init__(self, ser):
        self._ser = ser
        self._lock = threading.Lock()
        self._held = set()       # key ids currently physically down
        self._active = {}        # firmware byte -> number of held keys using it

    def press(self, kid):
        if kid not in KEY_MAP or kid in self._held:
            return
        self._held.add(kid)
        byte, name = KEY_MAP[kid]
        with self._lock:
            count = self._active.get(byte, 0)
            self._active[byte] = count + 1
            if count == 0:
                self._send(b"+" + byte)
                print(f"  press   {name}")

    def release(self, kid):
        if kid not in self._held:
            return
        self._held.discard(kid)
        byte, name = KEY_MAP[kid]
        with self._lock:
            count = self._active.get(byte, 0)
            if count <= 1:
                self._active.pop(byte, None)
                self._send(b"-" + byte)
                print(f"  release {name}")
            else:
                self._active[byte] = count - 1

    def release_all(self):
        """Release every held button -- run on exit so nothing sticks."""
        with self._lock:
            for byte in list(self._active):
                self._send(b"-" + byte)
            self._active.clear()
        self._held.clear()

    def _send(self, frame):
        try:
            self._ser.write(frame)
            self._ser.flush()
        except serial.SerialException as exc:
            print(f"serial write failed: {exc}", file=sys.stderr)


def echo_serial(ser, stop_event):
    """Print bytes coming back from the firmware (its debug log shares UART0)."""
    while not stop_event.is_set():
        try:
            data = ser.read(256)
        except serial.SerialException:
            break
        if data:
            sys.stdout.write(data.decode("utf-8", errors="replace"))
            sys.stdout.flush()


def find_port(preferred):
    if preferred:
        return preferred
    devices = [p.device for p in list_ports.comports()]
    candidates = [d for d in devices
                  if any(tag in d.lower() for tag in PORT_HINTS)]
    # macOS exposes each USB serial device twice (/dev/cu.* and /dev/tty.*);
    # the cu.* node is the correct one for an outgoing connection.
    if sys.platform == "darwin":
        cu = [d for d in candidates if "/cu." in d]
        if cu:
            candidates = cu
    return candidates[0] if len(candidates) == 1 else None


def open_serial(port, baud):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.1
    # ESP32 dev boards wire the USB-serial DTR/RTS lines to the auto-reset
    # circuit (EN + GPIO0). EN is only pulled low -- rebooting the chip and
    # dropping the Bluetooth link to the Wii -- when DTR=0 AND RTS=1. Holding
    # DTR asserted while the port opens locks EN high no matter how the OS
    # toggles the lines during open().
    ser.dtr = True
    ser.rts = False
    ser.open()
    # Port is open with RTS already low; releasing DTR now only ever passes
    # through the DTR=0,RTS=0 run state (no reset), and leaves GPIO0 high so
    # a later physical EN press still boots the firmware normally.
    ser.dtr = False
    ser.reset_input_buffer()
    return ser


def main():
    parser = argparse.ArgumentParser(
        description="Keyboard-to-UART button sender for the ESP32 Wii Remote emulator.")
    parser.add_argument("-p", "--port",
                        help="Serial port (e.g. /dev/cu.usbserial-10). Auto-detected if omitted.")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default {DEFAULT_BAUD}).")
    parser.add_argument("-l", "--list-ports", action="store_true",
                        help="List available serial ports and exit.")
    parser.add_argument("--no-echo", action="store_true",
                        help="Do not print firmware log output.")
    args = parser.parse_args()

    if args.list_ports:
        ports = list(list_ports.comports())
        if not ports:
            print("No serial ports found.")
        for p in ports:
            print(f"  {p.device:<28} {p.description}")
        return 0

    port = find_port(args.port)
    if not port:
        print("Could not auto-detect a serial port. "
              "Run with --list-ports, then pass --port.", file=sys.stderr)
        return 1

    try:
        ser = open_serial(port, args.baud)
    except serial.SerialException as exc:
        print(f"Could not open {port}: {exc}", file=sys.stderr)
        return 1

    print(f"Connected to {port} @ {args.baud} baud.")
    print(CONTROLS)
    print("Note: button input is only consumed while the firmware is "
          "HID-connected to the Wii.\n")

    sender = ButtonSender(ser)
    stop_event = threading.Event()

    if not args.no_echo:
        threading.Thread(target=echo_serial, args=(ser, stop_event),
                         daemon=True).start()

    def on_press(key):
        if key == keyboard.Key.esc:
            return False
        sender.press(key_id(key))

    def on_release(key):
        sender.release(key_id(key))

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    try:
        listener.join()
    except KeyboardInterrupt:
        listener.stop()
    finally:
        sender.release_all()
        stop_event.set()
        time.sleep(0.15)
        ser.close()
        print("\nReleased all buttons. Disconnected.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
