| Supported Targets | ESP32 |
| ----------------- | ----- |

# Wii Remote Bluetooth Emulator (ESP32)

This project turns an **ESP32** into something a **Nintendo Wii** can treat like a **Wii Remote** over **Classic Bluetooth**. It started as Espressif’s HID mouse example and was changed so the radio advertises the right name, device class, pairing method, HID report layout, and manufacturer IDs (VID/PID) that a real `Nintendo RVL-CNT-01` uses.

**What it does today:** the Wii can discover, pair, and open an HID connection to the ESP32. Button/accelerometer/IR data are not fully emulated yet—the firmware mainly proves identity and the connection handshake.

**What it does not do:** full 1:1 gameplay emulation (no real button bits, IR camera, rumble, speaker, or extension ports yet).

---

## Table of contents

1. [Bluetooth from zero](#1-bluetooth-from-zero)
2. [How a real Wii Remote talks to a Wii](#2-how-a-real-wii-remote-talks-to-a-wii)
3. [What this firmware does, step by step](#3-what-this-firmware-does-step-by-step)
4. [Every change from the original mouse example](#4-every-change-from-the-original-mouse-example)
5. [Project setup (first time)](#5-project-setup-first-time)
6. [Build, flash, and monitor](#6-build-flash-and-monitor)
7. [Pairing with the Wii (red sync)](#7-pairing-with-the-wii-red-sync)
8. [Reading the serial logs](#8-reading-the-serial-logs)
9. [Configuration options](#9-configuration-options)
10. [Troubleshooting](#10-troubleshooting)
11. [References](#11-references)

---

## 1. Bluetooth from zero

### What is Bluetooth here?

Bluetooth is a **short-range wireless link** between two devices. Your ESP32 has a **Bluetooth radio**. The Wii has one too. They agree on rules so bytes can move back and forth reliably.

Think of it in layers:

```text
┌─────────────────────────────────────────┐
│  Your app: "send button report 0x30"    │  ← main.c
├─────────────────────────────────────────┤
│  HID profile (game controller protocol) │
├─────────────────────────────────────────┤
│  L2CAP (channels / pipes for data)      │
├─────────────────────────────────────────┤
│  SDP (service directory: "what am I?")  │
├─────────────────────────────────────────┤
│  Pairing / security (PIN, encryption)   │
├─────────────────────────────────────────┤
│  Baseband / radio (actual wireless)     │
└─────────────────────────────────────────┘
```

You do not talk to the radio directly. ESP-IDF’s **Bluedroid** stack handles most layers; your code sets **identity** (name, class, descriptors) and reacts to **events** (PIN request, connected, etc.).

### Device vs host

- **HID device** = the thing being controlled (keyboard, mouse, **Wii Remote**). **This ESP32 is the device.**
- **HID host** = the thing that receives input (PC, **Wii console**). The Wii is the host.

Data flows **device → host** as *input reports* (buttons, accelerometer). The host can send **output reports** back (LEDs, rumble, “change reporting mode”).

### Bluetooth address (BD_ADDR)

Every adapter has a unique **6-byte address**, written like `88:f1:55:13:c1:c6`. It is like a MAC address. The Wii and ESP32 use these addresses when pairing and reconnecting.

### Discoverable vs connectable

- **Discoverable:** other devices can **find** you in a scan (like showing up in a Bluetooth device list).
- **Connectable:** other devices can **open a connection** to you.

A Wii Remote in sync mode is both for about **20 seconds**, then it stops advertising loudly.

### Pairing vs connecting

These are easy to confuse:

| Term | Meaning |
|------|---------|
| **Pairing** | Security setup: prove two devices trust each other (on Wii Remotes: special **PIN** derived from addresses). |
| **Connecting** | Opening the actual data session (HID channels) after pairing. |
| **Bonding** | Saving pairing info so reconnect works later without repeating full sync. |

The Wii **must** complete legacy PIN authentication before HID data works reliably on newer remotes.

### SSP vs legacy pairing (important for Wii)

Modern Bluetooth often uses **SSP** (Secure Simple Pairing)—the “compare numbers on screen” style.

**Wii Remotes do not use SSP.** They use **legacy pairing** with a **binary PIN** (six raw bytes), not a typed password like `1234`.

This project **disables SSP** and answers PIN requests in Wii-specific ways (see [section 2](#2-how-a-real-wii-remote-talks-to-a-wii)).

### SDP — the “menu” of services

**SDP (Service Discovery Protocol)** is a small database the device advertises. When the Wii connects, it reads SDP to learn:

- Device name (`Nintendo RVL-CNT-01`)
- That this is an HID device
- HID descriptor length and layout
- **VID/PID** (manufacturer/product IDs) — Nintendo `0x057E` / `0x0306`

Without correct SDP (especially VID/PID), the Wii may refuse the device even if the name looks right.

### HID and report IDs

**HID (Human Interface Device)** is the profile for keyboards, mice, and game controllers.

Instead of one big stream of bytes, HID uses **reports** identified by **report ID** (e.g. `0x30` = basic buttons). The **HID descriptor** is a blob that only says **how long** each report is—not the full button layout (the Wii uses a vendor-specific layout documented on [Wiibrew](https://wiibrew.org/wiki/Wiimote)).

### L2CAP channels the Wii uses

After pairing, HID uses two L2CAP ports (called **PSM**):

| PSM | Direction | Purpose |
|-----|-----------|---------|
| `0x11` | Host → device | Control (setup, SET_REPORT) |
| `0x13` | Device → host | Data (button reports, etc.) |

Bluedroid opens these when the HID connection is established. Your code sends reports with `esp_bt_hid_device_send_report()`.

---

## 2. How a real Wii Remote talks to a Wii

### Physical sync button (what you are imitating)

On a **real** Wii Remote:

1. You press the **red SYNC** button under the battery cover.
2. The remote **disconnects** from whatever it was linked to.
3. For **~20 seconds** it blinks LEDs and is **discoverable + connectable**.
4. On the Wii, you press the **red SYNC** button behind the front flap.
5. The Wii **searches**, finds `Nintendo RVL-CNT-01`, starts **legacy pairing**.
6. The Wii sends a **PIN request**. The remote replies with a **6-byte binary PIN**:
   - **Sync pairing (bonding):** PIN = host (Wii) Bluetooth address **byte-reversed**
   - **1+2 guest pairing:** PIN = remote’s own address **byte-reversed** (temporary, not saved the same way)
7. After **authentication succeeds**, HID channels open.
8. Default input mode is report **`0x30`** (2 bytes of button data).

On this ESP32 project, **pressing EN (reset)** after boot starts the same style **20-second pairing window** (there is no physical sync button unless you add a GPIO later).

### Identity the Wii expects (original RVL-CNT-01)

| Item | Value |
|------|--------|
| Name | `Nintendo RVL-CNT-01` |
| Class of Device | `0x002504` (peripheral / joystick) |
| Vendor ID (SDP) | `0x057E` (Nintendo) |
| Product ID (SDP) | `0x0306` (RVL-CNT-01) |
| Pairing | Legacy only (no SSP) |
| Default report | `0x30`, 2 bytes |

Wii Remote Plus (TR) uses a different product ID (`0x0330`); this project targets the **original** remote.

### PIN example (sync mode)

If the Wii’s Bluetooth address is `11:22:33:44:55:66`, the PIN bytes are:

`66 55 44 33 22 11` (reversed order, **raw bytes**, not ASCII digits).

The firmware computes this automatically when the Wii asks (`ESP_BT_GAP_PIN_REQ_EVT`).

---

## 3. What this firmware does, step by step

When you reset or power on the ESP32, `app_main()` runs this sequence:

```text
Power on / EN reset
    │
    ▼
Initialize NVS, Bluetooth controller (Classic only, no BLE)
    │
    ▼
Start Bluedroid with SSP DISABLED (legacy pairing only)
    │
    ▼
Register GAP callback (pairing, PIN, auth)
Register SDP callback (DI / VID-PID record)
    │
    ▼
Set device name: "Nintendo RVL-CNT-01"
Set Class of Device: joystick / limited discoverable
    │
    ▼
Create SDP Device ID record (057E:0306) via esp_sdp API
    │
    ▼
Register HID device app (Wii report descriptor, not mouse)
Start HID device profile
    │
    ▼
On HID register success → open 20s "sync" pairing window
    (connectable + limited discoverable)
    │
    ▼
[Wii user presses SYNC on console within ~20s]
    │
    ▼
Wii finds device → legacy PIN request → firmware replies with reversed Wii address
    │
    ▼
Authentication complete → HID connection open
    │
    ▼
Firmware sends empty report 0x30, stops being discoverable
```

### Callbacks you will see in code

| Callback | Role |
|----------|------|
| `esp_bt_gap_cb` | Pairing: PIN request, authentication success/fail |
| `esp_sdp_cb` | SDP server: DI record created |
| `esp_bt_hidd_cb` | HID: registered, connected, GET/SET report from Wii |

---

## 4. Every change from the original mouse example

The repo began as Espressif’s **Bluetooth HID mouse** demo. Below is what had to change for Wii compatibility.

### A. Identity and advertising

| Before (mouse) | After (Wii Remote) |
|----------------|-------------------|
| Name `HID Mouse Example` | `Nintendo RVL-CNT-01` |
| CoD pointing device | CoD `0x002504` (peripheral + joystick) |
| HID subclass mouse | `ESP_HID_CLASS_JOS` (joystick) |
| Provider `ESP32` | `Nintendo` |

### B. HID descriptor and reports

| Before | After |
|--------|--------|
| Standard mouse HID descriptor (buttons + X/Y/wheel) | Wii-style descriptor: report IDs `0x10`–`0x1A` (out), `0x20`–`0x3F` (in) with correct **byte lengths** per Wiibrew |
| Mouse movement task sending report ID `0` | Removed; sends default report **`0x30`** (2 bytes) on connect |
| Boot mouse protocol handling | Wii path ignores boot protocol; report-mode only |

### C. Pairing and security

| Before | After |
|--------|--------|
| SSP enabled by default | **`bluedroid_cfg.ssp_en = false`** |
| Fixed ASCII PIN `1234` | **Variable binary PIN**: reversed BD_ADDR (sync = host address) |
| Always discoverable | **20 s sync window**, then non-discoverable if not connected |

New Kconfig option: `EXAMPLE_WIIMOTE_GUEST_PAIRING_MODE` (off by default = sync/Wii bonding behavior).

### D. SDP / Device ID (VID & PID)

| Before | After |
|--------|--------|
| No Nintendo DI record | **`esp_sdp_init()`** + **`esp_sdp_create_record()`** with VID `0x057E`, PID `0x0306` |
| — | Enables `CONFIG_BT_SDP_COMMON_ENABLED` |

This avoids patching Bluedroid internals; VID/PID are set at the **application** level.

### E. SDK configuration fixes (required to boot cleanly)

These were **not** in the original example and were required for your successful connection:

| Setting | Was | Now | Why |
|---------|-----|-----|-----|
| `CONFIG_BT_SDP_COMMON_ENABLED` | off | **y** | Links `esp_sdp_*` APIs |
| `CONFIG_BT_SDP_PAD_LEN` | 300 | **1024** | HID SDP record did not fit; caused `SDP_AddAttribute fail ID 518 (0x0206)` |
| `CONFIG_BT_SDP_ATTR_LEN` | 300 | **1024** | Same—descriptor list attribute overflow |
| `CONFIG_EXAMPLE_SSP_ENABLED` | y | **n** | Wii legacy pairing only |

Defaults live in `sdkconfig.defaults` so new builds pick them up.

### F. Diagnostics

- `[PAIR]` logs in GAP callback (PIN source, auth result)
- `[PAIR][1/4]`…`[4/4]` logs for sync window timing
- DI record log with VID/PID/version
- Comments in `main.c` mapping behavior to Wiibrew

### Files touched

| File | Purpose |
|------|---------|
| `main/main.c` | Core Wii HID + pairing + SDP logic |
| `main/Kconfig.projbuild` | Device name, SSP off, guest pairing option |
| `sdkconfig` / `sdkconfig.defaults` | SDP sizes, SSP, SDP common |

---

## 5. Project setup (first time)

### Hardware

- **ESP32** board with Classic Bluetooth (e.g. ESP32-DevKitC)
- USB cable for power and serial
- A **Wii** console for pairing tests

**DevKitC buttons:**

- **EN** — reset (use this to start a new sync window)
- **BOOT** — only for firmware download mode (hold BOOT, tap EN); not used in normal pairing

### Software

1. Install **ESP-IDF** (this project was built with **v6.0.1**). Follow: [ESP-IDF Get Started](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)

2. In each new terminal, activate the IDF environment:

   ```bash
   get_idf
   ```

   (Or `. $HOME/esp/esp-idf/export.sh` if you do not use the `get_idf` alias.)

3. Clone or open this project folder:

   ```bash
   cd "/Users/lashfi/Desktop/Wii Remote/wiimote-emu"
   ```

4. Set target (if not already ESP32):

   ```bash
   idf.py set-target esp32
   ```

5. Optional: customize options:

   ```bash
   idf.py menuconfig
   ```

   Check under **Component config → Bluetooth**:

   - Bluetooth enabled
   - **Classic Bluetooth** enabled (not BLE-only)
   - **HID** and **HID Device** enabled
   - **SDP** enabled (common SDP)

   Under **HID Example Configuration**:

   - Local device name: `Nintendo RVL-CNT-01`
   - Secure Simple Pairing: **disabled**
   - Guest pairing mode: **off** for Wii sync bonding

---

## 6. Build, flash, and monitor

Replace `PORT` with your serial port (e.g. `/dev/cu.usbserial-10` on macOS, `COM3` on Windows).

```bash
get_idf
cd "/Users/lashfi/Desktop/Wii Remote/wiimote-emu"
idf.py build
idf.py -p PORT flash monitor
```

- **flash** — writes firmware and resets the board
- **monitor** — shows log output (115200 baud)

Exit monitor: **Ctrl+]**

Reset only (monitor already running): **Ctrl+T**, then **Ctrl+R**

### Healthy boot log (what you want to see)

```text
I app_main: === Wii Remote emulation boot ===
I app_main: Target identity: name='Nintendo RVL-CNT-01', VID=0x057E, PID=0x0306
I esp_sdp_cb: SDP initialized; creating Nintendo DI record
I esp_sdp_cb: DI record created (handle=..., vid=0x057E pid=0x0306 ver=0x0100)
I esp_bt_hidd_cb: HID register success; opening sync-style pairing window
I pairing_window: [PAIR][1/4] Entering 20s sync-style discoverable window
I pairing_window: [PAIR][2/4] Waiting for host inquiry/authentication
```

You should **not** see:

```text
E BT_SDP: SDP_AddAttribute fail, length exceed maximum: ID 518
E BT_HIDD: HID_DevAddRecord: failed to complete SDP record
```

If those appear, SDP buffer sizes are too small—see [Troubleshooting](#10-troubleshooting).

---

## 7. Pairing with the Wii (red sync)

### Roles

| Real Wii Remote | This ESP32 project |
|-----------------|-------------------|
| Press red sync on **remote** | Press **EN** (reset) on ESP32 |
| Press red sync on **Wii** | Press red sync on **Wii** (same) |

### Step-by-step

1. Connect ESP32 USB, run `idf.py -p PORT flash monitor` (or monitor after a prior flash).

2. **Reset the ESP32** (EN button). Wait for:
   - `[PAIR][1/4] Entering 20s sync-style discoverable window`

3. Within **20 seconds**, on the **Wii**:
   - Open the **SD card slot flap** on the front
   - Press the **red SYNC** button on the console

4. Watch the serial monitor on the PC:
   - `[PAIR] Legacy PIN request received`
   - `[PAIR] Using sync PIN source: reverse host BD_ADDR`
   - `[PAIR] Authentication complete`
   - `connected to xx:xx:xx:xx:xx:xx`
   - `[PAIR] HID connected; disabling discoverability`

5. On the Wii, the player LED slot should assign (connection succeeded from the Wii’s point of view).

6. If nothing happens before 20s:
   - Logs show `[PAIR][4/4] Pairing window closed`
   - **Reset ESP32 again** and press Wii SYNC **immediately** during the new window

### Timing diagram

```text
ESP32 EN reset          [======== 20s discoverable ========]
Wii SYNC press               [scan + pair + HID open]
                              ↑ do this inside the window
```

### After first successful bond

The Wii may reconnect later without repeating full discovery. If connection is lost, reset ESP32 to open another sync window, or power-cycle the Wii Bluetooth side per your setup.

---

## 8. Reading the serial logs

### Log tags

| Tag | Meaning |
|-----|---------|
| `app_main` | Startup, identity, init order |
| `esp_sdp_cb` | Nintendo VID/PID record |
| `esp_bt_gap_cb` | Pairing and PIN |
| `esp_bt_hidd_cb` | HID registration and connection |
| `pairing_window` | 20-second sync timer |

### Successful pairing sequence (typical)

1. `DI record created` — Wii can read manufacturer IDs  
2. `HID register success` — HID SDP record built (needs large enough `SDP_PAD_LEN`)  
3. `[PAIR][1/4]` — sync window started  
4. `Legacy PIN request` — Wii asked for PIN; firmware auto-replied  
5. `Authentication complete` — pairing OK  
6. `connected to ...` — HID session up  
7. `ESP_HIDD_SEND_REPORT_EVT id:0x30` — default button report sent  

---

## 9. Configuration options

In `idf.py menuconfig` → **HID Example Configuration**:

| Option | Default | Effect |
|--------|---------|--------|
| **Local Device Name** | `Nintendo RVL-CNT-01` | Name Wii sees in inquiry |
| **Secure Simple Pairing** | off | Must stay off for Wii |
| **Use 1+2 guest PIN mode** | off | off = sync bonding PIN (reverse **host** addr); on = guest PIN (reverse **local** addr) |

In `sdkconfig.defaults`:

| Option | Value | Effect |
|--------|-------|--------|
| `CONFIG_BT_SDP_COMMON_ENABLED` | y | Application SDP API |
| `CONFIG_BT_SDP_PAD_LEN` | 1024 | Space for HID SDP attributes |
| `CONFIG_BT_SDP_ATTR_LEN` | 1024 | Max SDP attribute count buffer |

---

## 10. Troubleshooting

### `SDP_AddAttribute fail ... ID 518` / `HID_DevAddRecord: failed`

**Cause:** SDP attribute buffer too small for the Wii HID descriptor + strings.

**Fix:** Ensure `CONFIG_BT_SDP_PAD_LEN=1024` and `CONFIG_BT_SDP_ATTR_LEN=1024`, then `idf.py fullclean build` and reflash.

### `undefined reference to esp_sdp_*`

**Cause:** SDP common module disabled.

**Fix:** Set `CONFIG_BT_SDP_COMMON_ENABLED=y` in menuconfig or `sdkconfig.defaults`.

### Wii does not find the device

- Reset ESP32 and confirm `[PAIR][1/4]` appears.
- Press Wii SYNC **within 20 seconds**.
- Stand a few feet from the console; reduce 2.4 GHz interference.
- Confirm Classic Bluetooth is enabled (not BLE-only mode).

### PIN / authentication fails

- Confirm SSP is **disabled** (menuconfig and `bluedroid_cfg.ssp_en = false`).
- Use **guest pairing mode off** for Wii sync bonding.
- Retry with a fresh EN reset + Wii SYNC.

### Connected once, hard to reconnect

- Normal Wii behavior stores bonded hosts; timing and slot order vary.
- Try reset ESP32, or clear Wii remote pairings in system settings if testing repeatedly.

### Flash size warning

If you see flash detected larger than 2MB in header: harmless for DevKitC with 4MB flash; partition table still uses 2MB unless you change it.

---

## 11. References

- [Wiibrew — Wiimote](https://wiibrew.org/wiki/Wiimote) — reports, pairing, SDP
- [xwiimote PROTOCOL doc](https://github.com/xwiimote/xwiimote/blob/master/doc/PROTOCOL) — pairing PIN rules
- [ESP-IDF Bluetooth HID Device API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_hidd_api.html)
- [ESP-IDF SDP API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_sdp_api.html)

---

## License

Example code carries Espressif SPDX headers (Unlicense OR CC0-1.0). See source file headers for details.
