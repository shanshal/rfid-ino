# ESP32 + RDM6300 RFID Scanner

Button-triggered RFID tag scanner with LED status indicators, buzzer, and vibration motor feedback.

## Pin Assignments

| GPIO | Component         | Direction    | Notes                      |
|------|-------------------|--------------|----------------------------|
| 16   | RDM6300 TX        | Input (RX2)  | UART2 RX, 9600 baud       |
| 25   | BTN1 (main)       | Input        | 10k pull-down, trigger     |
| 14   | Switch (extra)    | Input        | 10k pull-down, toggle      |
| 26   | Vibration motor   | Output / PWM | BC547 base through 1k      |
| 13   | Active buzzer     | Output       | HIGH = beep                |
| 32   | LED 1 (ready)     | Output       | Through 220 ohm            |
| 33   | LED 2 (scanning)  | Output       | Through 220 ohm            |
| 27   | LED 3 (result)    | Output       | Through 220 ohm            |

These are fixed to the schematic. Do not reassign.

## How It Works

### State Machine

The firmware runs a 4-state machine. Every iteration of `loop()` reads inputs, runs the state machine, then updates outputs (buzzer, motor, LEDs). Everything is **non-blocking** — no `delay()` calls in the main loop.

```
            button press
  [IDLE] ─────────────────> [SCANNING]
    ^                        /       \
    |              tag found /         \ 2s timeout
    |                       v           v
    |               [SUCCESS]         [FAIL]
    |                  |                 |
    └──────────────────┴─────────────────┘
            after feedback duration
```

#### IDLE
- **LED1** ON (device is ready).
- Waits for a button press.
- Everything else is off.

#### SCANNING
- **LED1 + LED2** ON.
- The UART buffer is flushed on entry to discard stale data.
- Reads the RDM6300 serial stream looking for a valid tag frame.
- If a tag is read successfully within 2 seconds, transitions to **SUCCESS**.
- If 2 seconds pass with no tag, transitions to **FAIL**.

#### SUCCESS
- **LED1 + LED3** ON.
- Buzzer plays a single short beep (150ms).
- Motor gives a vibration pulse (200ms at ~78% PWM).
- Tag ID is printed to serial.
- Returns to **IDLE** after 600ms.

#### FAIL
- **LED3** blinks rapidly (3 blinks in 400ms).
- Buzzer plays a double beep (two 150ms beeps with a 100ms gap).
- No motor vibration.
- Returns to **IDLE** after the blink sequence.

### Charging Mode

The switch on GPIO 14 acts as a manual toggle:
- **Switch HIGH** (flipped on): all 3 LEDs stay solid ON regardless of state.
- **Switch LOW** (flipped off): LEDs follow the normal state behavior above.

Scanning still works normally in charging mode — the LEDs just all stay lit.

### Button (Trigger, Not Toggle)

The button on GPIO 25 is a **trigger**. It uses debounced rising-edge detection:

1. Raw pin is read every loop iteration.
2. If the value has been stable for 50ms (debounce), the firmware checks for a LOW-to-HIGH transition.
3. Only that single rising edge fires one scan. Holding the button does nothing extra.

The 10k pull-down resistor keeps the pin at LOW when not pressed. Pressing connects it to 3V3.

```
GND ---[10k]---+--- GPIO 25
               |
3V3 ---[BTN]---+
```

### RDM6300 Protocol

The RDM6300 sends 125kHz RFID tag data over UART at 9600 baud. Each tag frame is:

```
[0x02] [10 ASCII hex chars (tag ID)] [2 checksum bytes] [0x03]
 STX                                                      ETX
```

The firmware:
1. Waits for the start byte `0x02`.
2. Reads 10 bytes as the tag ID string.
3. Skips 2 checksum bytes.
4. Validates the end byte `0x03`.
5. If any byte times out (200ms per byte), the read is discarded.

### Feedback Summary

| Event   | Buzzer           | Motor          | LED3        |
|---------|------------------|----------------|-------------|
| Success | Single beep      | Vibration pulse| Solid ON    |
| Fail    | Double beep      | None           | Blinks 3x   |

### Non-Blocking Timing

All timed actions (buzzer, motor, LED blinks) use `millis()` comparisons instead of `delay()`. Each action records a "turn off at" timestamp, and the corresponding `update*()` function checks it every loop. This keeps the main loop responsive — button presses and serial data are never missed.

## Debug Logging

Every serial print has a `[millis ms]` timestamp prefix. In the source code, every debug line is marked with a `// DEBUG` comment. To strip all debug output for production:

```bash
grep -n "// DEBUG" rfid_scanner.ino
```

Remove or comment out those lines. The `dbg()` and `dbgf()` helper functions can also be deleted once all their call sites are removed.

### Sample Serial Output

```
[100 ms] === ESP32 RDM6300 Scanner booting ===
[260 ms] Boot complete. Entering IDLE.
[3260 ms] Heartbeat | state=IDLE charging=OFF
[5412 ms] Button: pressed (rising edge)
[5412 ms] State: IDLE -> SCANNING
[5823 ms] RDM6300: STX received, reading tag...
[5831 ms] RDM6300: tag read OK -> 0A0042F3B2
[5831 ms] State: SCANNING -> SUCCESS
[5831 ms] Buzzer: single beep
[5831 ms] Motor: vibration pulse
[6431 ms] State: SUCCESS -> IDLE
```

## WiFi + MQTT (Prepared, Not Active)

A commented-out section at the top of the file contains the full WiFi and MQTT implementation, ready to be enabled later. It includes:

- WiFi connection with auto-reconnect
- MQTT client with configurable broker, topics, and credentials
- `publishScan()` — sends tag ID as JSON to `rfid/scan`
- `publishStatus()` — sends device status to `rfid/status`
- Device identity derived from the ESP32 MAC address

To enable, uncomment the block and follow the checklist at the bottom of that section for the `setup()` and `loop()` changes.

## Dependencies

- **Arduino core for ESP32** (includes `HardwareSerial.h`)
- No external libraries required for base functionality
- WiFi/MQTT mode will need: `WiFi.h` (built-in), `PubSubClient` (install via Library Manager)
