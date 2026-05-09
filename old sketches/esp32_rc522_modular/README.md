ESP32 RC522 modular sketch split into Arduino tabs:

- `esp32_rc522_modular.ino`: setup/loop and shared globals
- `rfid_rc522.ino`: RC522 setup, UID reading, buzzer handling
- `mqtt_broker.ino`: WiFi and MQTT connection plus JSON publishing
- `heartbeat.ino`: periodic `rfid/status` heartbeats
- `config.h`: local credentials, pins, topics, and reader identity

Wiring:

- RC522 `SDA` or `SS` -> ESP32 `GPIO 5`
- RC522 `SCK` -> ESP32 `GPIO 18`
- RC522 `MOSI` -> ESP32 `GPIO 23`
- RC522 `MISO` -> ESP32 `GPIO 19`
- RC522 `RST` -> ESP32 `GPIO 22`
- RC522 `3.3V` -> ESP32 `3V3`
- RC522 `GND` -> ESP32 `GND`
- Buzzer `SIG` -> ESP32 `GPIO 26`
- Buzzer `VCC` -> ESP32 `3V3` or `5V` depending on the buzzer module
- Buzzer `GND` -> ESP32 `GND`

Important:

- Use `3.3V` for the RC522, not `5V`.
- If your buzzer stays on all the time, flip `BUZZER_ACTIVE_HIGH` in `config.h` to `false`.

Backend wire protocol:

- scan topic: `scanners/{MAC}/scan`
- status topic: `scanners/{MAC}/status` (retained, also used for LWT)
- scan payload: `{"event_id":"...","device_mac":"...","rfid_uid":"...","scanned_at":"<ISO8601>"}`
- status payload: `{"device_mac":"...","status":"online|alive|offline","uptime_ms":...,"rssi":...,"firmware":"...","at":"<ISO8601>"}`

Device identity is the full WiFi MAC (e.g. `AA:BB:CC:DD:EE:FF`); the backend looks up `Device.mac_address` to resolve which room the scanner sits in. Room assignment is not baked into firmware.

Before flashing:

1. Update `WIFI_SSID` and `WIFI_PASSWORD` in `config.h`.
2. Set `MQTT_BROKER`, `MQTT_USERNAME`, `MQTT_PASSWORD` for your broker.
3. Adjust pins if your wiring differs.
4. Install Arduino libraries `PubSubClient` and `MFRC522`.
