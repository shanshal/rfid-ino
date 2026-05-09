/*
 * ESP32 + RDM6300 RFID Scanner
 * Button-triggered scan with LED status, buzzer, and vibration motor feedback.
 *
 * Pin assignments match the hardware schematic — do not reassign.
 */

#include <HardwareSerial.h>

// ── WiFi + MQTT (uncomment when ready) ───────────────────────────
// #include <WiFi.h>
// #include <PubSubClient.h>
//
// #define WIFI_SSID          "YOUR_WIFI_SSID"
// #define WIFI_PASSWORD      "YOUR_WIFI_PASSWORD"
//
// #define MQTT_BROKER        "192.168.0.109"
// #define MQTT_PORT          1883
// #define MQTT_USERNAME      ""
// #define MQTT_PASSWORD      ""
//
// #define MQTT_TOPIC_SCAN    "rfid/scan"
// #define MQTT_TOPIC_STATUS  "rfid/status"
//
// #define DEVICE_LABEL       "or1"
// #define DEVICE_ROOM        "OR-1"
//
// WiFiClient wifiClient;
// PubSubClient mqttClient(wifiClient);
// String mqttClientId;
// String deviceId;
//
// void initDeviceIdentity() {
//   String mac = WiFi.macAddress();
//   mac.replace(":", "");
//   mac.toLowerCase();
//   mqttClientId = "esp32-" + mac;
//   deviceId = "esp32-" + String(DEVICE_LABEL) + "-" + mac.substring(max(0, (int)mac.length() - 6));
//   dbgf("MQTT client ID: %s", mqttClientId.c_str());
//   dbgf("Device ID: %s", deviceId.c_str());
// }
//
// void connectWiFi() {
//   if (WiFi.status() == WL_CONNECTED) return;
//   dbg("WiFi: connecting...");
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print('.');
//   }
//   dbgf("WiFi: connected, IP=%s", WiFi.localIP().toString().c_str());
// }
//
// void connectMQTT() {
//   while (!mqttClient.connected()) {
//     dbg("MQTT: connecting...");
//     bool ok = strlen(MQTT_USERNAME) > 0
//       ? mqttClient.connect(mqttClientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)
//       : mqttClient.connect(mqttClientId.c_str());
//     if (ok) {
//       dbg("MQTT: connected");
//       publishStatus("boot");
//       return;
//     }
//     dbgf("MQTT: failed, state=%d", mqttClient.state());
//     delay(2000);
//   }
// }
//
// void ensureConnections() {
//   if (WiFi.status() != WL_CONNECTED) connectWiFi();
//   if (!mqttClient.connected()) connectMQTT();
// }
//
// bool publishScan(const char* uid) {
//   char payload[192];
//   snprintf(payload, sizeof(payload),
//     "{\"device_id\":\"%s\",\"rfid_uid\":\"%s\",\"room\":\"%s\"}",
//     deviceId.c_str(), uid, DEVICE_ROOM);
//   bool ok = mqttClient.publish(MQTT_TOPIC_SCAN, payload);
//   dbgf("MQTT: scan publish %s -> %s", ok ? "ok" : "FAIL", payload);
//   return ok;
// }
//
// bool publishStatus(const char* status) {
//   char payload[160];
//   snprintf(payload, sizeof(payload),
//     "{\"device_id\":\"%s\",\"status\":\"%s\"}",
//     deviceId.c_str(), status);
//   bool ok = mqttClient.publish(MQTT_TOPIC_STATUS, payload);
//   dbgf("MQTT: status publish %s -> %s", ok ? "ok" : "FAIL", payload);
//   return ok;
// }
//
// ── To enable WiFi+MQTT, also uncomment in setup() and loop():
// ──   setup():  initDeviceIdentity(); connectWiFi(); connectMQTT();
// ──   loop():   ensureConnections(); mqttClient.loop();
// ──   SUCCESS:  publishScan(lastTag);
// ──   heartbeat(): publishStatus("alive");
// ─────────────────────────────────────────────────────────────────

// ── Pin assignments (match schematic) ────────────────────────────
#define PIN_RDM6300_RX  16   // UART2 RX, 9600 baud
#define PIN_BTN1        25   // Main button, 10k pull-down
#define PIN_SWITCH      14   // Extra switch, 10k pull-down
#define PIN_MOTOR       26   // BC547 base, through 1k resistor
#define PIN_BUZZER      13   // Active buzzer, HIGH = beep
#define PIN_LED1        32   // Ready indicator
#define PIN_LED2        33   // Result indicator
#define PIN_LED3        27   // Scanning indicator

// ── Timing constants (ms) ────────────────────────────────────────
#define DEBOUNCE_MS          50
#define BEEP_DURATION_MS     150
#define DOUBLE_BEEP_GAP_MS   100
#define MOTOR_DURATION_MS    200
#define SCAN_TIMEOUT_MS      2000
#define HEARTBEAT_MS         3000
#define SUCCESS_DISPLAY_MS   600
#define FAIL_BLINK_MS        400
#define FAIL_BLINK_COUNT     3

// ── State machine ────────────────────────────────────────────────
enum State {
  STATE_IDLE,
  STATE_SCANNING,
  STATE_SUCCESS,
  STATE_FAIL
};

static const char* stateNames[] = {
  "IDLE", "SCANNING", "SUCCESS", "FAIL"
};

State currentState       = STATE_IDLE;
bool  chargingMode       = false;

// ── UART ─────────────────────────────────────────────────────────
HardwareSerial RFID(2);

// ── Timing trackers ──────────────────────────────────────────────
unsigned long lastHeartbeatMs   = 0;
unsigned long scanStartMs       = 0;
unsigned long feedbackStartMs   = 0;

// Button debounce
bool          lastRawBtn        = LOW;
bool          debouncedBtn      = LOW;
unsigned long lastBtnChangeMs   = 0;
bool          btnPressed        = false;

// Buzzer state (non-blocking)
unsigned long buzzerOffMs       = 0;
unsigned long buzzerSecondOnMs  = 0;   // for double-beep
unsigned long buzzerSecondOffMs = 0;

// Motor state (non-blocking)
unsigned long motorOffMs        = 0;

// Fail blink state
unsigned long failBlinkNextMs   = 0;
int           failBlinksLeft    = 0;
bool          failLedOn         = false;

// Last scanned tag
char lastTag[11];

// ── Debug logging helper ─────────────────────────────────────────
// All debug prints go through this so they're easy to find & remove.
// Search for "// DEBUG" to locate every debug line.
void dbg(const char* msg) {                               // DEBUG: helper
  Serial.printf("[%lu ms] %s\n", millis(), msg);          // DEBUG: timestamped log
}

void dbgf(const char* fmt, ...) {                         // DEBUG: helper (formatted)
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.printf("[%lu ms] %s\n", millis(), buf);          // DEBUG: timestamped log
}

// ── LED helpers ──────────────────────────────────────────────────
void setLeds(bool l1, bool l2, bool l3) {
  digitalWrite(PIN_LED1, l1 ? HIGH : LOW);
  digitalWrite(PIN_LED2, l2 ? HIGH : LOW);
  digitalWrite(PIN_LED3, l3 ? HIGH : LOW);
}

void applyLeds() {
  if (chargingMode) {
    setLeds(true, true, true);
    return;
  }

  switch (currentState) {
    case STATE_IDLE:
      setLeds(true, false, false);
      break;
    case STATE_SCANNING:
      setLeds(true, true, false);
      break;
    case STATE_SUCCESS:
      setLeds(true, true, false);
      break;
    case STATE_FAIL:
      // LED2 blinking is handled in updateFailBlink()
      setLeds(true, failLedOn, false);
      break;
  }
}

// ── Buzzer helpers (non-blocking) ────────────────────────────────
void startBeep() {
  digitalWrite(PIN_BUZZER, HIGH);
  buzzerOffMs = millis() + BEEP_DURATION_MS;
  buzzerSecondOnMs  = 0;
  buzzerSecondOffMs = 0;
  dbg("Buzzer: single beep");                             // DEBUG: buzzer feedback
}

void startDoubleBeep() {
  digitalWrite(PIN_BUZZER, HIGH);
  unsigned long now = millis();
  buzzerOffMs       = now + BEEP_DURATION_MS;
  buzzerSecondOnMs  = now + BEEP_DURATION_MS + DOUBLE_BEEP_GAP_MS;
  buzzerSecondOffMs = now + BEEP_DURATION_MS + DOUBLE_BEEP_GAP_MS + BEEP_DURATION_MS;
  dbg("Buzzer: double beep");                             // DEBUG: buzzer feedback
}

void updateBuzzer() {
  unsigned long now = millis();
  if (buzzerOffMs && (long)(now - buzzerOffMs) >= 0) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerOffMs = 0;
  }
  if (buzzerSecondOnMs && (long)(now - buzzerSecondOnMs) >= 0) {
    digitalWrite(PIN_BUZZER, HIGH);
    buzzerSecondOnMs = 0;
  }
  if (buzzerSecondOffMs && (long)(now - buzzerSecondOffMs) >= 0) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerSecondOffMs = 0;
  }
}

// ── Motor helpers (non-blocking) ─────────────────────────────────
void startMotor() {
  analogWrite(PIN_MOTOR, 200);   // ~78% duty
  motorOffMs = millis() + MOTOR_DURATION_MS;
  dbg("Motor: vibration pulse");                          // DEBUG: motor feedback
}

void updateMotor() {
  if (motorOffMs && (long)(millis() - motorOffMs) >= 0) {
    analogWrite(PIN_MOTOR, 0);
    motorOffMs = 0;
  }
}

// ── Fail blink helper ────────────────────────────────────────────
void startFailBlink() {
  failBlinksLeft  = FAIL_BLINK_COUNT;
  failLedOn       = true;
  failBlinkNextMs = millis() + (FAIL_BLINK_MS / (FAIL_BLINK_COUNT * 2));
}

void updateFailBlink() {
  if (failBlinksLeft <= 0) return;
  if ((long)(millis() - failBlinkNextMs) >= 0) {
    failLedOn = !failLedOn;
    if (!failLedOn) failBlinksLeft--;
    failBlinkNextMs = millis() + (FAIL_BLINK_MS / (FAIL_BLINK_COUNT * 2));
  }
}

// ── Button debounce ──────────────────────────────────────────────
void readButton() {
  bool raw = digitalRead(PIN_BTN1);
  btnPressed = false;

  // Reset debounce timer when raw input changes
  if (raw != lastRawBtn) {
    lastBtnChangeMs = millis();
  }
  lastRawBtn = raw;

  // Once stable for DEBOUNCE_MS, accept as new debounced state
  if ((millis() - lastBtnChangeMs) > DEBOUNCE_MS) {
    if (raw != debouncedBtn) {
      // Detect rising edge (LOW → HIGH)
      if (raw == HIGH) {
        btnPressed = true;
        dbg("Button: pressed (rising edge)");             // DEBUG: button event
      }
      debouncedBtn = raw;
    }
  }
}

// ── RDM6300 tag reading ──────────────────────────────────────────
// Returns true if a complete valid tag was read into lastTag[].
bool tryReadTag() {
  if (!RFID.available()) return false;

  if (RFID.read() != 0x02) return false;

  dbg("RDM6300: STX received, reading tag...");           // DEBUG: RFID protocol

  for (int i = 0; i < 10; i++) {
    unsigned long waitStart = millis();
    while (!RFID.available()) {
      if (millis() - waitStart > 200) {
        dbg("RDM6300: timeout reading tag byte");         // DEBUG: RFID timeout
        return false;
      }
    }
    lastTag[i] = RFID.read();
  }
  lastTag[10] = '\0';

  // Skip 2 checksum bytes
  for (int i = 0; i < 2; i++) {
    unsigned long waitStart = millis();
    while (!RFID.available()) {
      if (millis() - waitStart > 200) return false;
    }
    RFID.read();
  }

  // Expect ETX (0x03)
  unsigned long waitStart = millis();
  while (!RFID.available()) {
    if (millis() - waitStart > 200) return false;
  }
  if (RFID.read() != 0x03) {
    dbg("RDM6300: missing ETX, discarding");              // DEBUG: RFID protocol
    return false;
  }

  dbgf("RDM6300: tag read OK -> %s", lastTag);           // DEBUG: RFID result
  return true;
}

// ── State transitions ────────────────────────────────────────────
void changeState(State next) {
  dbgf("State: %s -> %s", stateNames[currentState], stateNames[next]); // DEBUG: state change
  currentState = next;
}

void enterScanning() {
  // Flush any stale data in the UART buffer
  while (RFID.available()) RFID.read();
  scanStartMs = millis();
  changeState(STATE_SCANNING);
}

void enterSuccess() {
  feedbackStartMs = millis();
  startBeep();
  startMotor();
  changeState(STATE_SUCCESS);
}

void enterFail() {
  feedbackStartMs = millis();
  startDoubleBeep();
  startFailBlink();
  changeState(STATE_FAIL);
}

void enterIdle() {
  changeState(STATE_IDLE);
}

// ── Charging mode check ──────────────────────────────────────────
void updateChargingMode() {
  bool sw = digitalRead(PIN_SWITCH);
  if (sw != chargingMode) {
    chargingMode = sw;
    dbgf("Charging mode: %s", chargingMode ? "ON" : "OFF"); // DEBUG: charging toggle
  }
}

// ── Heartbeat ────────────────────────────────────────────────────
void heartbeat() {
  unsigned long now = millis();
  if (now - lastHeartbeatMs < HEARTBEAT_MS) return;
  lastHeartbeatMs = now;
  dbgf("Heartbeat | state=%s charging=%s",                // DEBUG: heartbeat
       stateNames[currentState],
       chargingMode ? "ON" : "OFF");
}

// ── Main state machine loop ──────────────────────────────────────
void stateMachine() {
  switch (currentState) {

    case STATE_IDLE:
      if (btnPressed) {
        enterScanning();
      }
      break;

    case STATE_SCANNING:
      if (tryReadTag()) {
        enterSuccess();
      } else if (millis() - scanStartMs >= SCAN_TIMEOUT_MS) {
        dbg("Scan: timeout, no tag detected");            // DEBUG: scan timeout
        enterFail();
      }
      break;

    case STATE_SUCCESS:
      if (millis() - feedbackStartMs >= SUCCESS_DISPLAY_MS) {
        enterIdle();
      }
      break;

    case STATE_FAIL:
      updateFailBlink();
      if (millis() - feedbackStartMs >= FAIL_BLINK_MS) {
        failLedOn = false;
        failBlinksLeft = 0;
        enterIdle();
      }
      break;
  }
}

// ── Arduino entry points ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(100);
  dbg("=== ESP32 RDM6300 Scanner booting ===");          // DEBUG: boot

  // UART2 for RDM6300 (RX only, TX unused → -1)
  RFID.begin(9600, SERIAL_8N1, PIN_RDM6300_RX, -1);

  // Inputs
  pinMode(PIN_BTN1,   INPUT);
  pinMode(PIN_SWITCH, INPUT);

  // Outputs
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR,  OUTPUT);
  pinMode(PIN_LED1,   OUTPUT);
  pinMode(PIN_LED2,   OUTPUT);
  pinMode(PIN_LED3,   OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  analogWrite(PIN_MOTOR, 0);
  setLeds(false, false, false);

  // Short boot feedback: all LEDs flash + beep + vibrate
  setLeds(true, true, true);
  digitalWrite(PIN_BUZZER, HIGH);
  analogWrite(PIN_MOTOR, 200);
  delay(200);
  digitalWrite(PIN_BUZZER, LOW);
  analogWrite(PIN_MOTOR, 0);
  setLeds(false, false, false);
  delay(100);

  currentState = STATE_IDLE;
  dbg("Boot complete. Entering IDLE.");                   // DEBUG: boot done
}

void loop() {
  readButton();
  updateChargingMode();
  heartbeat();

  stateMachine();

  updateBuzzer();
  updateMotor();
  applyLeds();
}
