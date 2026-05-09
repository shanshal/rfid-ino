#include <WiFi.h>
#include <WiFiManager.h>
#include <HardwareSerial.h>
#include <stdarg.h>

// Pin map (hardware finalized)
const uint8_t PIN_RDM6300_RX = 16;
const uint8_t PIN_LED1 = 32;
const uint8_t PIN_LED2 = 33;
const uint8_t PIN_LED3 = 27;
const uint8_t PIN_BUZZER = 13;
const uint8_t PIN_MOTOR = 26;
const uint8_t PIN_BTN1 = 25;
const uint8_t PIN_SWITCH = 14;

// Timing constants (ms)
const unsigned long DEBOUNCE_MS = 50;
const unsigned long BEEP_DURATION_MS = 150;
const unsigned long DOUBLE_BEEP_GAP_MS = 100;
const unsigned long MOTOR_DURATION_MS = 200;
const unsigned long SCAN_WINDOW_MS = 2000;
const unsigned long SUCCESS_DISPLAY_MS = 600;
const unsigned long FAIL_BLINK_MS = 400;
const unsigned long FAIL_BLINK_TOGGLE_MS = 70;
const unsigned long HEARTBEAT_MS = 3000;
const char* PROVISION_AP_SSID = "RDM6300-Debug-Setup";
const char* PROVISION_AP_PASSWORD = "admin1234";
const unsigned long WIFI_MANAGER_CONNECT_TIMEOUT_SECONDS = 30;
const unsigned long CONFIG_PORTAL_TIMEOUT_SECONDS = 600;

enum State {
  STATE_IDLE,
  STATE_SCANNING,
  STATE_SUCCESS,
  STATE_FAIL,
};

static const char* STATE_NAMES[] = {
  "IDLE", "SCANNING", "SUCCESS", "FAIL"
};

HardwareSerial RFID(2);  // UART2

State currentState = STATE_IDLE;
bool chargingMode = false;

unsigned long scanWindowUntilMs = 0;
unsigned long successDisplayUntilMs = 0;
unsigned long failBlinkUntilMs = 0;
unsigned long failBlinkToggleAtMs = 0;
bool failBlinkState = false;

unsigned long buzzerOffAtMs = 0;
unsigned long buzzerSecondOnAtMs = 0;
unsigned long buzzerSecondOffAtMs = 0;
unsigned long motorOffAtMs = 0;

bool lastRawBtn = LOW;
bool debouncedBtn = LOW;
unsigned long lastBtnChangeMs = 0;

unsigned long lastHeartbeatMs = 0;
char lastTag[11] = {0};

void dbg(const char* msg) {
  Serial.printf("[%lu ms] %s\n", millis(), msg);
}

void dbgf(const char* fmt, ...) {
  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.printf("[%lu ms] %s\n", millis(), buf);
}

void setLeds(bool l1, bool l2, bool l3) {
  digitalWrite(PIN_LED1, l1 ? HIGH : LOW);
  digitalWrite(PIN_LED2, l2 ? HIGH : LOW);
  digitalWrite(PIN_LED3, l3 ? HIGH : LOW);
}

void setBuzzer(bool enabled) {
  digitalWrite(PIN_BUZZER, enabled ? HIGH : LOW);
}

void triggerSuccessFeedback() {
  const unsigned long now = millis();
  setBuzzer(true);
  buzzerOffAtMs = now + BEEP_DURATION_MS;
  buzzerSecondOnAtMs = 0;
  buzzerSecondOffAtMs = 0;

  analogWrite(PIN_MOTOR, 200);
  motorOffAtMs = now + MOTOR_DURATION_MS;

  successDisplayUntilMs = now + SUCCESS_DISPLAY_MS;
  failBlinkUntilMs = 0;
  dbg("Feedback: success");
}

void triggerFailFeedback() {
  const unsigned long now = millis();
  setBuzzer(true);
  buzzerOffAtMs = now + BEEP_DURATION_MS;
  buzzerSecondOnAtMs = now + BEEP_DURATION_MS + DOUBLE_BEEP_GAP_MS;
  buzzerSecondOffAtMs = now + BEEP_DURATION_MS + DOUBLE_BEEP_GAP_MS + BEEP_DURATION_MS;

  analogWrite(PIN_MOTOR, 0);
  motorOffAtMs = 0;
  successDisplayUntilMs = 0;

  failBlinkUntilMs = now + FAIL_BLINK_MS;
  failBlinkState = true;
  failBlinkToggleAtMs = now + FAIL_BLINK_TOGGLE_MS;
  dbg("Feedback: fail");
}

void updateFeedbackActuators() {
  const unsigned long now = millis();

  if (buzzerOffAtMs && (long)(now - buzzerOffAtMs) >= 0) {
    setBuzzer(false);
    buzzerOffAtMs = 0;
  }
  if (buzzerSecondOnAtMs && (long)(now - buzzerSecondOnAtMs) >= 0) {
    setBuzzer(true);
    buzzerSecondOnAtMs = 0;
  }
  if (buzzerSecondOffAtMs && (long)(now - buzzerSecondOffAtMs) >= 0) {
    setBuzzer(false);
    buzzerSecondOffAtMs = 0;
  }

  if (motorOffAtMs && (long)(now - motorOffAtMs) >= 0) {
    analogWrite(PIN_MOTOR, 0);
    motorOffAtMs = 0;
  }
}

void updateLedState() {
  const unsigned long now = millis();

  if (failBlinkUntilMs && (long)(now - failBlinkUntilMs) >= 0) {
    failBlinkUntilMs = 0;
    failBlinkState = false;
  }
  if (failBlinkUntilMs && (long)(now - failBlinkToggleAtMs) >= 0) {
    failBlinkState = !failBlinkState;
    failBlinkToggleAtMs = now + FAIL_BLINK_TOGGLE_MS;
  }

  if (digitalRead(PIN_SWITCH) == HIGH) {
    setLeds(true, true, true);
    return;
  }

  bool led1 = true;
  bool led2 = (currentState == STATE_SCANNING);
  bool led3 = false;

  if (currentState == STATE_SUCCESS && (long)(successDisplayUntilMs - now) > 0) {
    led3 = true;
  }
  if (currentState == STATE_FAIL && failBlinkUntilMs) {
    led3 = failBlinkState;
  }

  setLeds(led1, led2, led3);
}

bool consumeButtonPress() {
  const bool raw = digitalRead(PIN_BTN1);
  if (raw != lastRawBtn) {
    lastBtnChangeMs = millis();
  }
  lastRawBtn = raw;

  if ((millis() - lastBtnChangeMs) <= DEBOUNCE_MS) {
    return false;
  }

  if (raw != debouncedBtn) {
    debouncedBtn = raw;
    if (raw == HIGH) {
      return true;
    }
  }

  return false;
}

bool tryReadTag() {
  if (!RFID.available()) return false;
  if (RFID.read() != 0x02) return false;

  for (int i = 0; i < 10; i++) {
    const unsigned long waitStart = millis();
    while (!RFID.available()) {
      if (millis() - waitStart > 200) return false;
    }
    lastTag[i] = RFID.read();
  }
  lastTag[10] = '\0';

  for (int i = 0; i < 2; i++) {
    const unsigned long waitStart = millis();
    while (!RFID.available()) {
      if (millis() - waitStart > 200) return false;
    }
    RFID.read();
  }

  const unsigned long waitStart = millis();
  while (!RFID.available()) {
    if (millis() - waitStart > 200) return false;
  }
  if (RFID.read() != 0x03) return false;

  return true;
}

void changeState(State nextState) {
  dbgf("State: %s -> %s", STATE_NAMES[currentState], STATE_NAMES[nextState]);
  currentState = nextState;
}

void enterIdle() {
  changeState(STATE_IDLE);
}

void enterScanning() {
  while (RFID.available()) RFID.read();
  scanWindowUntilMs = millis() + SCAN_WINDOW_MS;
  changeState(STATE_SCANNING);
}

void enterSuccess() {
  triggerSuccessFeedback();
  changeState(STATE_SUCCESS);
}

void enterFail() {
  triggerFailFeedback();
  changeState(STATE_FAIL);
}

void heartbeat() {
  const unsigned long now = millis();
  if (now - lastHeartbeatMs < HEARTBEAT_MS) return;
  lastHeartbeatMs = now;
  dbgf("Heartbeat | state=%s switch=%s wifi=%s ip=%s",
       STATE_NAMES[currentState],
       digitalRead(PIN_SWITCH) == HIGH ? "ON" : "OFF",
       WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED",
       WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "-");
}

void connectWiFiForDebug() {
  WiFiManager wm;
  wm.setConnectTimeout(WIFI_MANAGER_CONNECT_TIMEOUT_SECONDS);
  wm.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT_SECONDS);
  const bool ok = wm.autoConnect(PROVISION_AP_SSID, PROVISION_AP_PASSWORD);
  if (ok) {
    dbgf("WiFi connected: %s", WiFi.localIP().toString().c_str());
    return;
  }

  dbg("WiFi not configured/connected. Running local debug without WiFi.");
}

void setup() {
  Serial.begin(115200);
  delay(100);
  dbg("=== RDM6300 local debug firmware booting ===");

  RFID.begin(9600, SERIAL_8N1, PIN_RDM6300_RX, -1);

  pinMode(PIN_BTN1, INPUT);
  pinMode(PIN_SWITCH, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);

  setBuzzer(false);
  analogWrite(PIN_MOTOR, 0);
  setLeds(false, false, false);

  setLeds(true, true, true);
  setBuzzer(true);
  analogWrite(PIN_MOTOR, 200);
  delay(200);
  setBuzzer(false);
  analogWrite(PIN_MOTOR, 0);
  setLeds(false, false, false);

  connectWiFiForDebug();

  enterIdle();
  dbg("Boot complete. Waiting for button press to arm scan.");
}

void loop() {
  if (consumeButtonPress() && currentState == STATE_IDLE) {
    dbg("Button pressed; arming scan window");
    enterScanning();
  }

  if (currentState == STATE_SCANNING) {
    if (tryReadTag()) {
      dbgf("Tag read: %s", lastTag);
      enterSuccess();
    } else if ((long)(millis() - scanWindowUntilMs) >= 0) {
      dbg("Scan timeout (no tag)");
      enterFail();
    }
  } else if (currentState == STATE_SUCCESS) {
    if ((long)(millis() - successDisplayUntilMs) >= 0) {
      enterIdle();
    }
  } else if (currentState == STATE_FAIL) {
    if (!failBlinkUntilMs) {
      enterIdle();
    }
  }

  heartbeat();
  updateFeedbackActuators();
  updateLedState();
}
