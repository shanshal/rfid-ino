// rdm6300_mock_wifi.ino
// RDM6300 RFID scanner with simulated network layer.
// Same feedback engine and scan UX as the wifi sketch, no real WiFi/MQTT.
//
// Scenario is selected at boot:
//   - Hold BTN1 (GPIO25) at boot for >1.5s to advance to the next scenario.
//     Selected scenario is persisted in Preferences and survives reboot.
//
// Scenarios:
//   MOCK_HAPPY      -> WiFi OK, MQTT OK -> CONNECTED_READY -> SCANNING
//   MOCK_WIFI_FAIL  -> WiFi fail pattern, then continue to SCANNING for UX testing
//   MOCK_MQTT_FAIL  -> WiFi OK, MQTT fail pattern, then continue to SCANNING

#include <Preferences.h>
#include <HardwareSerial.h>
#include <stdarg.h>

// Forward declarations so arduino-cli's auto-prototype generator
// (inserted before the first function) can reference these types.
enum Mode : int;
enum FeedbackEvent : int;
enum MockScenario : uint8_t;
struct FeedbackFrame;
struct FeedbackPattern;
struct FeedbackEngine;

// ---------------- Pin map ----------------

static const uint8_t PIN_RDM6300_RX = 16;
static const uint8_t PIN_LED1 = 32;
static const uint8_t PIN_LED2 = 33;
static const uint8_t PIN_LED3 = 27;
static const uint8_t PIN_BUZZER = 13;
static const uint8_t PIN_MOTOR = 26;
static const uint8_t PIN_BTN1 = 25;
static const uint8_t PIN_SWITCH = 14;

// ---------------- Timing ----------------

static const unsigned long DEBOUNCE_MS = 50;
static const unsigned long SCAN_WINDOW_MS = 1500;
static const unsigned long FEEDBACK_PRE_DELAY_MS = 500;
static const unsigned long SCENARIO_TOGGLE_HOLD_MS = 1500;

static const uint8_t HAPTIC_PWM = 110;
static const unsigned long HAPTIC_PULSE_MS = 30;

static const unsigned long BEEP_SHORT_MS = 60;
static const unsigned long BEEP_MED_MS = 120;

// Mock-specific delays so the connecting animation has time to be visible.
static const unsigned long MOCK_CONNECTING_MS = 1800;

// ---------------- Mode + scenario ----------------

enum Mode : int {
    MODE_CONNECTING,
    MODE_SCANNING,
};

static const char* modeName(Mode m) {
    switch (m) {
        case MODE_CONNECTING: return "CONNECTING";
        case MODE_SCANNING:   return "SCANNING";
    }
    return "?";
}

enum MockScenario : uint8_t {
    MOCK_HAPPY = 0,
    MOCK_WIFI_FAIL = 1,
    MOCK_MQTT_FAIL = 2,
};

static const uint8_t MOCK_SCENARIO_COUNT = 3;

static const char* scenarioName(MockScenario s) {
    switch (s) {
        case MOCK_HAPPY:     return "HAPPY";
        case MOCK_WIFI_FAIL: return "WIFI_FAIL";
        case MOCK_MQTT_FAIL: return "MQTT_FAIL";
    }
    return "?";
}

// ---------------- Feedback engine (mirror of wifi sketch) ----------------

enum FeedbackEvent : int {
    EVT_NONE,
    EVT_WIFI_CONNECTING_ANIM,
    EVT_WIFI_FAILED,
    EVT_MQTT_FAILED,
    EVT_CONNECTED_READY,
    EVT_SCAN_ARMED,
    EVT_SCAN_TIMEOUT_FAIL,
    EVT_SCAN_SUCCESS,
};

struct FeedbackFrame {
    unsigned long durationMs;
    bool led1;
    bool led2;
    bool led3;
    bool buzzer;
    uint8_t motorPwm;
};

struct FeedbackPattern {
    const FeedbackFrame* frames;
    uint8_t frameCount;
    bool loop;
    unsigned long preDelayMs;
};

static const FeedbackFrame FRAMES_WIFI_CONNECTING[] = {
    { 200, true,  false, false, false, 0 },
    { 200, false, true,  false, false, 0 },
    { 200, false, false, true,  false, 0 },
    { 100, false, false, false, false, 0 },
};

static const FeedbackFrame FRAMES_WIFI_FAILED[] = {
    { 250, true,  false, false, false, 0 },
    { 200, false, false, false, false, 0 },
    { 250, true,  false, false, false, 0 },
    { 200, false, false, false, false, 0 },
    { 250, true,  false, false, false, 0 },
    { 200, false, false, false, false, 0 },
    { 250, true,  false, false, false, 0 },
    { 200, false, false, false, false, 0 },
};

static const FeedbackFrame FRAMES_MQTT_FAILED[] = {
    { 250, true,  true,  false, false, 0 },
    { 200, false, false, false, false, 0 },
    { 250, true,  true,  false, false, 0 },
    { 200, false, false, false, false, 0 },
    { 250, true,  true,  false, false, 0 },
    { 200, false, false, false, false, 0 },
    { 250, true,  true,  false, false, 0 },
    { 200, false, false, false, false, 0 },
};

static const FeedbackFrame FRAMES_CONNECTED_READY[] = {
    { BEEP_SHORT_MS, true,  true,  true,  true,  0 },
    { 120 - BEEP_SHORT_MS, true,  true,  true,  false, 0 },
    { 120, false, false, false, false, 0 },
    { 120, true,  true,  true,  false, 0 },
    { 120, false, false, false, false, 0 },
    { 120, true,  true,  true,  false, 0 },
    { 120, false, false, false, false, 0 },
};

static const FeedbackFrame FRAMES_SCAN_TIMEOUT_FAIL[] = {
    { BEEP_SHORT_MS, false, false, true,  true,  0 },
    { 250 - BEEP_SHORT_MS, false, false, true,  false, 0 },
    { 100, false, false, false, false, 0 },
    { 250, false, false, true,  false, 0 },
    { 150, false, false, false, false, 0 },
};

static const FeedbackFrame FRAMES_SCAN_SUCCESS[] = {
    { HAPTIC_PULSE_MS, true,  true,  true,  true,  HAPTIC_PWM },
    { BEEP_MED_MS - HAPTIC_PULSE_MS, true,  true,  true,  true,  0 },
    { 150 - BEEP_MED_MS, true,  true,  true,  false, 0 },
    { 100, false, false, false, false, 0 },
    { 150, true,  true,  true,  false, 0 },
    { 150, false, false, false, false, 0 },
};

static const FeedbackFrame FRAMES_SCAN_ARMED[] = {
    { 100, false, true, false, false, 0 },
};

static const FeedbackPattern PATTERN_NONE              = { nullptr, 0, false, 0 };
static const FeedbackPattern PATTERN_WIFI_CONNECTING   = { FRAMES_WIFI_CONNECTING,   sizeof(FRAMES_WIFI_CONNECTING)/sizeof(FeedbackFrame),   true,  0 };
static const FeedbackPattern PATTERN_WIFI_FAILED       = { FRAMES_WIFI_FAILED,       sizeof(FRAMES_WIFI_FAILED)/sizeof(FeedbackFrame),       false, 0 };
static const FeedbackPattern PATTERN_MQTT_FAILED       = { FRAMES_MQTT_FAILED,       sizeof(FRAMES_MQTT_FAILED)/sizeof(FeedbackFrame),       false, 0 };
static const FeedbackPattern PATTERN_CONNECTED_READY   = { FRAMES_CONNECTED_READY,   sizeof(FRAMES_CONNECTED_READY)/sizeof(FeedbackFrame),   false, 0 };
static const FeedbackPattern PATTERN_SCAN_ARMED        = { FRAMES_SCAN_ARMED,        sizeof(FRAMES_SCAN_ARMED)/sizeof(FeedbackFrame),        true,  0 };
static const FeedbackPattern PATTERN_SCAN_TIMEOUT_FAIL = { FRAMES_SCAN_TIMEOUT_FAIL, sizeof(FRAMES_SCAN_TIMEOUT_FAIL)/sizeof(FeedbackFrame), false, FEEDBACK_PRE_DELAY_MS };
static const FeedbackPattern PATTERN_SCAN_SUCCESS      = { FRAMES_SCAN_SUCCESS,      sizeof(FRAMES_SCAN_SUCCESS)/sizeof(FeedbackFrame),      false, FEEDBACK_PRE_DELAY_MS };

static const FeedbackPattern& patternFor(FeedbackEvent evt) {
    switch (evt) {
        case EVT_WIFI_CONNECTING_ANIM: return PATTERN_WIFI_CONNECTING;
        case EVT_WIFI_FAILED:          return PATTERN_WIFI_FAILED;
        case EVT_MQTT_FAILED:          return PATTERN_MQTT_FAILED;
        case EVT_CONNECTED_READY:      return PATTERN_CONNECTED_READY;
        case EVT_SCAN_ARMED:           return PATTERN_SCAN_ARMED;
        case EVT_SCAN_TIMEOUT_FAIL:    return PATTERN_SCAN_TIMEOUT_FAIL;
        case EVT_SCAN_SUCCESS:         return PATTERN_SCAN_SUCCESS;
        case EVT_NONE:
        default:                       return PATTERN_NONE;
    }
}

struct FeedbackEngine {
    FeedbackEvent currentEvent = EVT_NONE;
    unsigned long startedAtMs = 0;
    uint8_t frameIndex = 0;
    unsigned long frameStartedAtMs = 0;
    bool inPreDelay = false;
    bool finished = true;

    void trigger(FeedbackEvent evt) {
        currentEvent = evt;
        const FeedbackPattern& p = patternFor(evt);
        startedAtMs = millis();
        frameIndex = 0;
        frameStartedAtMs = startedAtMs + p.preDelayMs;
        inPreDelay = (p.preDelayMs > 0);
        finished = (p.frameCount == 0);
    }

    void clear() {
        currentEvent = EVT_NONE;
        finished = true;
    }

    bool isFinished() const { return finished; }
    FeedbackEvent active() const { return currentEvent; }

    bool tick(bool& led1, bool& led2, bool& led3, bool& buzzer, uint8_t& motorPwm) {
        if (finished || currentEvent == EVT_NONE) return false;
        const FeedbackPattern& p = patternFor(currentEvent);
        const unsigned long now = millis();

        if (inPreDelay) {
            if ((long)(now - (startedAtMs + p.preDelayMs)) < 0) {
                led1 = led2 = led3 = false;
                buzzer = false;
                motorPwm = 0;
                return true;
            }
            inPreDelay = false;
            frameStartedAtMs = now;
        }

        if (p.frameCount == 0) {
            finished = true;
            return false;
        }

        const FeedbackFrame& frame = p.frames[frameIndex];
        if ((long)(now - (frameStartedAtMs + frame.durationMs)) >= 0) {
            frameIndex++;
            frameStartedAtMs = now;
            if (frameIndex >= p.frameCount) {
                if (p.loop) {
                    frameIndex = 0;
                } else {
                    finished = true;
                    return false;
                }
            }
        }

        const FeedbackFrame& active = p.frames[frameIndex];
        led1 = active.led1;
        led2 = active.led2;
        led3 = active.led3;
        buzzer = active.buzzer;
        motorPwm = active.motorPwm;
        return true;
    }
};

// ---------------- Globals ----------------

HardwareSerial RFID(2);
Preferences preferences;
FeedbackEngine feedback;

Mode mode = MODE_CONNECTING;
MockScenario scenario = MOCK_HAPPY;

bool scanArmed = false;
unsigned long scanWindowUntilMs = 0;

bool lastRawBtn = LOW;
bool debouncedBtn = LOW;
unsigned long lastBtnChangeMs = 0;

uint32_t mockScanCounter = 0;

// ---------------- Logging ----------------

void dbgf(const char* fmt, ...) {
    char buf[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.printf("[%lu ms] %s\n", millis(), buf);
}

// ---------------- Outputs ----------------

void writeLeds(bool l1, bool l2, bool l3) {
    digitalWrite(PIN_LED1, l1 ? HIGH : LOW);
    digitalWrite(PIN_LED2, l2 ? HIGH : LOW);
    digitalWrite(PIN_LED3, l3 ? HIGH : LOW);
}

void writeBuzzer(bool on) { digitalWrite(PIN_BUZZER, on ? HIGH : LOW); }
void writeMotor(uint8_t pwm) { analogWrite(PIN_MOTOR, pwm); }

void allOutputsOff() {
    writeLeds(false, false, false);
    writeBuzzer(false);
    writeMotor(0);
}

bool consumeButtonPress() {
    const bool raw = digitalRead(PIN_BTN1);
    if (raw != lastRawBtn) lastBtnChangeMs = millis();
    lastRawBtn = raw;
    if ((millis() - lastBtnChangeMs) <= DEBOUNCE_MS) return false;
    if (raw != debouncedBtn) {
        debouncedBtn = raw;
        if (raw == HIGH) return true;
    }
    return false;
}

void updateFeedbackOutputs() {
    bool l1 = false, l2 = false, l3 = false, buzz = false;
    uint8_t motor = 0;
    const bool active = feedback.tick(l1, l2, l3, buzz, motor);

    if (digitalRead(PIN_SWITCH) == HIGH) {
        writeLeds(true, true, true);
        writeBuzzer(false);
        writeMotor(0);
        return;
    }

    if (active) {
        writeLeds(l1, l2, l3);
        writeBuzzer(buzz);
        writeMotor(motor);
        return;
    }

    writeLeds(false, false, false);
    writeBuzzer(false);
    writeMotor(0);
}

void waitForFeedback() {
    while (!feedback.isFinished()) {
        updateFeedbackOutputs();
        delay(5);
    }
}

void runFeedbackForMs(unsigned long durationMs) {
    const unsigned long until = millis() + durationMs;
    while ((long)(millis() - until) < 0) {
        updateFeedbackOutputs();
        delay(5);
    }
}

// ---------------- Scenario persistence ----------------

MockScenario loadScenario() {
    preferences.begin("mock_cfg", true);
    uint8_t raw = preferences.getUChar("scenario", MOCK_HAPPY);
    preferences.end();
    if (raw >= MOCK_SCENARIO_COUNT) raw = MOCK_HAPPY;
    return static_cast<MockScenario>(raw);
}

void saveScenario(MockScenario s) {
    preferences.begin("mock_cfg", false);
    preferences.putUChar("scenario", static_cast<uint8_t>(s));
    preferences.end();
}

MockScenario nextScenario(MockScenario s) {
    return static_cast<MockScenario>((static_cast<uint8_t>(s) + 1) % MOCK_SCENARIO_COUNT);
}

// Returns true if the user held BTN1 long enough at boot to advance scenario.
bool detectScenarioToggleAtBoot() {
    pinMode(PIN_BTN1, INPUT);
    if (digitalRead(PIN_BTN1) != HIGH) return false;
    const unsigned long start = millis();
    while (digitalRead(PIN_BTN1) == HIGH) {
        if (millis() - start >= SCENARIO_TOGGLE_HOLD_MS) return true;
        delay(10);
    }
    return false;
}

// ---------------- Mock scan + RFID ----------------

bool tryReadTag(char outTag[11]) {
    if (!RFID.available()) return false;
    if (RFID.read() != 0x02) return false;
    for (int i = 0; i < 10; i++) {
        const unsigned long start = millis();
        while (!RFID.available()) {
            if (millis() - start > 200) return false;
        }
        outTag[i] = RFID.read();
    }
    outTag[10] = '\0';
    for (int i = 0; i < 2; i++) {
        const unsigned long start = millis();
        while (!RFID.available()) {
            if (millis() - start > 200) return false;
        }
        RFID.read();
    }
    const unsigned long start = millis();
    while (!RFID.available()) {
        if (millis() - start > 200) return false;
    }
    if (RFID.read() != 0x03) return false;
    return true;
}

// In mock mode "publishing" always succeeds and just logs.
bool mockPublishScan(const char* uid) {
    mockScanCounter++;
    dbgf("MOCK publish scan #%lu uid=%s", (unsigned long)mockScanCounter, uid);
    return true;
}

// ---------------- Mode transitions ----------------

void enterScanningMode() {
    mode = MODE_SCANNING;
    feedback.clear();
    scanArmed = false;
    while (RFID.available()) RFID.read();
    dbgf("Mode -> %s", modeName(mode));
}

void runMockBootSequence(MockScenario s) {
    feedback.trigger(EVT_WIFI_CONNECTING_ANIM);
    runFeedbackForMs(MOCK_CONNECTING_MS);

    switch (s) {
        case MOCK_HAPPY:
            dbgf("MOCK: WiFi OK");
            dbgf("MOCK: MQTT OK");
            feedback.trigger(EVT_CONNECTED_READY);
            waitForFeedback();
            break;
        case MOCK_WIFI_FAIL:
            dbgf("MOCK: WiFi failed (simulated)");
            feedback.trigger(EVT_WIFI_FAILED);
            waitForFeedback();
            break;
        case MOCK_MQTT_FAIL:
            dbgf("MOCK: WiFi OK");
            dbgf("MOCK: MQTT failed (simulated)");
            feedback.trigger(EVT_MQTT_FAILED);
            waitForFeedback();
            break;
    }
    enterScanningMode();
}

// ---------------- Scanning loop ----------------

void loopScanning() {
    if (consumeButtonPress() && !scanArmed) {
        while (RFID.available()) RFID.read();
        scanArmed = true;
        scanWindowUntilMs = millis() + SCAN_WINDOW_MS;
        feedback.trigger(EVT_SCAN_ARMED);
        Serial.println("Scan armed by button press");
    }

    if (!scanArmed) return;

    char tag[11];
    if (tryReadTag(tag)) {
        scanArmed = false;
        feedback.clear();
        dbgf("Tag: %s", tag);
        bool ok = mockPublishScan(tag);
        feedback.trigger(ok ? EVT_SCAN_SUCCESS : EVT_SCAN_TIMEOUT_FAIL);
        return;
    }

    if ((long)(millis() - scanWindowUntilMs) >= 0) {
        scanArmed = false;
        feedback.clear();
        Serial.println("Scan window timeout");
        feedback.trigger(EVT_SCAN_TIMEOUT_FAIL);
    }
}

// ---------------- Setup / loop ----------------

void boardSelfTestPulse() {
    writeLeds(true, true, true);
    writeBuzzer(true);
    writeMotor(HAPTIC_PWM);
    delay(150);
    allOutputsOff();
    delay(80);
}

void setup() {
    Serial.begin(115200);
    delay(50);
    Serial.println();
    Serial.println("=== rdm6300_mock_wifi booting ===");

    pinMode(PIN_BTN1, INPUT);
    pinMode(PIN_SWITCH, INPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_MOTOR, OUTPUT);
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    allOutputsOff();
    boardSelfTestPulse();

    RFID.begin(9600, SERIAL_8N1, PIN_RDM6300_RX, -1);

    scenario = loadScenario();
    if (detectScenarioToggleAtBoot()) {
        scenario = nextScenario(scenario);
        saveScenario(scenario);
        dbgf("Scenario advanced to: %s", scenarioName(scenario));
        // brief acknowledgement: flash LED2 twice
        writeLeds(false, true, false); delay(120); writeLeds(false, false, false); delay(120);
        writeLeds(false, true, false); delay(120); writeLeds(false, false, false); delay(120);
    } else {
        dbgf("Scenario: %s (hold BTN1 at boot >1.5s to advance)", scenarioName(scenario));
    }

    mode = MODE_CONNECTING;
    runMockBootSequence(scenario);
    Serial.println("Ready.");
}

void loop() {
    if (mode == MODE_SCANNING) {
        loopScanning();
    }
    updateFeedbackOutputs();
}
