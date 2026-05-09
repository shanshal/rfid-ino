// rdm6300_wifi.ino
// RDM6300 RFID scanner with WiFi + MQTT, WiFiManager provisioning,
// MAC QR code in portal, and shared feedback event engine.
//
// Required libraries:
//   - WiFiManager (tzapu)
//   - PubSubClient (knolleary)
// Uses the QR encoder bundled with the ESP32 Arduino core (qrcode.h /
// esp_qrcode_generate). No extra library install needed.
//
// Pin map (hardware finalized):
//   RDM6300 RX = 16
//   LED1 = 32, LED2 = 33, LED3 = 27
//   BUZZER = 13, MOTOR = 26
//   BTN1 = 25, SWITCH = 14

#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <HardwareSerial.h>
#include <qrcode.h>
#include <time.h>
#include <stdarg.h>

// Forward declarations so arduino-cli's auto-prototype generator
// (inserted before the first function) can reference these types.
enum Mode : int;
enum FeedbackEvent : int;
struct RuntimeConfig;
struct FeedbackFrame;
struct FeedbackPattern;
struct FeedbackEngine;

// ---------------- Configuration ----------------

static const char* PROVISION_AP_SSID = "RDM6300-Setup";
static const char* PROVISION_AP_PASSWORD = "admin1234";

static const char* DEFAULT_MQTT_BROKER = "159.195.41.12";
static const uint16_t DEFAULT_MQTT_PORT = 1883;
static const char* DEFAULT_MQTT_USERNAME = "scanner";
static const char* DEFAULT_MQTT_PASSWORD = "10203040";

static const char* MQTT_TOPIC_PREFIX = "scanners/";
static const char* MQTT_TOPIC_SCAN_SUFFIX = "/scan";
static const char* MQTT_TOPIC_STATUS_SUFFIX = "/status";
static const char* MQTT_TOPIC_ANNOUNCE_SUFFIX = "/announce";

static const char* FIRMWARE_VERSION = "rdm6300-wifi-1.0";
static const char* NTP_SERVER = "pool.ntp.org";

static const unsigned long HEARTBEAT_INTERVAL_MS = 30000;
static const unsigned long WIFI_RETRY_DELAY_MS = 500;
static const unsigned long MQTT_RETRY_DELAY_MS = 2000;
static const unsigned long CONFIG_PORTAL_TIMEOUT_SECONDS = 600;
static const unsigned long WIFI_MANAGER_CONNECT_TIMEOUT_SECONDS = 30;
static const int WIFI_CONNECT_ATTEMPTS = 40;
static const int MQTT_CONNECT_ATTEMPTS = 3;
static const uint8_t MANUAL_PORTAL_PIN = 0;
static const unsigned long MANUAL_PORTAL_HOLD_MS = 3000;

// ---------------- Pin map ----------------

static const uint8_t PIN_RDM6300_RX = 16;
static const uint8_t PIN_RDM6300_TX = 17;
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

// Haptic tuning ("iPhone-like" tap). Adjust to your motor.
static const uint8_t HAPTIC_PWM = 110;
static const unsigned long HAPTIC_PULSE_MS = 30;

// Buzzer beep durations.
static const unsigned long BEEP_SHORT_MS = 60;
static const unsigned long BEEP_MED_MS = 120;

// ---------------- Mode enum ----------------

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

// ---------------- Feedback event engine ----------------

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

// WiFi connecting: chase 1->2->3 with brief gap, looping.
static const FeedbackFrame FRAMES_WIFI_CONNECTING[] = {
    { 200, true,  false, false, false, 0 },
    { 200, false, true,  false, false, 0 },
    { 200, false, false, true,  false, 0 },
    { 100, false, false, false, false, 0 },
};

// WiFi failed: LED1 blinks slowly 4 times.
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

// MQTT failed: LED1 + LED2 blink slowly 4 times.
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

// Connected ready: all three flash 3 times with a short beep on first frame.
static const FeedbackFrame FRAMES_CONNECTED_READY[] = {
    { BEEP_SHORT_MS, true,  true,  true,  true,  0 },
    { 120 - BEEP_SHORT_MS, true,  true,  true,  false, 0 },
    { 120, false, false, false, false, 0 },
    { 120, true,  true,  true,  false, 0 },
    { 120, false, false, false, false, 0 },
    { 120, true,  true,  true,  false, 0 },
    { 120, false, false, false, false, 0 },
};

// Scan timeout fail: LED3 double-blink + single short beep on first frame.
// No haptic on failure (haptic reserved for successful reads).
static const FeedbackFrame FRAMES_SCAN_TIMEOUT_FAIL[] = {
    { BEEP_SHORT_MS, false, false, true,  true,  0 },
    { 250 - BEEP_SHORT_MS, false, false, true,  false, 0 },
    { 100, false, false, false, false, 0 },
    { 250, false, false, true,  false, 0 },
    { 150, false, false, false, false, 0 },
};

// Scan success: all three flash twice with beep + haptic pulse on first frame.
static const FeedbackFrame FRAMES_SCAN_SUCCESS[] = {
    { HAPTIC_PULSE_MS, true,  true,  true,  true,  HAPTIC_PWM },
    { BEEP_MED_MS - HAPTIC_PULSE_MS, true,  true,  true,  true,  0 },
    { 150 - BEEP_MED_MS, true,  true,  true,  false, 0 },
    { 100, false, false, false, false, 0 },
    { 150, true,  true,  true,  false, 0 },
    { 150, false, false, false, false, 0 },
};

// Scan armed: steady LED2 (engine repeats single frame).
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

    // Returns true if outputs were written (non-pre-delay frame active).
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
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Preferences preferences;
FeedbackEngine feedback;

struct RuntimeConfig {
    String mqttBroker;
    uint16_t mqttPort;
    String mqttUsername;
    String mqttPassword;
};

RuntimeConfig runtimeConfig;

String deviceMac;
String scanTopic;
String statusTopic;
String announceTopic;
String lwtPayload;
String portalCustomMenuHtml;

Mode mode = MODE_CONNECTING;
unsigned long lastHeartbeatMs = 0;
uint32_t scanCounter = 0;

bool scanArmed = false;
unsigned long scanWindowUntilMs = 0;

bool lastRawBtn = LOW;
bool debouncedBtn = LOW;
unsigned long lastBtnChangeMs = 0;

// ---------------- Logging helpers ----------------

void dbgf(const char* fmt, ...) {
    char buf[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.printf("[%lu ms] %s\n", millis(), buf);
}

// ---------------- Output helpers ----------------

void writeLeds(bool l1, bool l2, bool l3) {
    digitalWrite(PIN_LED1, l1 ? HIGH : LOW);
    digitalWrite(PIN_LED2, l2 ? HIGH : LOW);
    digitalWrite(PIN_LED3, l3 ? HIGH : LOW);
}

void writeBuzzer(bool on) {
    digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

void writeMotor(uint8_t pwm) {
    analogWrite(PIN_MOTOR, pwm);
}

void allOutputsOff() {
    writeLeds(false, false, false);
    writeBuzzer(false);
    writeMotor(0);
}

// ---------------- Button ----------------

bool consumeButtonPress() {
    const bool raw = digitalRead(PIN_BTN1);
    if (raw != lastRawBtn) {
        lastBtnChangeMs = millis();
    }
    lastRawBtn = raw;
    if ((millis() - lastBtnChangeMs) <= DEBOUNCE_MS) return false;
    if (raw != debouncedBtn) {
        debouncedBtn = raw;
        if (raw == HIGH) return true;
    }
    return false;
}

// ---------------- Output update loop ----------------

void updateFeedbackOutputs() {
    bool l1 = false, l2 = false, l3 = false, buzz = false;
    uint8_t motor = 0;
    const bool active = feedback.tick(l1, l2, l3, buzz, motor);

    // Charge switch overrides everything to "all on".
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

    // No active pattern: idle baseline by mode.
    if (mode == MODE_SCANNING) {
        writeLeds(false, false, false);
    } else {
        writeLeds(false, false, false);
    }
    writeBuzzer(false);
    writeMotor(0);
}

// Block until the current feedback pattern finishes (still pumps outputs).
void waitForFeedback() {
    while (!feedback.isFinished()) {
        updateFeedbackOutputs();
        delay(5);
    }
}

// ---------------- Config persistence ----------------

bool parsePort(const String& raw, uint16_t& outPort) {
    if (raw.length() == 0) return false;
    long parsed = raw.toInt();
    if (parsed < 1 || parsed > 65535) return false;
    outPort = static_cast<uint16_t>(parsed);
    return true;
}

void loadRuntimeConfig() {
    preferences.begin("rfid_cfg", true);
    runtimeConfig.mqttBroker = preferences.getString("mqtt_host", DEFAULT_MQTT_BROKER);
    runtimeConfig.mqttPort = preferences.getUShort("mqtt_port", DEFAULT_MQTT_PORT);
    runtimeConfig.mqttUsername = preferences.getString("mqtt_user", DEFAULT_MQTT_USERNAME);
    runtimeConfig.mqttPassword = preferences.getString("mqtt_pass", DEFAULT_MQTT_PASSWORD);
    preferences.end();
}

void saveRuntimeConfig(const RuntimeConfig& cfg) {
    preferences.begin("rfid_cfg", false);
    preferences.putString("mqtt_host", cfg.mqttBroker);
    preferences.putUShort("mqtt_port", cfg.mqttPort);
    preferences.putString("mqtt_user", cfg.mqttUsername);
    preferences.putString("mqtt_pass", cfg.mqttPassword);
    preferences.end();
}

bool hasValidRuntimeConfig() {
    return runtimeConfig.mqttBroker.length() > 0 && runtimeConfig.mqttPort > 0;
}

// ---------------- Portal QR generation ----------------

// The Espressif QR encoder hands the matrix back via a callback that
// only owns the handle for the duration of the call. We build the SVG
// in the callback, parking it in this static so the outer function
// can resume.
static String* g_qrSvgTarget = nullptr;
static const uint8_t QR_PIX_SIZE = 6;

static void qrSvgWriter(esp_qrcode_handle_t qr) {
    if (!g_qrSvgTarget) return;
    String& svg = *g_qrSvgTarget;
    const int dim = esp_qrcode_get_size(qr);
    svg += "<svg xmlns='http://www.w3.org/2000/svg' width='";
    svg += String(dim * QR_PIX_SIZE);
    svg += "' height='";
    svg += String(dim * QR_PIX_SIZE);
    svg += "' viewBox='0 0 ";
    svg += String(dim);
    svg += " ";
    svg += String(dim);
    svg += "' shape-rendering='crispEdges'>";
    svg += "<rect width='100%' height='100%' fill='#ffffff'/>";
    for (int y = 0; y < dim; y++) {
        for (int x = 0; x < dim; x++) {
            if (esp_qrcode_get_module(qr, x, y)) {
                svg += "<rect x='";
                svg += String(x);
                svg += "' y='";
                svg += String(y);
                svg += "' width='1' height='1' fill='#000000'/>";
            }
        }
    }
    svg += "</svg>";
}

// Build inline SVG QR for the device MAC (no separators, uppercase),
// plus a small device-info card. Injected into WiFiManager pages via
// setCustomMenuHTML so it appears at the bottom of every screen.
void buildPortalCustomHtml() {
    String macCompact = deviceMac;
    macCompact.replace(":", "");
    macCompact.toUpperCase();

    String svg;
    svg.reserve(2048);
    g_qrSvgTarget = &svg;

    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    cfg.display_func = qrSvgWriter;
    cfg.max_qrcode_version = 4;          // 33 modules; covers MAC easily at ECC L
    cfg.qrcode_ecc_level = ESP_QRCODE_ECC_LOW;
    if (esp_qrcode_generate(&cfg, macCompact.c_str()) != ESP_OK) {
        Serial.println("QR encode failed");
        svg = "<div style='color:#a00'>QR encode failed</div>";
    }
    g_qrSvgTarget = nullptr;

    String html;
    html.reserve(svg.length() + 600);
    html += "<style>"
            ".rdm-card{margin:14px auto;max-width:360px;padding:14px;border:1px solid #ccc;border-radius:8px;font-family:sans-serif;text-align:center;background:#fafafa;}"
            ".rdm-card h3{margin:0 0 8px;font-size:16px;}"
            ".rdm-card .qr{margin:8px auto 6px;display:inline-block;background:#fff;padding:6px;border:1px solid #eee;}"
            ".rdm-kv{font-size:13px;text-align:left;margin:6px 0;}"
            ".rdm-kv span{display:inline-block;min-width:90px;color:#555;}"
            "</style>"
            "<div class='rdm-card'>"
            "<h3>Device info</h3>"
            "<div class='qr'>";
    html += svg;
    html += "</div>";
    html += "<div class='rdm-kv'><span>MAC</span>" + deviceMac + "</div>";
    html += "<div class='rdm-kv'><span>Firmware</span>" + String(FIRMWARE_VERSION) + "</div>";
    html += "<div class='rdm-kv'><span>Broker</span>" + runtimeConfig.mqttBroker + ":" + String(runtimeConfig.mqttPort) + "</div>";
    html += "<div class='rdm-kv'><span>MQTT user</span>" + runtimeConfig.mqttUsername + "</div>";
    html += "<div class='rdm-kv'><span>Topic</span>" + scanTopic + "</div>";
    html += "<div class='rdm-kv'><span>Announce</span>" + announceTopic + "</div>";
    html += "</div>";

    portalCustomMenuHtml = html;
}

// ---------------- WiFiManager portal ----------------

bool runProvisioningPortal(bool forcePortal) {
    WiFiManager wm;

    char mqttHost[64];
    char mqttPort[8];
    char mqttUser[64];
    char mqttPass[64];

    runtimeConfig.mqttBroker.toCharArray(mqttHost, sizeof(mqttHost));
    snprintf(mqttPort, sizeof(mqttPort), "%u", runtimeConfig.mqttPort);
    runtimeConfig.mqttUsername.toCharArray(mqttUser, sizeof(mqttUser));
    runtimeConfig.mqttPassword.toCharArray(mqttPass, sizeof(mqttPass));

    WiFiManagerParameter pHost("mqtt_host", "MQTT broker host", mqttHost, sizeof(mqttHost));
    WiFiManagerParameter pPort("mqtt_port", "MQTT broker port", mqttPort, sizeof(mqttPort));
    WiFiManagerParameter pUser("mqtt_user", "MQTT username", mqttUser, sizeof(mqttUser));
    WiFiManagerParameter pPass("mqtt_pass", "MQTT password", mqttPass, sizeof(mqttPass));

    wm.addParameter(&pHost);
    wm.addParameter(&pPort);
    wm.addParameter(&pUser);
    wm.addParameter(&pPass);
    wm.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT_SECONDS);
    wm.setConnectTimeout(WIFI_MANAGER_CONNECT_TIMEOUT_SECONDS);

    if (deviceMac.length() == 0) {
        deviceMac = WiFi.macAddress();
        deviceMac.toUpperCase();
    }
    scanTopic = String(MQTT_TOPIC_PREFIX) + deviceMac + MQTT_TOPIC_SCAN_SUFFIX;
    announceTopic = String(MQTT_TOPIC_PREFIX) + deviceMac + MQTT_TOPIC_ANNOUNCE_SUFFIX;
    buildPortalCustomHtml();
    wm.setCustomMenuHTML(portalCustomMenuHtml.c_str());

    bool wifiReady = false;
    if (forcePortal) {
        Serial.println("Starting setup portal (forced mode)");
        wifiReady = wm.startConfigPortal(PROVISION_AP_SSID, PROVISION_AP_PASSWORD);
    } else {
        wifiReady = wm.autoConnect(PROVISION_AP_SSID, PROVISION_AP_PASSWORD);
    }

    if (!wifiReady) {
        Serial.println("Provisioning portal timed out or connection failed");
        return false;
    }

    RuntimeConfig updated;
    updated.mqttBroker = String(pHost.getValue());
    updated.mqttBroker.trim();
    updated.mqttUsername = String(pUser.getValue());
    updated.mqttUsername.trim();
    updated.mqttPassword = String(pPass.getValue());
    String portRaw = String(pPort.getValue());
    portRaw.trim();

    if (!parsePort(portRaw, updated.mqttPort) || updated.mqttBroker.length() == 0) {
        Serial.println("Invalid MQTT host/port from setup portal");
        return false;
    }

    runtimeConfig = updated;
    saveRuntimeConfig(runtimeConfig);
    dbgf("Saved MQTT target: %s:%u user=%s",
         runtimeConfig.mqttBroker.c_str(), runtimeConfig.mqttPort, runtimeConfig.mqttUsername.c_str());
    return true;
}

bool shouldForcePortalAtBoot() {
    pinMode(MANUAL_PORTAL_PIN, INPUT_PULLUP);
    if (digitalRead(MANUAL_PORTAL_PIN) == HIGH) return false;
    const unsigned long start = millis();
    while (millis() - start < MANUAL_PORTAL_HOLD_MS) {
        if (digitalRead(MANUAL_PORTAL_PIN) == HIGH) return false;
        delay(10);
    }
    Serial.println("Manual portal trigger detected (BOOT held)");
    return true;
}

// ---------------- WiFi / MQTT ----------------

const char* mqttStateText(int state) {
    switch (state) {
        case MQTT_CONNECTION_TIMEOUT: return "connection_timeout";
        case MQTT_CONNECTION_LOST: return "connection_lost";
        case MQTT_CONNECT_FAILED: return "connect_failed";
        case MQTT_DISCONNECTED: return "disconnected";
        case MQTT_CONNECTED: return "connected";
        case MQTT_CONNECT_BAD_PROTOCOL: return "bad_protocol";
        case MQTT_CONNECT_BAD_CLIENT_ID: return "bad_client_id";
        case MQTT_CONNECT_UNAVAILABLE: return "broker_unavailable";
        case MQTT_CONNECT_BAD_CREDENTIALS: return "bad_credentials";
        case MQTT_CONNECT_UNAUTHORIZED: return "unauthorized";
        default: return "unknown";
    }
}

bool connectWiFiNonBlocking() {
    if (WiFi.status() == WL_CONNECTED) return true;
    WiFi.mode(WIFI_STA);
    WiFi.begin();
    Serial.print("Connecting WiFi");
    for (int i = 0; i < WIFI_CONNECT_ATTEMPTS; i++) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println();
            dbgf("WiFi IP: %s", WiFi.localIP().toString().c_str());
            return true;
        }
        // Pump feedback while waiting so the connecting animation runs.
        const unsigned long until = millis() + WIFI_RETRY_DELAY_MS;
        while ((long)(millis() - until) < 0) {
            updateFeedbackOutputs();
            delay(10);
        }
        Serial.print('.');
    }
    Serial.println(" failed");
    return false;
}

void initDeviceIdentity() {
    deviceMac = WiFi.macAddress();
    deviceMac.toUpperCase();
    scanTopic = String(MQTT_TOPIC_PREFIX) + deviceMac + MQTT_TOPIC_SCAN_SUFFIX;
    statusTopic = String(MQTT_TOPIC_PREFIX) + deviceMac + MQTT_TOPIC_STATUS_SUFFIX;
    announceTopic = String(MQTT_TOPIC_PREFIX) + deviceMac + MQTT_TOPIC_ANNOUNCE_SUFFIX;

    char will[128];
    snprintf(will, sizeof(will), "{\"device_mac\":\"%s\",\"status\":\"offline\"}", deviceMac.c_str());
    lwtPayload = String(will);
    dbgf("MAC=%s scan=%s status=%s announce=%s",
         deviceMac.c_str(), scanTopic.c_str(), statusTopic.c_str(), announceTopic.c_str());
}

void syncTime() {
    configTime(0, 0, NTP_SERVER);
    Serial.print("Syncing time");
    for (int i = 0; i < 20; i++) {
        if (time(nullptr) > 1700000000) {
            Serial.println(" ok");
            return;
        }
        delay(250);
        Serial.print('.');
    }
    Serial.println(" timeout");
}

bool getIsoTimestamp(char* out, size_t out_len) {
    time_t now = time(nullptr);
    if (now < 1700000000) return false;
    struct tm tm_utc;
    gmtime_r(&now, &tm_utc);
    strftime(out, out_len, "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
    return true;
}

bool publishStatus(const char* status) {
    char ts[32];
    bool have_ts = getIsoTimestamp(ts, sizeof(ts));
    char payload[320];
    if (have_ts) {
        snprintf(payload, sizeof(payload),
            "{\"device_mac\":\"%s\",\"status\":\"%s\",\"uptime_ms\":%lu,\"rssi\":%d,\"firmware\":\"%s\",\"at\":\"%s\"}",
            deviceMac.c_str(), status, (unsigned long)millis(), WiFi.RSSI(), FIRMWARE_VERSION, ts);
    } else {
        snprintf(payload, sizeof(payload),
            "{\"device_mac\":\"%s\",\"status\":\"%s\",\"uptime_ms\":%lu,\"rssi\":%d,\"firmware\":\"%s\",\"at\":null}",
            deviceMac.c_str(), status, (unsigned long)millis(), WiFi.RSSI(), FIRMWARE_VERSION);
    }
    return mqttClient.publish(statusTopic.c_str(), payload, true);
}

bool publishAnnounce() {
    char ts[32];
    bool have_ts = getIsoTimestamp(ts, sizeof(ts));
    char payload[512];
    if (have_ts) {
        snprintf(payload, sizeof(payload),
            "{\"device_mac\":\"%s\",\"local_ip\":\"%s\",\"scan_topic\":\"%s\",\"status_topic\":\"%s\",\"mqtt_host\":\"%s\",\"mqtt_port\":%u,\"firmware\":\"%s\",\"at\":\"%s\"}",
            deviceMac.c_str(),
            WiFi.localIP().toString().c_str(),
            scanTopic.c_str(),
            statusTopic.c_str(),
            runtimeConfig.mqttBroker.c_str(),
            runtimeConfig.mqttPort,
            FIRMWARE_VERSION,
            ts);
    } else {
        snprintf(payload, sizeof(payload),
            "{\"device_mac\":\"%s\",\"local_ip\":\"%s\",\"scan_topic\":\"%s\",\"status_topic\":\"%s\",\"mqtt_host\":\"%s\",\"mqtt_port\":%u,\"firmware\":\"%s\",\"at\":null}",
            deviceMac.c_str(),
            WiFi.localIP().toString().c_str(),
            scanTopic.c_str(),
            statusTopic.c_str(),
            runtimeConfig.mqttBroker.c_str(),
            runtimeConfig.mqttPort,
            FIRMWARE_VERSION);
    }
    bool ok = mqttClient.publish(announceTopic.c_str(), payload, true);
    dbgf("Announce publish %s: %s", ok ? "ok" : "failed", payload);
    return ok;
}

bool publishScan(const char* uid) {
    char ts[32];
    bool have_ts = getIsoTimestamp(ts, sizeof(ts));
    scanCounter++;
    char event_id[80];
    snprintf(event_id, sizeof(event_id), "%s-%lu-%lu",
             deviceMac.c_str(), (unsigned long)millis(), (unsigned long)scanCounter);
    char payload[320];
    if (have_ts) {
        snprintf(payload, sizeof(payload),
            "{\"event_id\":\"%s\",\"device_mac\":\"%s\",\"rfid_uid\":\"%s\",\"scanned_at\":\"%s\"}",
            event_id, deviceMac.c_str(), uid, ts);
    } else {
        snprintf(payload, sizeof(payload),
            "{\"event_id\":\"%s\",\"device_mac\":\"%s\",\"rfid_uid\":\"%s\",\"scanned_at\":null}",
            event_id, deviceMac.c_str(), uid);
    }
    bool ok = mqttClient.publish(scanTopic.c_str(), payload, false);
    dbgf("Scan publish %s: %s", ok ? "ok" : "failed", payload);
    return ok;
}

bool connectMQTT() {
    if (mqttClient.connected()) return true;
    for (int attempt = 0; attempt < MQTT_CONNECT_ATTEMPTS; attempt++) {
        Serial.print("Connecting MQTT...");
        bool connected = mqttClient.connect(
            deviceMac.c_str(),
            runtimeConfig.mqttUsername.length() > 0 ? runtimeConfig.mqttUsername.c_str() : nullptr,
            runtimeConfig.mqttPassword.length() > 0 ? runtimeConfig.mqttPassword.c_str() : nullptr,
            statusTopic.c_str(),
            1, true, lwtPayload.c_str()
        );
        if (connected) {
            Serial.println("connected");
            publishStatus("online");
            publishAnnounce();
            lastHeartbeatMs = millis();
            return true;
        }
        dbgf("failed state=%d (%s) host=%s:%u user=%s",
             mqttClient.state(), mqttStateText(mqttClient.state()),
             runtimeConfig.mqttBroker.c_str(), runtimeConfig.mqttPort,
             runtimeConfig.mqttUsername.c_str());

        const unsigned long until = millis() + MQTT_RETRY_DELAY_MS;
        while ((long)(millis() - until) < 0) {
            updateFeedbackOutputs();
            delay(10);
        }
    }
    return false;
}

void publishHeartbeatIfDue() {
    unsigned long now = millis();
    if (now - lastHeartbeatMs < HEARTBEAT_INTERVAL_MS) return;
    lastHeartbeatMs = now;
    publishStatus("alive");
    publishAnnounce();
}

// ---------------- Mode transitions ----------------

void enterScanningMode() {
    mode = MODE_SCANNING;
    feedback.clear();
    scanArmed = false;
    while (RFID.available()) RFID.read();
    dbgf("Mode -> %s", modeName(mode));
}

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
    Serial.println("=== rdm6300_wifi booting ===");

    pinMode(PIN_BTN1, INPUT);
    pinMode(PIN_SWITCH, INPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_MOTOR, OUTPUT);
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    allOutputsOff();
    boardSelfTestPulse();

    RFID.begin(9600, SERIAL_8N1, PIN_RDM6300_RX, PIN_RDM6300_TX);

    const bool forcePortalAtBoot = shouldForcePortalAtBoot();

    loadRuntimeConfig();
    if (!hasValidRuntimeConfig()) {
        runtimeConfig.mqttBroker = DEFAULT_MQTT_BROKER;
        runtimeConfig.mqttPort = DEFAULT_MQTT_PORT;
        runtimeConfig.mqttUsername = DEFAULT_MQTT_USERNAME;
        runtimeConfig.mqttPassword = DEFAULT_MQTT_PASSWORD;
    }

    mode = MODE_CONNECTING;
    feedback.trigger(EVT_WIFI_CONNECTING_ANIM);

    if (!runProvisioningPortal(forcePortalAtBoot || !hasValidRuntimeConfig())) {
        Serial.println("Provisioning failed; showing WIFI_FAILED then restart");
        feedback.trigger(EVT_WIFI_FAILED);
        waitForFeedback();
        delay(1500);
        ESP.restart();
    }

    initDeviceIdentity();
    syncTime();

    feedback.trigger(EVT_WIFI_CONNECTING_ANIM);
    mqttClient.setServer(runtimeConfig.mqttBroker.c_str(), runtimeConfig.mqttPort);
    mqttClient.setBufferSize(512);

    if (!connectMQTT()) {
        Serial.println("MQTT failed at boot");
        feedback.trigger(EVT_MQTT_FAILED);
        waitForFeedback();
        // Stay in CONNECTING; ensureConnections() in loop will keep trying.
        return;
    }

    feedback.trigger(EVT_CONNECTED_READY);
    waitForFeedback();
    enterScanningMode();
    Serial.println("Ready.");
}

void ensureConnections() {
    auto enterConnectingIfNeeded = []() {
        if (mode != MODE_CONNECTING) {
            mode = MODE_CONNECTING;
            dbgf("Mode -> %s", modeName(mode));
        }
        // Re-arm the connecting animation any time we land here without an
        // active feedback event (e.g. after a fail pattern finished playing).
        if (feedback.isFinished() || feedback.active() != EVT_WIFI_CONNECTING_ANIM) {
            feedback.trigger(EVT_WIFI_CONNECTING_ANIM);
        }
    };

    if (WiFi.status() != WL_CONNECTED) {
        enterConnectingIfNeeded();
        if (!connectWiFiNonBlocking()) {
            feedback.trigger(EVT_WIFI_FAILED);
            waitForFeedback();
            return;
        }
    }

    if (!mqttClient.connected()) {
        enterConnectingIfNeeded();
        if (!connectMQTT()) {
            feedback.trigger(EVT_MQTT_FAILED);
            waitForFeedback();
            return;
        }
        feedback.trigger(EVT_CONNECTED_READY);
        waitForFeedback();
        enterScanningMode();
    }
}

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
        bool ok = publishScan(tag);
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

void loop() {
    ensureConnections();
    mqttClient.loop();
    publishHeartbeatIfDue();

    if (mode == MODE_SCANNING) {
        loopScanning();
    }

    updateFeedbackOutputs();
}
