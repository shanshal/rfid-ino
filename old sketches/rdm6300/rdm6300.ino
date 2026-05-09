#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <time.h>

const char* PROVISION_AP_SSID = "RDM6300-Setup";
const char* PROVISION_AP_PASSWORD = "admin1234";

const char* DEFAULT_MQTT_BROKER = "192.168.10.188";
const uint16_t DEFAULT_MQTT_PORT = 1883;
const char* DEFAULT_MQTT_USERNAME = "scanner";
const char* DEFAULT_MQTT_PASSWORD = "10203040";

const char* MQTT_TOPIC_PREFIX = "scanners/";
const char* MQTT_TOPIC_SCAN_SUFFIX = "/scan";
const char* MQTT_TOPIC_STATUS_SUFFIX = "/status";

const char* FIRMWARE_VERSION = "rdm6300-1.0";
const char* NTP_SERVER = "pool.ntp.org";

const unsigned long HEARTBEAT_INTERVAL_MS = 30000;
const unsigned long WIFI_RETRY_DELAY_MS = 500;
const unsigned long MQTT_RETRY_DELAY_MS = 2000;
const unsigned long CONFIG_PORTAL_TIMEOUT_SECONDS = 600;
const unsigned long WIFI_MANAGER_CONNECT_TIMEOUT_SECONDS = 30;
const int WIFI_CONNECT_ATTEMPTS = 40;
const int MQTT_CONNECT_ATTEMPTS = 3;
const int MAX_CONSECUTIVE_FAILURES = 3;
const uint8_t MANUAL_PORTAL_PIN = 0;
const unsigned long MANUAL_PORTAL_HOLD_MS = 3000;

const uint8_t PIN_SWITCH = 14;
const uint8_t PIN_MOTOR = 26;
const uint8_t PIN_BUZZER = 13;
const uint8_t PIN_LED1 = 32;
const uint8_t PIN_LED2 = 33;
const uint8_t PIN_LED3 = 27;
const uint8_t PIN_BTN1 = 25;

const unsigned long BUTTON_DEBOUNCE_MS = 50;
const unsigned long BEEP_DURATION_MS = 150;
const unsigned long DOUBLE_BEEP_GAP_MS = 100;
const unsigned long MOTOR_DURATION_MS = 200;
const unsigned long SUCCESS_DISPLAY_MS = 600;
const unsigned long FAIL_BLINK_MS = 400;
const unsigned long FAIL_BLINK_TOGGLE_MS = 70;
const unsigned long SCAN_WINDOW_MS = 2000;

HardwareSerial RFID(2); // UART2: RX=16, TX=17
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Preferences preferences;

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
String lwtPayload;

unsigned long lastHeartbeatMs = 0;
uint32_t scan_counter = 0;
int wifiFailureCount = 0;
int mqttFailureCount = 0;

unsigned long buzzerOffAtMs = 0;
unsigned long buzzerSecondOnAtMs = 0;
unsigned long buzzerSecondOffAtMs = 0;
unsigned long motorOffAtMs = 0;
unsigned long successDisplayUntilMs = 0;
unsigned long failBlinkUntilMs = 0;
unsigned long failBlinkToggleAtMs = 0;
bool failBlinkState = false;
bool scanArmed = false;
unsigned long scanWindowUntilMs = 0;

bool lastRawBtn = LOW;
bool debouncedBtn = LOW;
unsigned long lastBtnChangeMs = 0;

bool consumeButtonPress() {
    const bool raw = digitalRead(PIN_BTN1);
    if (raw != lastRawBtn) {
        lastBtnChangeMs = millis();
    }
    lastRawBtn = raw;

    if ((millis() - lastBtnChangeMs) <= BUTTON_DEBOUNCE_MS) {
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
    Serial.println("Feedback: success");
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
    Serial.println("Feedback: publish failed");
}

void initFeedbackHardware() {
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
    delay(100);
}

void updateFeedbackOutputs() {
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

    if (failBlinkUntilMs && (long)(now - failBlinkUntilMs) >= 0) {
        failBlinkUntilMs = 0;
        failBlinkState = false;
    }
    if (failBlinkUntilMs && failBlinkToggleAtMs && (long)(now - failBlinkToggleAtMs) >= 0) {
        failBlinkState = !failBlinkState;
        failBlinkToggleAtMs = now + FAIL_BLINK_TOGGLE_MS;
    }

    if (digitalRead(PIN_SWITCH) == HIGH) {
        setLeds(true, true, true);
        return;
    }

    bool led1 = true;
    bool led2 = scanArmed || (WiFi.status() == WL_CONNECTED);
    bool led3 = mqttClient.connected();

    if (successDisplayUntilMs && (long)(successDisplayUntilMs - now) > 0) {
        led3 = true;
    }
    if (failBlinkUntilMs) {
        led2 = failBlinkState;
    }

    setLeds(led1, led2, led3);
}

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
    Serial.printf("Saved MQTT target: %s:%u\n", runtimeConfig.mqttBroker.c_str(), runtimeConfig.mqttPort);
    Serial.printf("Saved MQTT user: %s\n", runtimeConfig.mqttUsername.c_str());
    return true;
}

bool shouldForcePortalAtBoot() {
    pinMode(MANUAL_PORTAL_PIN, INPUT_PULLUP);

    if (digitalRead(MANUAL_PORTAL_PIN) == HIGH) {
        return false;
    }

    const unsigned long start = millis();
    while (millis() - start < MANUAL_PORTAL_HOLD_MS) {
        if (digitalRead(MANUAL_PORTAL_PIN) == HIGH) {
            return false;
        }
        delay(10);
    }

    Serial.println("Manual portal trigger detected (BOOT held)");
    return true;
}

bool connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return true;
    Serial.print("Connecting WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin();
    for (int i = 0; i < WIFI_CONNECT_ATTEMPTS; i++) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println();
            Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());
            return true;
        }
        delay(WIFI_RETRY_DELAY_MS);
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

    char will[128];
    snprintf(will, sizeof(will), "{\"device_mac\":\"%s\",\"status\":\"offline\"}", deviceMac.c_str());
    lwtPayload = String(will);

    Serial.println("Device MAC: " + deviceMac);
    Serial.println("Scan topic: " + scanTopic);
    Serial.printf("MQTT target: %s:%u\n", runtimeConfig.mqttBroker.c_str(), runtimeConfig.mqttPort);
    Serial.printf("MQTT user: %s\n", runtimeConfig.mqttUsername.c_str());
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

bool publishScan(const char* uid) {
    char ts[32];
    bool have_ts = getIsoTimestamp(ts, sizeof(ts));
    scan_counter++;
    char event_id[80];
    snprintf(event_id, sizeof(event_id), "%s-%lu-%lu",
             deviceMac.c_str(), (unsigned long)millis(), (unsigned long)scan_counter);
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
    Serial.printf("Scan publish %s: %s\n", ok ? "ok" : "failed", payload);
    return ok;
}

bool connectMQTT() {
    if (mqttClient.connected()) {
        return true;
    }

    for (int attempt = 0; attempt < MQTT_CONNECT_ATTEMPTS; attempt++) {
        Serial.print("Connecting MQTT...");
        bool connected = mqttClient.connect(
            deviceMac.c_str(),
            runtimeConfig.mqttUsername.length() > 0 ? runtimeConfig.mqttUsername.c_str() : nullptr,
            runtimeConfig.mqttPassword.length() > 0 ? runtimeConfig.mqttPassword.c_str() : nullptr,
            statusTopic.c_str(),
            1,
            true,
            lwtPayload.c_str()
        );
        if (connected) {
            Serial.println("connected");
            publishStatus("online");
            lastHeartbeatMs = millis();
            return true;
        }
        Serial.printf(
            "failed, state=%d (%s), host=%s, port=%u, user=%s\n",
            mqttClient.state(),
            mqttStateText(mqttClient.state()),
            runtimeConfig.mqttBroker.c_str(),
            runtimeConfig.mqttPort,
            runtimeConfig.mqttUsername.c_str()
        );
        delay(MQTT_RETRY_DELAY_MS);
    }

    return false;
}

void ensureConnections() {
    if (WiFi.status() != WL_CONNECTED) {
        if (!connectWiFi()) {
            wifiFailureCount++;
            if (wifiFailureCount >= MAX_CONSECUTIVE_FAILURES) {
                wifiFailureCount = 0;
                runProvisioningPortal(true);
                initDeviceIdentity();
                mqttClient.setServer(runtimeConfig.mqttBroker.c_str(), runtimeConfig.mqttPort);
            }
            return;
        }
        wifiFailureCount = 0;
    }

    if (!mqttClient.connected()) {
        if (!connectMQTT()) {
            mqttFailureCount++;
            if (mqttFailureCount >= MAX_CONSECUTIVE_FAILURES) {
                mqttFailureCount = 0;
                runProvisioningPortal(true);
                initDeviceIdentity();
                mqttClient.setServer(runtimeConfig.mqttBroker.c_str(), runtimeConfig.mqttPort);
            }
            return;
        }
        mqttFailureCount = 0;
    }
}

void publishHeartbeatIfDue() {
    unsigned long now = millis();
    if (now - lastHeartbeatMs < HEARTBEAT_INTERVAL_MS) return;
    lastHeartbeatMs = now;
    publishStatus("alive");
}

void setup() {
    Serial.begin(115200);
    RFID.begin(9600, SERIAL_8N1, 16, 17);
    initFeedbackHardware();
    pinMode(PIN_BTN1, INPUT);

    const bool forcePortalAtBoot = shouldForcePortalAtBoot();

    loadRuntimeConfig();
    if (!hasValidRuntimeConfig()) {
        runtimeConfig.mqttBroker = DEFAULT_MQTT_BROKER;
        runtimeConfig.mqttPort = DEFAULT_MQTT_PORT;
        runtimeConfig.mqttUsername = DEFAULT_MQTT_USERNAME;
        runtimeConfig.mqttPassword = DEFAULT_MQTT_PASSWORD;
    }

    if (!runProvisioningPortal(forcePortalAtBoot || !hasValidRuntimeConfig())) {
        Serial.println("Provisioning failed, restarting in 3 seconds");
        delay(3000);
        ESP.restart();
    }

    initDeviceIdentity();
    syncTime();

    mqttClient.setServer(runtimeConfig.mqttBroker.c_str(), runtimeConfig.mqttPort);
    mqttClient.setBufferSize(512);
    connectMQTT();

    Serial.println("RDM6300 reader ready.");
}

void loop() {
    ensureConnections();
    mqttClient.loop();
    publishHeartbeatIfDue();
    updateFeedbackOutputs();

    if (consumeButtonPress()) {
        while (RFID.available()) RFID.read();
        scanArmed = true;
        scanWindowUntilMs = millis() + SCAN_WINDOW_MS;
        Serial.println("Scan armed by button press");
    }

    if (scanArmed && (long)(millis() - scanWindowUntilMs) >= 0) {
        scanArmed = false;
        triggerFailFeedback();
        Serial.println("Scan window timeout");
    }

    if (!scanArmed) {
        return;
    }

    if (RFID.available()) {
        if (RFID.read() == 0x02) {
            char tag[11];
            for (int i = 0; i < 10; i++) {
                while (!RFID.available());
                tag[i] = RFID.read();
            }
            tag[10] = '\0';

            for (int i = 0; i < 2; i++) {
                while (!RFID.available());
                RFID.read(); // skip checksum bytes
            }

            while (!RFID.available());
            if (RFID.read() == 0x03) {
                Serial.print("Tag: "); Serial.println(tag);
                if (publishScan(tag)) {
                    scanArmed = false;
                    triggerSuccessFeedback();
                } else {
                    triggerFailFeedback();
                }
            }
        }
    }
}
