#include <dummy.h>

// scanner.ino
// Energy-optimized RDM6300 RFID scanner — button-triggered, WiFi modem sleep,
// no heartbeat. Broker learns device state via LWT only.
//
// Required libraries:
//   - PubSubClient (knolleary)
//
// Pin map:
//   RDM6300 TX→ESP RX=34 | LED1=32, LED2=33, LED3=27 | BUZZER=13 | BTN=25 (3.3V→GPIO, INPUT_PULLDOWN)

#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <time.h>
#include <stdarg.h>

enum State         : int;
enum FeedbackEvent : int;
struct FeedbackFrame;
struct FeedbackPattern;
struct FeedbackEngine;

// ---- Configuration ----

static const char*    WIFI_SSID     = "Ameer ftth";
static const char*    WIFI_PASSWORD = "ameer2017";
static const char*    MQTT_BROKER   = "159.195.41.12";
static const uint16_t MQTT_PORT     = 1883;
static const char*    MQTT_USERNAME = "scanner";
static const char*    MQTT_PASSWORD = "10203040";

static const char* TOPIC_PREFIX   = "scanners/";
static const char* TOPIC_SCAN     = "/scan";
static const char* TOPIC_STATUS   = "/status";
static const char* TOPIC_ANNOUNCE = "/announce";
static const char* FIRMWARE       = "scanner-3.0";
static const char* NTP_SERVER     = "pool.ntp.org";

static const int           WIFI_ATTEMPTS = 40;
static const int           MQTT_ATTEMPTS = 3;
static const unsigned long WIFI_WAIT_MS  = 500;
static const unsigned long MQTT_WAIT_MS  = 2000;
static const unsigned long SCAN_WINDOW   = 5000;  // DEBUG: extended for testing
static const unsigned long HOLD_MS       = 300;

// ---- Pins ----

static const uint8_t PIN_RX     = 16;  // receives tag data from RDM6300 TX
static const uint8_t PIN_LED1   = 32;
static const uint8_t PIN_LED2   = 33;
static const uint8_t PIN_LED3   = 27;
static const uint8_t PIN_BUZZER = 13;
static const uint8_t PIN_BTN    = 25;

// ---- State ----

enum State : int { ST_CONNECTING, ST_IDLE, ST_SCANNING, ST_PUBLISHING };

static State s = ST_CONNECTING;

// ---- Feedback engine ----
//
// Events map 1:1 onto PATS[] entries (evt - 1 is the index).
// ST_IDLE/SCANNING/PUBLISHING LED baselines are driven separately in updateFB().

enum FeedbackEvent : int {
    EVT_NONE = 0,
    EVT_WIFI_CONNECTING,   // 1
    EVT_WIFI_FAILED,       // 2
    EVT_MQTT_FAILED,       // 3
    EVT_CONNECTED_READY,   // 4
    EVT_SCAN_ACK,          // 5
    EVT_SCAN_FOUND,        // 6
    EVT_SCAN_TIMEOUT,      // 7
    EVT_PUBLISH_SUCCESS,   // 8
    EVT_PUBLISH_FAIL,      // 9
};

struct FeedbackFrame   { unsigned long ms; bool l1, l2, l3, bz; };
struct FeedbackPattern { const FeedbackFrame* fr; uint8_t n; bool loop; };

static const FeedbackFrame FR_WIFI_CONN[] = {
    { 200, 1,0,0,0 }, { 200, 0,1,0,0 }, { 200, 0,0,1,0 }, { 100, 0,0,0,0 },
};
static const FeedbackFrame FR_WIFI_FAIL[] = {
    { 200, 1,0,0,1 }, { 100, 0,0,0,0 },
    { 200, 1,0,0,1 }, { 100, 0,0,0,0 },
    { 250, 1,0,0,0 }, { 200, 0,0,0,0 },
    { 250, 1,0,0,0 }, { 200, 0,0,0,0 },
};
static const FeedbackFrame FR_MQTT_FAIL[] = {
    { 250, 1,1,0,0 }, { 200, 0,0,0,0 },
    { 250, 1,1,0,0 }, { 200, 0,0,0,0 },
    { 250, 1,1,0,0 }, { 200, 0,0,0,0 },
    { 250, 1,1,0,0 }, { 200, 0,0,0,0 },
};
static const FeedbackFrame FR_CONNECTED[] = {
    {  60, 1,1,1,1 }, {  60, 1,1,1,0 }, {  80, 0,0,0,0 },
    { 120, 1,1,1,0 }, { 120, 0,0,0,0 },
    { 120, 1,1,1,0 }, { 120, 0,0,0,0 },
};
static const FeedbackFrame FR_SCAN_ACK[] = {
    { 60, 0,0,0,1 }, { 40, 0,0,0,0 },
};
static const FeedbackFrame FR_SCAN_FOUND[] = {
    {  60, 0,1,0,1 }, { 100, 0,1,0,0 }, { 80, 0,0,0,0 },
};
static const FeedbackFrame FR_SCAN_TIMEOUT[] = {
    {  80, 0,0,1,1 }, {  80, 0,0,1,0 }, {  80, 0,0,0,0 },
    {  80, 0,0,1,1 }, {  80, 0,0,1,0 }, { 100, 0,0,0,0 },
};
static const FeedbackFrame FR_PUB_OK[] = {
    {  80, 0,0,1,1 }, {  80, 0,0,1,0 }, {  80, 0,0,0,0 },
    {  80, 0,0,1,1 }, {  80, 0,0,1,0 }, { 100, 0,0,1,0 },
};
static const FeedbackFrame FR_PUB_FAIL[] = {
    { 200, 1,1,1,1 }, { 100, 0,0,0,0 },
    { 200, 1,1,1,1 }, { 100, 0,0,0,0 },
    { 200, 1,1,1,0 }, { 100, 0,0,0,0 },
};

#define PAT(fr, loop) { fr, (uint8_t)(sizeof(fr)/sizeof(FeedbackFrame)), loop }

static const FeedbackPattern PATS[] = {
    PAT(FR_WIFI_CONN,    true ),  // EVT_WIFI_CONNECTING
    PAT(FR_WIFI_FAIL,    false),  // EVT_WIFI_FAILED
    PAT(FR_MQTT_FAIL,    false),  // EVT_MQTT_FAILED
    PAT(FR_CONNECTED,    false),  // EVT_CONNECTED_READY
    PAT(FR_SCAN_ACK,     false),  // EVT_SCAN_ACK
    PAT(FR_SCAN_FOUND,   false),  // EVT_SCAN_FOUND
    PAT(FR_SCAN_TIMEOUT, false),  // EVT_SCAN_TIMEOUT
    PAT(FR_PUB_OK,       false),  // EVT_PUBLISH_SUCCESS
    PAT(FR_PUB_FAIL,     false),  // EVT_PUBLISH_FAIL
};

struct FeedbackEngine {
    FeedbackEvent evt   = EVT_NONE;
    uint8_t       frame = 0;
    unsigned long fms   = 0;
    bool          done  = true;

    void trigger(FeedbackEvent e) {
        evt = e; frame = 0; fms = millis();
        done = (e == EVT_NONE);
    }
    void clear()                  { evt = EVT_NONE; done = true; }
    bool isFinished()       const { return done; }
    FeedbackEvent active()  const { return evt; }

    bool tick(bool& l1, bool& l2, bool& l3, bool& bz) {
        if (done || evt == EVT_NONE) return false;
        const FeedbackPattern& p = PATS[evt - 1];
        unsigned long now = millis();
        if ((long)(now - (fms + p.fr[frame].ms)) >= 0) {
            fms = now;
            if (++frame >= p.n) {
                if (p.loop) frame = 0;
                else { done = true; return false; }
            }
        }
        l1 = p.fr[frame].l1; l2 = p.fr[frame].l2;
        l3 = p.fr[frame].l3; bz = p.fr[frame].bz;
        return true;
    }
};

// ---- Globals ----

HardwareSerial RFID(2);
WiFiClient     wifiClient;
PubSubClient   mqttClient(wifiClient);
FeedbackEngine fb;

String   deviceMac, scanTopic, statusTopic, announceTopic, lwtPayload;
uint32_t scanCounter = 0;
unsigned long scanUntil = 0;

bool          scanRequested  = false;
unsigned long holdStart      = 0;
bool          holdTriggered  = false;  // true until button released; prevents re-fire on continuous hold
unsigned long blinkMs        = 0;
bool          blinkOn        = false;

// ---- Logging ----

void dbgf(const char* fmt, ...) {
    char buf[200]; va_list a;
    va_start(a, fmt); vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
    Serial.printf("[%lu] %s\n", millis(), buf);
}

// ---- Output helpers ----

void leds(bool l1, bool l2, bool l3) {
    digitalWrite(PIN_LED1, l1); digitalWrite(PIN_LED2, l2); digitalWrite(PIN_LED3, l3);
}
void allOff() { leds(0,0,0); digitalWrite(PIN_BUZZER, LOW); }

// ---- Feedback update ----

void updateFB() {
    bool l1=0, l2=0, l3=0, bz=0;
    const bool active = fb.tick(l1, l2, l3, bz);
    digitalWrite(PIN_BUZZER, bz);
    if (active) { leds(l1, l2, l3); return; }

    // State-based LED baseline when no animation is playing
    unsigned long now = millis();
    switch (s) {
        case ST_IDLE:
            if (WiFi.status() == WL_CONNECTED) {
                leds(1, 0, 1);  // LED1 + LED3 solid = connected, ready
            } else {
                // LED1 fast blink = WiFi lost alert
                if (now - blinkMs >= 250) { blinkOn = !blinkOn; blinkMs = now; }
                leds(blinkOn, 0, 0);
            }
            break;
        case ST_SCANNING:
            // LED2 solid + LED3 slow blink = listening for tag
            if (now - blinkMs >= 500) { blinkOn = !blinkOn; blinkMs = now; }
            leds(0, 1, blinkOn);
            break;
        case ST_PUBLISHING:
            // LED3 fast blink = waking modem + publishing
            if (now - blinkMs >= 150) { blinkOn = !blinkOn; blinkMs = now; }
            leds(0, 0, blinkOn);
            break;
        default:
            leds(0, 0, 0);
    }
}

void waitFB() { while (!fb.isFinished()) { updateFB(); delay(5); } }

// ---- WiFi / MQTT ----

void initTopics() {
    deviceMac     = WiFi.macAddress(); deviceMac.toUpperCase();
    scanTopic     = String(TOPIC_PREFIX) + deviceMac + TOPIC_SCAN;
    statusTopic   = String(TOPIC_PREFIX) + deviceMac + TOPIC_STATUS;
    announceTopic = String(TOPIC_PREFIX) + deviceMac + TOPIC_ANNOUNCE;
    char will[128];
    snprintf(will, sizeof(will), "{\"device_mac\":\"%s\",\"status\":\"offline\"}", deviceMac.c_str());
    lwtPayload = will;
    dbgf("mac=%s scan=%s", deviceMac.c_str(), scanTopic.c_str());
}

void syncTime() {
    configTime(0, 0, NTP_SERVER);
    Serial.print("NTP");
    for (int i = 0; i < 20; i++) {
        if (time(nullptr) > 1700000000) { Serial.println(" ok"); return; }
        delay(250); Serial.print('.');
    }
    Serial.println(" timeout");
}

bool getTs(char* out, size_t n) {
    time_t t = time(nullptr);
    if (t < 1700000000) return false;
    struct tm tm; gmtime_r(&t, &tm);
    strftime(out, n, "%Y-%m-%dT%H:%M:%SZ", &tm);
    return true;
}

bool connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return true;
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("WiFi");
    for (int i = 0; i < WIFI_ATTEMPTS; i++) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println(); dbgf("IP=%s", WiFi.localIP().toString().c_str());
            return true;
        }
        unsigned long t = millis() + WIFI_WAIT_MS;
        while ((long)(millis()-t) < 0) { updateFB(); delay(10); }
        Serial.print('.');
    }
    Serial.println(" fail"); return false;
}

bool pubStatus(const char* st) {
    char ts[32], p[256]; bool hts = getTs(ts, sizeof(ts));
    snprintf(p, sizeof(p),
        "{\"device_mac\":\"%s\",\"status\":\"%s\",\"uptime_ms\":%lu,\"rssi\":%d,\"firmware\":\"%s\",\"at\":%s%s%s}",
        deviceMac.c_str(), st, (unsigned long)millis(), WiFi.RSSI(), FIRMWARE,
        hts?"\"":"", hts?ts:"null", hts?"\"":"");
    return mqttClient.publish(statusTopic.c_str(), p, true);
}

bool pubAnnounce() {
    char ts[32], p[512]; bool hts = getTs(ts, sizeof(ts));
    snprintf(p, sizeof(p),
        "{\"device_mac\":\"%s\",\"local_ip\":\"%s\",\"scan_topic\":\"%s\","
        "\"status_topic\":\"%s\",\"mqtt_host\":\"%s\",\"mqtt_port\":%u,\"firmware\":\"%s\",\"at\":%s%s%s}",
        deviceMac.c_str(), WiFi.localIP().toString().c_str(),
        scanTopic.c_str(), statusTopic.c_str(),
        MQTT_BROKER, MQTT_PORT, FIRMWARE,
        hts?"\"":"", hts?ts:"null", hts?"\"":"");
    bool ok = mqttClient.publish(announceTopic.c_str(), p, true);
    dbgf("announce %s", ok?"ok":"FAIL"); return ok;
}

bool pubScan(const char* uid) {
    char ts[32], eid[80], p[320]; bool hts = getTs(ts, sizeof(ts));
    scanCounter++;
    snprintf(eid, sizeof(eid), "%s-%lu-%lu",
             deviceMac.c_str(), (unsigned long)millis(), (unsigned long)scanCounter);
    snprintf(p, sizeof(p),
        "{\"event_id\":\"%s\",\"device_mac\":\"%s\",\"rfid_uid\":\"%s\",\"scanned_at\":%s%s%s}",
        eid, deviceMac.c_str(), uid,
        hts?"\"":"", hts?ts:"null", hts?"\"":"");
    bool ok = mqttClient.publish(scanTopic.c_str(), p, false);
    dbgf("scan %s uid=%s", ok?"ok":"FAIL", uid); return ok;
}

bool connectMQTT() {
    if (mqttClient.connected()) return true;
    for (int i = 0; i < MQTT_ATTEMPTS; i++) {
        Serial.print("MQTT..");
        if (mqttClient.connect(deviceMac.c_str(), MQTT_USERNAME, MQTT_PASSWORD,
                               statusTopic.c_str(), 1, true, lwtPayload.c_str())) {
            Serial.println("ok");
            pubStatus("online");
            pubAnnounce();
            return true;
        }
        dbgf("MQTT fail state=%d", mqttClient.state());
        unsigned long t = millis() + MQTT_WAIT_MS;
        while ((long)(millis()-t) < 0) { updateFB(); delay(10); }
    }
    return false;
}

// ---- RFID reading ----

bool tryReadTag(char out[11]) {
    if (!RFID.available()) return false;
    if (RFID.read() != 0x02) return false;
    for (int i = 0; i < 10; i++) {
        unsigned long t = millis();
        while (!RFID.available()) { if (millis()-t > 200) return false; }
        out[i] = RFID.read();
    }
    out[10] = '\0';
    for (int i = 0; i < 2; i++) {
        unsigned long t = millis();
        while (!RFID.available()) { if (millis()-t > 200) return false; }
        RFID.read();
    }
    unsigned long t = millis();
    while (!RFID.available()) { if (millis()-t > 200) return false; }
    return RFID.read() == 0x03;
}

// ---- State transitions ----

void enterIdle() {
    s = ST_IDLE;
    setCpuFrequencyMhz(80);
    WiFi.setSleep(true);
    fb.clear();
    scanRequested = false;
    holdStart = 0;  // holdTriggered intentionally not reset here — clears only on button release
    blinkOn = false; blinkMs = millis();
    dbgf("→ IDLE (CPU 80MHz, modem sleep)");
}

// ---- Setup / loop ----

void setup() {
    Serial.begin(115200); delay(50);
    Serial.println("\n=== scanner-3.0 ===");

    btStop();                  // ~40mA saving
    setCpuFrequencyMhz(80);

    pinMode(PIN_LED1, OUTPUT); pinMode(PIN_LED2, OUTPUT); pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_BTN, INPUT_PULLDOWN);
    allOff();

    // LED self-test chase
    leds(1,0,0); delay(120);
    leds(0,1,0); delay(120);
    leds(0,0,1); delay(120);
    leds(0,0,0);

    RFID.begin(9600, SERIAL_8N1, PIN_RX, -1);  // standard TTL UART, idles HIGH
    WiFi.mode(WIFI_STA);

    fb.trigger(EVT_WIFI_CONNECTING);
    if (!connectWiFi()) {
        fb.trigger(EVT_WIFI_FAILED); waitFB(); delay(1500); ESP.restart();
    }

    initTopics();
    syncTime();
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setBufferSize(512);

    fb.trigger(EVT_WIFI_CONNECTING);
    if (!connectMQTT()) {
        fb.trigger(EVT_MQTT_FAILED); waitFB();
        return;  // Stay ST_CONNECTING; loopConnecting() will retry
    }

    fb.trigger(EVT_CONNECTED_READY); waitFB();
    enterIdle();
    Serial.println("Ready.");
}

// Called from loop() when MQTT failed at boot — keeps retrying
void loopConnecting() {
    if (!fb.isFinished()) return;
    fb.trigger(EVT_WIFI_CONNECTING);
    if (WiFi.status() != WL_CONNECTED && !connectWiFi()) {
        fb.trigger(EVT_WIFI_FAILED); waitFB(); return;
    }
    if (!connectMQTT()) {
        fb.trigger(EVT_MQTT_FAILED); waitFB(); return;
    }
    fb.trigger(EVT_CONNECTED_READY); waitFB();
    enterIdle();
}

void loopIdle() {
    WiFi.setSleep(false);
    mqttClient.loop();
    WiFi.setSleep(true);

    bool btn = digitalRead(PIN_BTN) == HIGH;
    unsigned long now = millis();
    if (btn) {
        if (holdStart == 0) holdStart = now;
        if (!holdTriggered && (now - holdStart) >= HOLD_MS) {
            scanRequested = true;
            holdTriggered = true;
        }
    } else {
        holdStart = 0;
        holdTriggered = false;
    }

    if (!scanRequested) return;
    scanRequested = false;
    s = ST_SCANNING;
    setCpuFrequencyMhz(240);  // full speed for tight UART byte timing
    while (RFID.available()) RFID.read();
    blinkOn = false; blinkMs = millis();
    scanUntil = millis() + SCAN_WINDOW;
    fb.trigger(EVT_SCAN_ACK);
    dbgf("→ SCANNING");
}

void loopScanning() {
    char tag[11];
    if (tryReadTag(tag)) {
        dbgf("tag=%s", tag);
        fb.trigger(EVT_SCAN_FOUND); waitFB();

        s = ST_PUBLISHING;
        WiFi.setSleep(false);
        delay(50);  // give modem a moment to wake
        blinkOn = false; blinkMs = millis();
        dbgf("→ PUBLISHING");

        // Reconnect MQTT if it timed out during idle sleep
        if (!mqttClient.connected()) {
            if (WiFi.status() != WL_CONNECTED) connectWiFi();
            connectMQTT();
        }

        bool ok = false;
        if (mqttClient.connected()) {
            mqttClient.loop();
            ok = pubScan(tag);
        }

        fb.trigger(ok ? EVT_PUBLISH_SUCCESS : EVT_PUBLISH_FAIL); waitFB();
        enterIdle();
        return;
    }

    if ((long)(millis() - scanUntil) >= 0) {
        dbgf("scan timeout");
        fb.trigger(EVT_SCAN_TIMEOUT); waitFB();
        enterIdle();
    }
}

void loop() {
    switch (s) {
        case ST_CONNECTING: loopConnecting(); break;
        case ST_IDLE:       loopIdle();       break;
        case ST_SCANNING:   loopScanning();   break;
        case ST_PUBLISHING: break;  // handled inline inside loopScanning
    }
    updateFB();
}
