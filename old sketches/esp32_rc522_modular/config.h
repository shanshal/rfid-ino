#pragma once

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* MQTT_BROKER = "192.168.0.109";
const int MQTT_PORT = 1883;
const char* MQTT_USERNAME = "scanner";
const char* MQTT_PASSWORD = "";

const char* MQTT_TOPIC_SCAN_PREFIX = "scanners/";
const char* MQTT_TOPIC_SCAN_SUFFIX = "/scan";
const char* MQTT_TOPIC_STATUS_SUFFIX = "/status";

const char* FIRMWARE_VERSION = "rc522-modular-1.0";
const char* NTP_SERVER = "pool.ntp.org";

// RC522 wiring for ESP32
const uint8_t RFID_RST_PIN = 22;
const uint8_t RFID_SS_PIN = 5;
const uint8_t RFID_SCK_PIN = 18;
const uint8_t RFID_MISO_PIN = 19;
const uint8_t RFID_MOSI_PIN = 23;

const uint8_t BUZZER_PIN = 26;
const bool BUZZER_ACTIVE_HIGH = true;

const unsigned long HEARTBEAT_INTERVAL_MS = 30000;
const unsigned long WIFI_RETRY_DELAY_MS = 500;
const unsigned long MQTT_RETRY_DELAY_MS = 2000;
const unsigned long BEEP_DURATION_MS = 120;
