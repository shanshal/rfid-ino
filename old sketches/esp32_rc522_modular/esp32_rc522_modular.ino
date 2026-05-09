#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <time.h>

#include "config.h"

MFRC522 reader(RFID_SS_PIN, RFID_RST_PIN);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

String deviceMac;
String scanTopic;
String statusTopic;
String lwtPayload;

unsigned long lastHeartbeatMs = 0;
unsigned long buzzerOffAtMs = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  initBuzzer();
  initRFID();

  connectWiFi();
  initDeviceIdentity();
  syncTime();

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setBufferSize(512);
  connectMQTT();

  Serial.println("ESP32 RC522 modular reader ready.");
}

void loop() {
  ensureConnections();
  mqttClient.loop();

  publishHeartbeatIfDue();
  handleRFIDScan();
  updateBuzzer();
}
