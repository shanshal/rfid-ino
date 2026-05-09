void initDeviceIdentity() {
  deviceMac = WiFi.macAddress();
  deviceMac.toUpperCase();

  scanTopic = String(MQTT_TOPIC_SCAN_PREFIX) + deviceMac + MQTT_TOPIC_SCAN_SUFFIX;
  statusTopic = String(MQTT_TOPIC_SCAN_PREFIX) + deviceMac + MQTT_TOPIC_STATUS_SUFFIX;

  char will[128];
  snprintf(will, sizeof(will), "{\"device_mac\":\"%s\",\"status\":\"offline\"}", deviceMac.c_str());
  lwtPayload = String(will);

  Serial.println("Device MAC: " + deviceMac);
  Serial.println("Scan topic: " + scanTopic);
  Serial.println("Status topic: " + statusTopic);
}

void syncTime() {
  configTime(0, 0, NTP_SERVER);
  Serial.print("Syncing time");
  for (int i = 0; i < 20; i++) {
    time_t now = time(nullptr);
    if (now > 1700000000) {
      Serial.println(" ok");
      return;
    }
    delay(250);
    Serial.print('.');
  }
  Serial.println(" timeout (will fall back to null timestamps)");
}

bool getIsoTimestamp(char* out, size_t out_len) {
  time_t now = time(nullptr);
  if (now < 1700000000) {
    return false;
  }
  struct tm tm_utc;
  gmtime_r(&now, &tm_utc);
  strftime(out, out_len, "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
  return true;
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(WIFI_RETRY_DELAY_MS);
    Serial.print('.');
  }

  Serial.println();
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");

    bool connected = mqttClient.connect(
      deviceMac.c_str(),
      strlen(MQTT_USERNAME) > 0 ? MQTT_USERNAME : nullptr,
      strlen(MQTT_PASSWORD) > 0 ? MQTT_PASSWORD : nullptr,
      statusTopic.c_str(),
      1,
      true,
      lwtPayload.c_str()
    );

    if (connected) {
      Serial.println("connected");
      publishStatus("online");
      lastHeartbeatMs = millis();
      return;
    }

    Serial.print("failed, state=");
    Serial.println(mqttClient.state());
    delay(MQTT_RETRY_DELAY_MS);
  }
}

void ensureConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!mqttClient.connected()) {
    connectMQTT();
  }
}

bool publishScan(const String& uid, uint32_t scan_counter) {
  char ts[32];
  bool have_ts = getIsoTimestamp(ts, sizeof(ts));

  char event_id[80];
  snprintf(event_id, sizeof(event_id), "%s-%lu-%lu",
           deviceMac.c_str(), (unsigned long)millis(), (unsigned long)scan_counter);

  char payload[320];
  if (have_ts) {
    snprintf(payload, sizeof(payload),
      "{\"event_id\":\"%s\",\"device_mac\":\"%s\",\"rfid_uid\":\"%s\",\"scanned_at\":\"%s\"}",
      event_id, deviceMac.c_str(), uid.c_str(), ts);
  } else {
    snprintf(payload, sizeof(payload),
      "{\"event_id\":\"%s\",\"device_mac\":\"%s\",\"rfid_uid\":\"%s\",\"scanned_at\":null}",
      event_id, deviceMac.c_str(), uid.c_str());
  }

  bool ok = mqttClient.publish(scanTopic.c_str(), payload, false);
  Serial.printf("Scan publish %s: %s\n", ok ? "ok" : "failed", payload);
  return ok;
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

  bool ok = mqttClient.publish(statusTopic.c_str(), payload, true);
  Serial.printf("Status publish %s: %s\n", ok ? "ok" : "failed", payload);
  return ok;
}
