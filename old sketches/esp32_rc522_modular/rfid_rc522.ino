void initBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_HIGH ? LOW : HIGH);
}

void setBuzzer(bool enabled) {
  digitalWrite(BUZZER_PIN, enabled == BUZZER_ACTIVE_HIGH ? HIGH : LOW);
}

void beep() {
  setBuzzer(true);
  buzzerOffAtMs = millis() + BEEP_DURATION_MS;
}

void updateBuzzer() {
  if (buzzerOffAtMs == 0) {
    return;
  }

  if ((long)(millis() - buzzerOffAtMs) >= 0) {
    setBuzzer(false);
    buzzerOffAtMs = 0;
  }
}

void initRFID() {
  SPI.begin(RFID_SCK_PIN, RFID_MISO_PIN, RFID_MOSI_PIN, RFID_SS_PIN);
  reader.PCD_Init();

  byte version = reader.PCD_ReadRegister(reader.VersionReg);
  Serial.printf("MFRC522 version: 0x%02X\n", version);
}

String readCurrentUid() {
  String uid;
  uid.reserve(reader.uid.size * 2);

  for (byte i = 0; i < reader.uid.size; i++) {
    if (reader.uid.uidByte[i] < 0x10) {
      uid += '0';
    }
    uid += String(reader.uid.uidByte[i], HEX);
  }

  uid.toUpperCase();
  return uid;
}

void handleRFIDScan() {
  static uint32_t scan_counter = 0;

  if (!reader.PICC_IsNewCardPresent()) {
    return;
  }

  if (!reader.PICC_ReadCardSerial()) {
    Serial.println("Reader: failed to read card serial");
    return;
  }

  const String uid = readCurrentUid();
  Serial.println("Reader UID: " + uid);

  scan_counter++;
  if (publishScan(uid, scan_counter)) {
    beep();
  }

  reader.PICC_HaltA();
  reader.PCD_StopCrypto1();
}
