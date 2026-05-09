// rdm_debug.ino
// Minimal RDM6300 reader — prints tag UID to Serial monitor.
//
// Wiring: RDM6300 TX → ESP GPIO17
//
// Packet format: 0x02 [10 ASCII hex bytes] [2 checksum bytes] 0x03

#include <HardwareSerial.h>

HardwareSerial RFID(2);  // UART2

bool readTag(char out[11]) {
    if (!RFID.available()) return false;
    if (RFID.read() != 0x02) return false;

    for (int i = 0; i < 10; i++) {
        unsigned long t = millis();
        while (!RFID.available()) {
            if (millis() - t > 200) return false;
        }
        out[i] = RFID.read();
    }
    out[10] = '\0';

    // consume 2 checksum bytes + ETX
    for (int i = 0; i < 3; i++) {
        unsigned long t = millis();
        while (!RFID.available()) {
            if (millis() - t > 200) return false;
        }
        RFID.read();
    }

    return true;
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RDM6300 debug ===");
    RFID.begin(9600, SERIAL_8N1, 17, -1, true);  // RX=17, no TX, inverted for active-low clone
    Serial.println("Waiting for tags...");
}

void loop() {
    char tag[11];
    if (readTag(tag)) {
        Serial.printf("TAG: %s\n", tag);
    }
}
