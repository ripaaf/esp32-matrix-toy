#include <Arduino.h>

// Matrix ESP32S3 receiver test (set to your RX pin)
static const int UART_RX_PIN = 44;
static const int UART_TX_PIN = -1;
static const uint32_t UART_BAUD = 115200;

static const uint8_t SOF1 = 0xA5;
static const uint8_t SOF2 = 0x5A;
static const uint8_t EOF1 = 0x5A;
static const uint8_t EOF2 = 0xA5;

static uint8_t crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

static bool readBytesUart(uint8_t *dst, size_t len, uint32_t timeoutMs) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < len && (millis() - start) < timeoutMs) {
    int avail = Serial1.available();
    if (avail > 0) {
      int toRead = (int)min((size_t)avail, len - got);
      int n = Serial1.readBytes((char *)(dst + got), toRead);
      if (n > 0) got += (size_t)n;
    } else {
      delay(1);
    }
  }
  return got == len;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART receiver test started");
}

void loop() {
  static uint32_t okCount = 0;
  static uint32_t crcErr = 0;
  static uint32_t framingErr = 0;
  static uint32_t timeoutErr = 0;
  static uint32_t lostSeq = 0;
  static uint32_t expectedSeq = 0;
  static bool firstPacket = true;
  static uint32_t lastStat = 0;

  // Find SOF
  bool sofFound = false;
  uint32_t sofStart = millis();
  uint8_t prev = 0;
  while ((millis() - sofStart) < 200) {
    if (Serial1.available() > 0) {
      uint8_t b = (uint8_t)Serial1.read();
      if (prev == SOF1 && b == SOF2) {
        sofFound = true;
        break;
      }
      prev = b;
    } else {
      delay(1);
    }
  }

  if (!sofFound) {
    timeoutErr++;
  } else {
    uint8_t seqBytes[4];
    uint8_t lenBytes[2];

    if (!readBytesUart(seqBytes, sizeof(seqBytes), 80) || !readBytesUart(lenBytes, sizeof(lenBytes), 80)) {
      timeoutErr++;
    } else {
      uint32_t seq =
          (uint32_t)seqBytes[0] |
          ((uint32_t)seqBytes[1] << 8) |
          ((uint32_t)seqBytes[2] << 16) |
          ((uint32_t)seqBytes[3] << 24);

      uint16_t len = (uint16_t)lenBytes[0] | (uint16_t)((uint16_t)lenBytes[1] << 8);
      if (len == 0 || len > 256) {
        framingErr++;
      } else {
        uint8_t payload[256];
        uint8_t rxCrc = 0;
        uint8_t eof[2] = {0};

        if (!readBytesUart(payload, len, 100) || !readBytesUart(&rxCrc, 1, 40) || !readBytesUart(eof, 2, 40)) {
          timeoutErr++;
        } else if (eof[0] != EOF1 || eof[1] != EOF2) {
          framingErr++;
        } else {
          uint8_t calc = 0;
          calc = crc8_update(calc, seqBytes[0]);
          calc = crc8_update(calc, seqBytes[1]);
          calc = crc8_update(calc, seqBytes[2]);
          calc = crc8_update(calc, seqBytes[3]);
          calc = crc8_update(calc, lenBytes[0]);
          calc = crc8_update(calc, lenBytes[1]);
          for (uint16_t i = 0; i < len; i++) calc = crc8_update(calc, payload[i]);

          if (calc != rxCrc) {
            crcErr++;
          } else {
            if (!firstPacket) {
              if (seq > expectedSeq) lostSeq += (seq - expectedSeq);
            }
            firstPacket = false;
            expectedSeq = seq + 1;
            okCount++;
          }
        }
      }
    }
  }

  uint32_t now = millis();
  if (now - lastStat >= 1000) {
    lastStat = now;
    Serial.printf("ok=%lu lost=%lu crc=%lu frame=%lu timeout=%lu baud=%lu rxPin=%d\n",
                  (unsigned long)okCount,
                  (unsigned long)lostSeq,
                  (unsigned long)crcErr,
                  (unsigned long)framingErr,
                  (unsigned long)timeoutErr,
                  (unsigned long)UART_BAUD,
                  UART_RX_PIN);
  }
}
