#include <Arduino.h>

// XIAO ESP32S3 sender test
static const int UART_TX_PIN = 43;
static const int UART_RX_PIN = -1;
static const uint32_t UART_BAUD = 115200;

static const uint8_t SOF1 = 0xA5;
static const uint8_t SOF2 = 0x5A;
static const uint8_t EOF1 = 0x5A;
static const uint8_t EOF2 = 0xA5;
static const uint16_t PAYLOAD_LEN = 64;

static uint8_t crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

static void sendPacket(uint32_t seq) {
  uint8_t payload[PAYLOAD_LEN];
  for (uint16_t i = 0; i < PAYLOAD_LEN; i++) {
    payload[i] = (uint8_t)((seq + i) & 0xFF);
  }

  uint8_t crc = 0;
  crc = crc8_update(crc, (uint8_t)(seq & 0xFF));
  crc = crc8_update(crc, (uint8_t)((seq >> 8) & 0xFF));
  crc = crc8_update(crc, (uint8_t)((seq >> 16) & 0xFF));
  crc = crc8_update(crc, (uint8_t)((seq >> 24) & 0xFF));
  crc = crc8_update(crc, (uint8_t)(PAYLOAD_LEN & 0xFF));
  crc = crc8_update(crc, (uint8_t)((PAYLOAD_LEN >> 8) & 0xFF));
  for (uint16_t i = 0; i < PAYLOAD_LEN; i++) crc = crc8_update(crc, payload[i]);

  Serial1.write(SOF1);
  Serial1.write(SOF2);

  uint8_t seqBytes[4] = {
    (uint8_t)(seq & 0xFF),
    (uint8_t)((seq >> 8) & 0xFF),
    (uint8_t)((seq >> 16) & 0xFF),
    (uint8_t)((seq >> 24) & 0xFF)
  };
  Serial1.write(seqBytes, sizeof(seqBytes));

  uint8_t lenBytes[2] = {
    (uint8_t)(PAYLOAD_LEN & 0xFF),
    (uint8_t)((PAYLOAD_LEN >> 8) & 0xFF)
  };
  Serial1.write(lenBytes, sizeof(lenBytes));

  Serial1.write(payload, PAYLOAD_LEN);
  Serial1.write(crc);
  Serial1.write(EOF1);
  Serial1.write(EOF2);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART sender test started");
}

void loop() {
  static uint32_t seq = 0;
  static uint32_t lastLog = 0;

  sendPacket(seq++);
  delay(20); // ~50 packets/sec

  uint32_t now = millis();
  if (now - lastLog >= 1000) {
    lastLog = now;
    Serial.printf("sent=%lu baud=%lu\n", (unsigned long)seq, (unsigned long)UART_BAUD);
  }
}
