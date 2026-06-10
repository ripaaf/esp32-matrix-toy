#include <Arduino.h>
#include <FS.h>
#include <SPI.h>
#include <SD.h>
#include "ESP_I2S.h"

// XIAO ESP32S3 Sense mic + SD wiring used in this project.
static const int MIC_DATA_PIN = 41;
static const int MIC_CLK_PIN = 42;
static const int SD_CS_PIN = 21;
static const int SD_SCK_PIN = 7;
static const int SD_MISO_PIN = 8;
static const int SD_MOSI_PIN = 9;

static const uint32_t SAMPLE_RATE = 16000;     // Try 8000 here if needed.
static const uint16_t BITS_PER_SAMPLE = 16;
static const uint16_t CHANNELS = 1;
static const uint32_t RECORD_SECONDS = 20;
static const size_t READ_CHUNK_BYTES = 1024;

static SPIClass sdSpi(FSPI);
static I2SClass micI2S;

static void writeLE16(File &f, uint16_t v) {
  uint8_t b[2] = {(uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF)};
  f.write(b, sizeof(b));
}

static void writeLE32(File &f, uint32_t v) {
  uint8_t b[4] = {
    (uint8_t)(v & 0xFF),
    (uint8_t)((v >> 8) & 0xFF),
    (uint8_t)((v >> 16) & 0xFF),
    (uint8_t)((v >> 24) & 0xFF)
  };
  f.write(b, sizeof(b));
}

static void writeWavHeader(File &f, uint32_t dataBytes) {
  const uint32_t byteRate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8);
  const uint16_t blockAlign = CHANNELS * (BITS_PER_SAMPLE / 8);
  const uint32_t riffSize = 36 + dataBytes;

  f.seek(0);
  f.write((const uint8_t *)"RIFF", 4);
  writeLE32(f, riffSize);
  f.write((const uint8_t *)"WAVE", 4);

  f.write((const uint8_t *)"fmt ", 4);
  writeLE32(f, 16);
  writeLE16(f, 1);
  writeLE16(f, CHANNELS);
  writeLE32(f, SAMPLE_RATE);
  writeLE32(f, byteRate);
  writeLE16(f, blockAlign);
  writeLE16(f, BITS_PER_SAMPLE);

  f.write((const uint8_t *)"data", 4);
  writeLE32(f, dataBytes);
}

static bool initMic() {
  micI2S.end();
  micI2S.setTimeout(25);
  micI2S.setPinsPdmRx(MIC_CLK_PIN, MIC_DATA_PIN);
  if (!micI2S.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("[ERR] mic i2s begin failed");
    return false;
  }

  // Flush startup buffer.
  uint8_t flush[128];
  for (int i = 0; i < 8; i++) {
    (void)micI2S.readBytes((char *)flush, sizeof(flush));
    delay(2);
  }
  return true;
}

static bool initSd() {
  sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, sdSpi)) {
    Serial.println("[ERR] SD begin failed");
    return false;
  }
  return true;
}

static String makeFileName() {
  uint32_t t = millis() / 1000;
  return String("/aud_test_") + String(t) + String(".wav");
}

static void recordWavToSd() {
  String path = makeFileName();
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    Serial.println("[ERR] open wav file failed");
    return;
  }

  // Reserve header first, then patch with actual data size after recording.
  writeWavHeader(f, 0);

  const uint32_t targetBytes = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8) * RECORD_SECONDS;
  uint8_t chunk[READ_CHUNK_BYTES];
  uint32_t writtenBytes = 0;

  Serial.println("[REC] start");
  Serial.print("[REC] file: ");
  Serial.println(path);
  Serial.print("[REC] target bytes: ");
  Serial.println(targetBytes);

  uint32_t startMs = millis();
  while (writtenBytes < targetBytes) {
    size_t ask = targetBytes - writtenBytes;
    if (ask > sizeof(chunk)) ask = sizeof(chunk);

    size_t got = micI2S.readBytes((char *)chunk, ask);
    if (got == 0) {
      delay(1);
      continue;
    }

    size_t aligned = got & ~((size_t)1);
    if (aligned == 0) continue;

    if (f.write(chunk, aligned) != aligned) {
      Serial.println("[ERR] write wav data failed");
      break;
    }

    writtenBytes += (uint32_t)aligned;

    if ((millis() - startMs) % 1000 < 40) {
      Serial.print("[REC] bytes: ");
      Serial.println(writtenBytes);
    }
  }

  writeWavHeader(f, writtenBytes);
  f.flush();
  f.close();

  uint32_t elapsed = millis() - startMs;
  Serial.println("[REC] done");
  Serial.print("[REC] duration ms: ");
  Serial.println(elapsed);
  Serial.print("[REC] bytes written: ");
  Serial.println(writtenBytes);
  Serial.print("[REC] saved: ");
  Serial.println(path);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("audio sd wav test");
  if (!initSd()) {
    while (1) delay(1000);
  }
  if (!initMic()) {
    while (1) delay(1000);
  }

  recordWavToSd();
}

void loop() {
  delay(1000);
}
