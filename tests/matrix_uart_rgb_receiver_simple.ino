#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// TFT pins from your matrix project
#define TFT_CS    -1
#define TFT_DC     34
#define TFT_RST    35
#define TFT_MOSI   36
#define TFT_SCLK   37
#define TFT_BLK    33

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

// UART: Matrix RX <- XIAO TX
static const int UART_RX_PIN = 44;
static const int UART_TX_PIN = -1;
static const uint32_t UART_BAUD = 921600;

// Packet framing
static const uint8_t PKT_HEAD1 = 0xC3;
static const uint8_t PKT_HEAD2 = 0x3C;
static const uint8_t PKT_TAIL1 = 0xA5;
static const uint8_t PKT_TAIL2 = 0x5A;

enum PacketType : uint8_t {
  PKT_FRAME_START = 0x01,
  PKT_LINE = 0x02,
  PKT_FRAME_END = 0x03,
  PKT_PING = 0x04
};

static const uint16_t MAX_W = 160;
static const uint16_t MAX_H = 120;
static uint16_t frameW = 96;
static uint16_t frameH = 96;

static uint16_t drawX = 24;
static uint16_t drawY = 24;
static uint16_t drawW = 192;
static uint16_t drawH = 192;

static uint8_t payloadBuf[2 + MAX_W * 2];
static uint16_t frameBuf[MAX_W * MAX_H];
static uint16_t prevFrameBuf[MAX_W * MAX_H];
static bool frameReady = false;
static uint16_t scaledLine[240];
static bool clearNeeded = true;
static bool havePrevFrame = false;
static bool lineSeen[MAX_H];
static uint16_t linesReceived = 0;

static uint32_t okPackets = 0;
static uint32_t crcErrors = 0;
static uint32_t framingErrors = 0;
static uint32_t timeoutErrors = 0;
static uint32_t droppedFrames = 0;
static uint32_t lastStatMs = 0;

static uint8_t crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

static uint8_t crc8_buf(const uint8_t *buf, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; i++) c = crc8_update(c, buf[i]);
  return c;
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

static void computeLayout() {
  // Fit frame to screen while keeping aspect ratio.
  float sx = (float)tft.width() / (float)frameW;
  float sy = (float)tft.height() / (float)frameH;
  float s = (sx < sy) ? sx : sy;
  drawW = (uint16_t)((float)frameW * s + 0.5f);
  drawH = (uint16_t)((float)frameH * s + 0.5f);
  if (drawW == 0) drawW = 1;
  if (drawH == 0) drawH = 1;
  drawX = (uint16_t)((tft.width() - drawW) / 2);
  drawY = (uint16_t)((tft.height() - drawH) / 2);
}

static void renderFrame() {
  if (!frameReady || frameW == 0 || frameH == 0) return;

  // Drop incomplete frames to avoid artifacts like random lines near top.
  if (linesReceived < frameH) {
    droppedFrames++;
    frameReady = false;
    return;
  }

  if (clearNeeded) {
    tft.fillScreen(ST77XX_BLACK);
  }

  // Nearest-neighbor scaling from frameBuf to draw area.
  for (uint16_t dy = 0; dy < drawH; dy++) {
    uint16_t sy = (uint16_t)((uint32_t)dy * frameH / drawH);
    const uint16_t *srcRow = frameBuf + (size_t)sy * frameW;

    if (!clearNeeded && havePrevFrame) {
      const uint16_t *prevRow = prevFrameBuf + (size_t)sy * frameW;
      if (memcmp(srcRow, prevRow, (size_t)frameW * sizeof(uint16_t)) == 0) {
        continue;
      }
    }

    for (uint16_t dx = 0; dx < drawW; dx++) {
      uint16_t sx = (uint16_t)((uint32_t)dx * frameW / drawW);
      scaledLine[dx] = srcRow[sx];
    }
    tft.drawRGBBitmap(drawX, drawY + dy, scaledLine, drawW, 1);
  }

  memcpy(prevFrameBuf, frameBuf, (size_t)frameW * frameH * sizeof(uint16_t));
  havePrevFrame = true;
  clearNeeded = false;

  frameReady = false;
}

static bool readOnePacket() {
  // seek header
  uint8_t prev = 0;
  bool found = false;
  uint32_t start = millis();
  while ((millis() - start) < 150) {
    if (Serial1.available() > 0) {
      uint8_t b = (uint8_t)Serial1.read();
      if (prev == PKT_HEAD1 && b == PKT_HEAD2) {
        found = true;
        break;
      }
      prev = b;
    } else {
      delay(1);
    }
  }
  if (!found) {
    timeoutErrors++;
    return false;
  }

  uint8_t meta[3] = {0}; // type + lenL + lenH
  if (!readBytesUart(meta, sizeof(meta), 60)) {
    timeoutErrors++;
    return false;
  }

  uint8_t type = meta[0];
  uint16_t len = (uint16_t)meta[1] | (uint16_t)((uint16_t)meta[2] << 8);
  if (len > sizeof(payloadBuf)) {
    framingErrors++;
    return false;
  }

  if (len > 0 && !readBytesUart(payloadBuf, len, 120)) {
    timeoutErrors++;
    return false;
  }

  uint8_t rxCrc = 0;
  uint8_t tail[2] = {0};
  if (!readBytesUart(&rxCrc, 1, 30) || !readBytesUart(tail, 2, 30)) {
    timeoutErrors++;
    return false;
  }

  if (tail[0] != PKT_TAIL1 || tail[1] != PKT_TAIL2) {
    framingErrors++;
    return false;
  }

  uint8_t calc = 0;
  calc = crc8_update(calc, meta[0]);
  calc = crc8_update(calc, meta[1]);
  calc = crc8_update(calc, meta[2]);
  if (len > 0) calc ^= crc8_buf(payloadBuf, len);

  if (calc != rxCrc) {
    crcErrors++;
    return false;
  }

  if (type == PKT_FRAME_START) {
    if (len != 4) {
      framingErrors++;
      return false;
    }
    uint16_t w = (uint16_t)payloadBuf[0] | (uint16_t)((uint16_t)payloadBuf[1] << 8);
    uint16_t h = (uint16_t)payloadBuf[2] | (uint16_t)((uint16_t)payloadBuf[3] << 8);
    if (w == 0 || h == 0 || w > MAX_W || h > MAX_H) {
      framingErrors++;
      return false;
    }
    frameW = w;
    frameH = h;
    computeLayout();
    frameReady = false;
    clearNeeded = true;
    havePrevFrame = false;
    linesReceived = 0;
    memset(lineSeen, 0, sizeof(lineSeen));
  } else if (type == PKT_LINE) {
    if (len < 2) {
      framingErrors++;
      return false;
    }
    uint16_t y = (uint16_t)payloadBuf[0] | (uint16_t)((uint16_t)payloadBuf[1] << 8);
    if (len != (uint16_t)(2 + frameW * 2)) {
      framingErrors++;
      return false;
    }
    if (y < frameH) {
      const uint8_t *src = payloadBuf + 2;
      uint16_t *dst = frameBuf + (size_t)y * frameW;
      for (uint16_t x = 0; x < frameW; x++) {
        // Camera sends RGB565 high-byte first in this stream path.
        dst[x] = (uint16_t)((uint16_t)src[x * 2] << 8) | (uint16_t)src[x * 2 + 1];
      }
      if (!lineSeen[y]) {
        lineSeen[y] = true;
        linesReceived++;
      }
      frameReady = true;
    }
  } else if (type == PKT_FRAME_END) {
    renderFrame();
    frameReady = false;
    linesReceived = 0;
    memset(lineSeen, 0, sizeof(lineSeen));
  }

  okPackets++;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Match the matrix board wiring used by your main firmware.
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);

  tft.init(240, 240, SPI_MODE2);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(8, 8);
  tft.print("UART RGB RX ready");
  delay(250);
  tft.fillScreen(ST77XX_BLUE);
  delay(120);
  tft.fillScreen(ST77XX_BLACK);

  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  computeLayout();
}

void loop() {
  (void)readOnePacket();

  uint32_t now = millis();
  if (now - lastStatMs >= 1000) {
    lastStatMs = now;
    Serial.printf("ok=%lu crc=%lu frame=%lu timeout=%lu drop=%lu baud=%lu rx=%d\n",
                  (unsigned long)okPackets,
                  (unsigned long)crcErrors,
                  (unsigned long)framingErrors,
                  (unsigned long)timeoutErrors,
                  (unsigned long)droppedFrames,
                  (unsigned long)UART_BAUD,
                  UART_RX_PIN);
  }
}
