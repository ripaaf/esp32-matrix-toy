#include "esp_camera.h"
#include <Arduino.h>

// XIAO ESP32S3 Sense camera pins
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// UART: XIAO TX -> Matrix RX
static const int UART_TX_PIN = 43;
static const int UART_RX_PIN = -1;
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
  PKT_PING = 0x04,

  // WIFI Offload via XIAO
  PKT_WIFI_SCAN_CMD = 0x50,
  PKT_WIFI_SCAN_RES = 0x51,
  PKT_DEAUTH_ATK_CMD = 0x52,
  PKT_DEAUTH_ATK_STAT = 0x53
};

static const uint16_t CAM_W = 96;
static const uint16_t CAM_H = 96;

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

static void sendPacket(uint8_t type, const uint8_t *payload, uint16_t len) {
  uint8_t hdr[5];
  hdr[0] = PKT_HEAD1;
  hdr[1] = PKT_HEAD2;
  hdr[2] = type;
  hdr[3] = (uint8_t)(len & 0xFF);
  hdr[4] = (uint8_t)((len >> 8) & 0xFF);

  uint8_t crc = 0;
  crc = crc8_update(crc, hdr[2]);
  crc = crc8_update(crc, hdr[3]);
  crc = crc8_update(crc, hdr[4]);
  if (payload && len) crc = crc8_buf(payload, len) ^ crc;

  Serial1.write(hdr, sizeof(hdr));
  if (payload && len) Serial1.write(payload, len);
  Serial1.write(crc);
  Serial1.write(PKT_TAIL1);
  Serial1.write(PKT_TAIL2);
}

static bool initCamera() {
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;

  // No JPEG encoding: send RGB565 directly.
  c.pixel_format = PIXFORMAT_RGB565;
  c.frame_size = FRAMESIZE_96X96;
  c.jpeg_quality = 0;
  c.fb_count = 1;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("camera init error: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
    s->set_brightness(s, -1);
    s->set_contrast(s, -1);
    s->set_saturation(s, -2);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  if (!initCamera()) {
    Serial.println("camera init failed, restart...");
    delay(2000);
    ESP.restart();
  }

  Serial.println("xiao rgb sender ready");
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    delay(20);
    return;
  }

  if (fb->width != CAM_W || fb->height != CAM_H || fb->format != PIXFORMAT_RGB565) {
    esp_camera_fb_return(fb);
    delay(30);
    return;
  }

  uint8_t startPayload[4] = {
    (uint8_t)(CAM_W & 0xFF),
    (uint8_t)((CAM_W >> 8) & 0xFF),
    (uint8_t)(CAM_H & 0xFF),
    (uint8_t)((CAM_H >> 8) & 0xFF)
  };
  sendPacket(PKT_FRAME_START, startPayload, sizeof(startPayload));

  const uint16_t lineBytes = CAM_W * 2;
  uint8_t linePacket[2 + CAM_W * 2];

  for (uint16_t y = 0; y < CAM_H; y++) {
    linePacket[0] = (uint8_t)(y & 0xFF);
    linePacket[1] = (uint8_t)((y >> 8) & 0xFF);
    const uint8_t *src = fb->buf + (size_t)y * lineBytes;
    memcpy(linePacket + 2, src, lineBytes);
    sendPacket(PKT_LINE, linePacket, (uint16_t)(2 + lineBytes));
  }

  sendPacket(PKT_FRAME_END, nullptr, 0);
  esp_camera_fb_return(fb);

  // Tuned for smoother yet stable transport in RGB565 mode.
  delay(90);
}
