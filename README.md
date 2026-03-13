# KaniOS — ESP32S3 Matrix Smart Gadget OS

| Front | Back |
| --- | --- |
| <img src="images/front.jpeg" alt="KaniOS device - front" width="420" /> | <img src="images/back.jpeg" alt="KaniOS device - back" width="420" /> |

*im not using any rf anntenna because im not done implementing it also the rubber is to keep the battery in place.. :)*

**KaniOS** is a custom ESP32-based operating system for a portable smart gadget. It combines entertainment features (games, music, animations), utility tools (calculator, timer, stopwatch), and network security tools (Deauther, Spoofer, Jammer) in one interactive interface.

---

## 🧰 Arduino IDE Settings

<img src="images/arduinosetting.png" alt="Arduino IDE settings for KaniOS" width="720" />

---

## ✨ Key Features

- **Menu-Based UI:** Simple, responsive navigation using 3 physical buttons (Up, Down, OK).
- **Apps & Tools:**
  - Calculator, Stopwatch, and Timer.
  - Flashlight (uses the LED matrix).
  - **Penetration Testing Tools:** WiFi Scanner & Deauther (WiFi Killer), Apple BLE Spoofer, BLE Monitor, and a 2.4GHz RF Jammer (**requires an optional NRF24 module**).
  - **File Explorer:** Manage, view, and delete files directly on the device.
  - **Note Editor:** Create and play buzzer tones/music directly on-device.
- **Gyro/IMU-Based Games:**
  - *Space Gyro* (Shooter)
  - *Gyro Tetris*
  - *Pong* (vs AI)
  - *Water Simulation* (water physics simulation on the LED matrix using the IMU)
- **Media Player:** BMP image viewer and GIF animation player.
- **Web Control Panel:** Built-in web server to control the device via phone/PC browser (upload images, draw on the LED matrix, remote settings).
- **Full Customization:** Screen brightness, LED brightness, buzzer volume, sleep mode, and timezone (UTC) settings.

---

## 🛠️ Hardware Requirements

- **Microcontroller:** ESP32 (LittleFS supported)
- **Display:** TFT ST7789 (240x240) via SPI
- **LED Matrix:** WS2812B 8x8 (64 LEDs)
- **IMU Sensor:** QMI8658 (Accelerometer & Gyroscope) via I2C
- **Buzzer:** Passive/active buzzer module
- **Buttons:** 3x push buttons (Up, Down, OK) using `INPUT_PULLUP`
- **Optional:** NRF24L01+ module (**only needed for the 2.4GHz RF Jammer feature**)

### 📌 Pin Configuration (Pinout)

Below is the full pin mapping according to the source code. Make sure your wiring matches this table:

| Module / Component | ESP32 Pin | Additional Notes |
| :--- | :---: | :--- |
| **TFT Display (ST7789)** |  |  |
| `TFT_DC` (Data/Command) | 34 |  |
| `TFT_RST` (Reset) | 35 |  |
| `TFT_MOSI` (SDA) | 36 | Connect to SDA/DIN on the display |
| `TFT_SCLK` (SCL) | 37 | Connect to SCL/CLK on the display |
| `TFT_BLK` (Backlight) | 33 | Used to control display brightness |
| `TFT_CS` (Chip Select) | -1 | Unused / tied to GND |
| **LED Matrix (WS2812)** |  |  |
| `RGB_CONTROL_PIN` | 14 | Connect to the matrix DIN (Data In) |
| **IMU Sensor (QMI8658)** |  |  |
| `I2C_SDA` | 11 | I2C data line |
| `I2C_SCL` | 12 | I2C clock line |
| **Buttons (Push Buttons)** |  | Uses internal pull-up |
| `BTN_UP_PIN` | 38 | Up navigation button |
| `BTN_DOWN_PIN` | 39 | Down navigation button |
| `BTN_OK_PIN` | 40 | Select / confirm / back button |
| **Buzzer** |  |  |
| `BUZZER_PIN` | 1 | Connect to the buzzer module |
| **NRF24L01+ Module** |  | **[OPTIONAL]** |
| `NRF_CE_PIN` | 15 | Chip Enable |
| `NRF_CSN_PIN` | 16 | Chip Select Not |

> **Note:** The **NRF24** module also needs the ESP32 standard SPI lines (**MISO, MOSI, SCK**) if enabled. If the module is not installed, the jammer feature will automatically show the warning **"NO NRF24 MODULE!"**.

---

## 💻 Software Requirements (Dependencies)

Install these libraries in Arduino IDE or PlatformIO before compiling:

- `WiFi`, `WebServer`, `LittleFS`, `Wire`, `SPI` (built into the ESP32 core)
- Adafruit GFX Library
- Adafruit ST7735 and ST7789 Library
- FastLED
- AnimatedGIF
- WiFiManager (used during first WiFi setup)
- RF24 (**optional**, for the jammer feature; the code can detect if this library is missing)

---

## 🚀 Installation Guide

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/username/KaniOS.git
   cd KaniOS
   ```

2. **Compile and Upload the Code:**
   - Select your ESP32 board in Arduino IDE.
   - Since the program is large, set the partition scheme to **No OTA (2MB APP / 2MB SPIFFS)**.
   - Click **Upload**.

3. **First-Time WiFi Setup:**
   - Power on the device. If WiFi credentials are empty, it will create an Access Point named **Ripa-Setup**.
   - Connect your phone/PC to that AP and open **192.168.4.1** in a browser.
   - Enter your local WiFi name and password, then save. The ESP32 will reboot automatically and enter the KaniOS main interface.

---

## 🎮 How to Use

- **Basic Navigation:** Use **UP** and **DOWN** to scroll through menu items. Press **OK** to select.
- **Back Function:** Press and hold **OK** for ~1 second on any screen to cancel an action, return to the previous menu, or exit an app/game.
- **Time Settings:** The clock may not match your location by default. Go to **Setting > UTC** and set the correct offset (example: choose **+08:00** for WITA/Makassar).
- **Web Portal:** The ESP32’s IP address will always appear at the bottom bar of the main menu (example: `192.168.x.x`). Open that IP in a browser on another device on the same network to access the **Control Panel**.
- **Upload Media:** From the web control panel, you can upload images (**.bmp 24-bit uncompressed**) or **.gif** animations directly to the device storage.

---

## ⚠️ Legal & Ethical Warning (Disclaimer)

This project includes code/tools that can disrupt communications (WiFi Deauther, BLE Spoofer, RF Jammer). These tools are provided **ONLY FOR EDUCATIONAL PURPOSES AND AUTHORIZED SECURITY TESTING** on networks/devices you legally own or have explicit permission to test.

Using jammers or deauthers on public networks, public facilities, or other people’s devices without written permission is **ILLEGAL** and may lead to criminal penalties. The creator and contributors are not responsible for damages or legal violations caused by misuse. Use responsibly.
