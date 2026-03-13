# KaniOS — ESP32 Smart Gadget OS

**KaniOS** (juga dikenal sebagai **Ripaaf**) adalah sistem operasi *custom* berbasis ESP32 untuk *smart gadget* portabel. Perangkat ini menggabungkan fitur hiburan (*game*, musik, animasi), alat utilitas (kalkulator, *timer*, *stopwatch*), hingga *network security tools* (*Deauther*, *Spoofer*, *Jammer*) dalam satu antarmuka yang interaktif.

## ✨ Fitur Utama

- **Antarmuka Berbasis Menu (UI):** Navigasi simpel dan responsif menggunakan 3 tombol fisik (Up, Down, OK).
- **Aplikasi & Alat (Tools):**
  - Kalkulator, Stopwatch, & Timer.
  - Lampu Senter (memanfaatkan LED Matrix).
  - **Penetration Testing Tools:** WiFi Scanner & Deauther (WiFi Killer), Apple BLE Spoofer, BLE Monitor, dan 2.4GHz RF Jammer (**menggunakan modul NRF24 opsional**).
  - **File Explorer:** Kelola, lihat, dan hapus file langsung dari perangkat.
  - **Note Editor:** Buat dan putar nada/musik *buzzer* secara langsung (*on-device*).
- **Game Berbasis Gyro (IMU):**
  - *Space Gyro* (Shooter)
  - *Gyro Tetris*
  - *Pong* (vs AI)
  - *Water Simulation* (simulasi fisika air di LED Matrix menggunakan IMU)
- **Media Player:** Penampil gambar BMP dan pemutar animasi GIF.
- **Web Control Panel:** Web server bawaan untuk mengontrol perangkat via *browser* HP/PC (upload gambar, menggambar di LED matrix, dan pengaturan jarak jauh).
- **Kustomisasi Lengkap:** Pengaturan kecerahan layar, LED, volume *buzzer*, *sleep mode*, dan zona waktu (UTC).

---

## 🛠️ Persyaratan Perangkat Keras (Hardware)

- **Microcontroller:** ESP32 (support LittleFS)
- **Layar:** TFT ST7789 (240x240 pixel) via antarmuka SPI
- **LED Matrix:** WS2812B 8x8 (64 LED)
- **IMU Sensor:** QMI8658 (Accelerometer & Gyroscope) via I2C
- **Buzzer:** Modul Buzzer Pasif/Aktif
- **Tombol:** 3x Push Button (Up, Down, OK) dengan konfigurasi `INPUT_PULLUP`
- **Opsional:** Modul NRF24L01+ (**hanya dibutuhkan jika ingin menggunakan fitur 2.4GHz RF Jammer**)

### 📌 Dokumentasi Konfigurasi Pin (Pinout)

Berikut adalah pemetaan pin lengkap sesuai dengan *source code*. Pastikan pengkabelan (*wiring*) sesuai dengan tabel di bawah ini:

| Modul / Komponen | Pin di ESP32 | Keterangan Tambahan |
| :--- | :---: | :--- |
| **Layar TFT (ST7789)** |  |  |
| `TFT_DC` (Data/Command) | 34 |  |
| `TFT_RST` (Reset) | 35 |  |
| `TFT_MOSI` (SDA) | 36 | Sambungkan ke pin SDA/DIN pada layar |
| `TFT_SCLK` (SCL) | 37 | Sambungkan ke pin SCL/CLK pada layar |
| `TFT_BLK` (Backlight) | 33 | Untuk mengontrol kecerahan layar |
| `TFT_CS` (Chip Select) | -1 | *Unused / tied to GND* |
| **LED Matrix (WS2812)** |  |  |
| `RGB_CONTROL_PIN` | 14 | Sambungkan ke pin DIN (Data In) matrix |
| **Sensor IMU (QMI8658)** |  |  |
| `I2C_SDA` | 11 | Jalur Data I2C |
| `I2C_SCL` | 12 | Jalur Clock I2C |
| **Tombol (Push Buttons)** |  | Menggunakan internal pull-up |
| `BTN_UP_PIN` | 38 | Tombol navigasi Atas |
| `BTN_DOWN_PIN` | 39 | Tombol navigasi Bawah |
| `BTN_OK_PIN` | 40 | Tombol Pilih / Konfirmasi / Kembali |
| **Buzzer** |  |  |
| `BUZZER_PIN` | 1 | Sambungkan ke modul *buzzer* |
| **Modul NRF24L01+** |  | **[OPSIONAL]** |
| `NRF_CE_PIN` | 15 | Chip Enable |
| `NRF_CSN_PIN` | 16 | Chip Select Not |

> Catatan: Modul **NRF24** juga membutuhkan koneksi ke jalur **SPI standar ESP32** untuk **MISO, MOSI, dan SCK** jika diaktifkan. Jika modul tidak dipasang, fitur Jammer akan otomatis menampilkan peringatan **"NO NRF24 MODULE!"**.

---

## 💻 Kebutuhan Perangkat Lunak (Dependencies)

Pastikan kamu telah menginstal *library* berikut di Arduino IDE atau PlatformIO sebelum melakukan kompilasi:

- `WiFi`, `WebServer`, `LittleFS`, `Wire`, `SPI` (bawaan core ESP32)
- Adafruit GFX Library
- Adafruit ST7735 and ST7789 Library
- FastLED
- AnimatedGIF
- WiFiManager (digunakan saat setup WiFi pertama kali)
- RF24 (**opsional**, untuk fitur Jammer. Kode otomatis mendeteksi jika *library* ini tidak ada)

---

## 🚀 Cara Instalasi

1. **Clone Repository:**
   ```bash
   git clone https://github.com/username/KaniOS.git
   cd KaniOS
   ```

2. **Siapkan Memori Internal (LittleFS):**  
   Sistem ini membutuhkan file sistem untuk menyimpan gambar, konfigurasi, dan catatan. Gunakan plugin **ESP32 LittleFS Data Upload** di Arduino IDE untuk mengunggah folder `data/` ke flash memory ESP32.

3. **Kompilasi dan Upload Kode:**
   - Pilih board ESP32 yang kamu gunakan di Arduino IDE.
   - Karena programnya cukup besar, atur skema partisi minimal ke **Huge APP (3MB No OTA / 1MB SPIFFS)**.
   - Klik **Upload**.

4. **Setup Awal (WiFi):**
   - Nyalakan perangkat. Jika kredensial WiFi kosong, perangkat akan memancarkan Access Point bernama **Ripa-Setup**.
   - Sambungkan HP/PC kamu ke AP tersebut dan buka alamat **192.168.4.1** di browser.
   - Masukkan nama dan password WiFi lokal kamu, lalu simpan. ESP32 akan reboot otomatis dan masuk ke antarmuka utama KaniOS.

---

## 🎮 Cara Penggunaan

- **Navigasi Dasar:** Gunakan tombol **UP** dan **DOWN** untuk menggulir daftar menu. Tekan **OK** untuk memilih.
- **Fungsi Kembali (Back):** Tekan dan tahan tombol **OK** selama ~1 detik di layar mana pun untuk membatalkan aksi, kembali ke menu sebelumnya, atau keluar dari aplikasi/game.
- **Pengaturan Waktu:** Jam secara default mungkin tidak sesuai dengan lokasimu. Masuk ke menu **Setting > UTC** dan sesuaikan offset waktu (misalnya, pilih **+08:00** untuk waktu wilayah WITA/Makassar).
- **Web Portal:** Alamat IP ESP32 akan selalu tampil di bilah bawah layar menu utama (misal: `192.168.x.x`). Buka IP tersebut via browser perangkat lain di jaringan yang sama untuk mengakses **Control Panel**.
- **Upload Media:** Dari Control Panel web, kamu bisa mengunggah file gambar (**.bmp 24-bit tanpa kompresi**) atau animasi **.gif** langsung ke storage perangkat.

---

## ⚠️ Peringatan Hukum & Etika (Disclaimer)

Project ini mencakup kode dan alat yang dapat mengganggu jaringan komunikasi (WiFi Deauther, BLE Spoofer, RF Jammer). Alat-alat ini disertakan **HANYA UNTUK TUJUAN EDUKASI DAN PENGUJIAN KEAMANAN** pada perangkat dan jaringan yang secara sah kamu miliki.

Menggunakan fungsi jammer atau deauther pada jaringan publik, fasilitas umum, atau perangkat milik orang lain tanpa izin tertulis yang sah adalah tindakan **ILEGAL** dan dapat dikenakan sanksi pidana. Pembuat atau kontributor repository ini tidak bertanggung jawab atas kerugian atau pelanggaran hukum yang diakibatkan oleh penyalahgunaan software ini. Gunakan dengan bijak dan bertanggung jawab!
