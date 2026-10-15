# M5 Cardputer + Si5351 DDS VFO

[![Board](https://img.shields.io/badge/board-M5Stack%20Cardputer%20v1.1-orange)](#kebutuhan-hardware)
[![SoC](https://img.shields.io/badge/SoC-ESP32--S3-E7352C?logo=espressif&logoColor=white)](https://www.espressif.com/en/products/socs/esp32-s3)
[![Framework](https://img.shields.io/badge/ESP--IDF-v6.1-E7352C?logo=espressif&logoColor=white)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
[![Language](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![UI](https://img.shields.io/badge/LVGL-v9.6-343434?logo=lvgl&logoColor=white)](https://lvgl.io/)
[![BSP](https://img.shields.io/badge/BSP-espp%2Fm5stack--cardputer-blueviolet)](https://github.com/esp-cpp/espp)
[![RF source](https://img.shields.io/badge/RF%20source-Si5351A-008080)](https://www.silabs.com/timing)
[![Output](https://img.shields.io/badge/output-1%E2%80%93150%20MHz-success)](#spesifikasi-sinyal-output)
[![License](https://img.shields.io/badge/license-MIT-green)](#lisensi)

[![Last commit](https://img.shields.io/github/last-commit/Muhammad-Yunus/M5Cardputer-Si5351-VFO?logo=github)](https://github.com/Muhammad-Yunus/M5Cardputer-Si5351-VFO/commits/main)
[![Issues](https://img.shields.io/github/issues/Muhammad-Yunus/M5Cardputer-Si5351-VFO?logo=github)](https://github.com/Muhammad-Yunus/M5Cardputer-Si5351-VFO/issues)
[![Stars](https://img.shields.io/github/stars/Muhammad-Yunus/M5Cardputer-Si5351-VFO?style=social)](https://github.com/Muhammad-Yunus/M5Cardputer-Si5351-VFO/stargazers)
[![Forks](https://img.shields.io/github/forks/Muhammad-Yunus/M5Cardputer-Si5351-VFO?style=social)](https://github.com/Muhammad-Yunus/M5Cardputer-Si5351-VFO/network/members)

Firmware **VFO (Variable Frequency Oscillator) berbasis DDS Si5351** untuk
**M5Stack Cardputer v1.1**, ditulis dengan **ESP-IDF v6.1 (C++)** dan menampilkan
antarmuka bergaya **dial analog retro** di layar 240×135.

Cardputer berperan sebagai *controller* + *display* + *keypad*, sedangkan modul
**Si5351A** (lewat konektor Grove) menghasilkan sinyal clock/RF presisi.

---

## Fitur

- **Rentang output 1 MHz – 150 MHz** pada `CLK0` Si5351 (LO).
- **Tuning bertahap (step):** 10 Hz, 100 Hz, 1 kHz, 10 kHz, 100 kHz, 1 MHz, 10 MHz.
- **20 memori** frekuensi (next/prev + simpan). 10 slot pertama sudah diisi
  preset band HF/VHF umum.
- **Entry frekuensi langsung** (ketik dalam Hz lalu Enter).
- **IF offset** yang bisa diatur: LO = frekuensi tampil + offset, sedangkan
  display tetap menunjukkan frekuensi sinyal (untuk receiver superheterodyne).
- **Carrier / BFO I/Q** pada `CLK1` (I) dan `CLK2` (Q), 90° terpisah, dengan
  mode **USB / LSB** (±1500 Hz) dan on/off.
- **Tampilan retro:** angka frekuensi besar + **skala dial bergerak** dengan
  pointer tetap di tengah + tick minor/major + label amber.
- **Status bar:** step aktif (kiri) dan nomor memori (kanan).
- **LED RGB** internal diset warna amber hangat (meniru lampu dial).
- Update frekuensi **real-time** ke Si5351 setiap kali diubah.

---

## Kebutuhan Hardware

| Bagian        | Detail                                                            |
|---------------|-------------------------------------------------------------------|
| Board         | M5Stack Cardputer v1.1 (modul M5Stamp S3)                         |
| SoC           | ESP32-S3, 8 MB Flash                                              |
| Display       | ST7789 240×135 (via BSP)                                          |
| Keyboard      | Matriks 4×14 (74HC138) via BSP                                    |
| Sumber RF     | Modul **Si5351A** (breakout generik, kristal 25 MHz)             |
| Bus kontrol   | I2C di konektor **Grove**                                         |

### Koneksi Grove → Si5351

| Sinyal | GPIO Cardputer | Keterangan            |
|--------|----------------|-----------------------|
| SCL    | GPIO 1         | Clock I2C             |
| SDA    | GPIO 2         | Data I2C              |
| 3V3    | 3V3            | Catu daya modul       |
| GND    | GND            | Ground                |
| OUT    | —              | `CLK0` = LO (VFO)     |
| AUX    | —              | `CLK1` = carrier I, `CLK2` = carrier Q (BFO) |

Alamat I2C Si5351: **0x60**.

> **Catatan:** bus Grove memakai I2C port 0. BSP terlebih dahulu dipicu
> mendeteksi varian board (`cardputer.variant()`) agar bus internal ADV di
> GPIO 8/9 selesai dan port 0 dilepas sebelum diklaim untuk Si5351.

---

## Arsitektur Perangkat Lunak

- **ESP-IDF v6.1**, bahasa **C++**.
- BSP resmi **`espp/m5stack-cardputer`** (managed component) — driver LCD,
  keyboard matriks, LED, dll.
- **LVGL v9.6** untuk UI.
- Driver Si5351 kustom: **`app/main/si5351.hpp`** (header-only).

### Strategi driver Si5351 (`si5351.hpp`)

- **LO (CLK0):** **PLLA VCO dikunci tetap di 900 MHz** (feedback divider =
  900/25 = 36, integer, dari kristal 25 MHz). Frekuensi diatur lewat
  **MultiSynth** `VCO / Fout` (fraksional bila perlu, resolusi ≈ 0.1 Hz).
- **Carrier (CLK1/CLK2):** PLLB dengan divider **integer N** (9–127) sehingga
  `VCO = N × Fout`; register phase-offset `CLK2 = N` memberi **90° tepat** (trik
  AN619). Karena itu carrier dibatasi ≈ **4.8 – 100 MHz**.
- Ini menjaga LO dan carrier pada **PLL terpisah** sehingga tuning tidak
  mengganggu fasa carrier.
- API: `init()`, `set_frequency(freq_hz, clk=0)`, `set_carrier(freq_hz, enable)`,
  `frequency()`.

### Alur `app_main()`

1. Inisialisasi LCD, display (LVGL), brightness, dan LED.
2. `cardputer.variant()` → mendeteksi varian & melepas I2C port 0.
3. Buat bus I2C Grove + Si5351, lalu set frekuensi awal (7.074 MHz).
4. Bangun UI VFO (readout, skala dial, status bar).
5. `initialize_keyboard(callback)` — callback hanya memasukkan event ke
   `key_queue` (aman dari konteks task keyboard).
6. **Loop utama:** `lv_timer_handler()` + proses `key_queue` + log status
   periodik 3 detik.

---

## Struktur Proyek

```
M5Cardputer-Si5351-VFO/
├─ README.md
└─ app/
   ├─ CMakeLists.txt
   ├─ sdkconfig
   ├─ main/
   │  ├─ CMakeLists.txt      # REQUIRES espp__m5stack-cardputer lvgl
   │  ├─ idf_component.yml   # dependensi espp/m5stack-cardputer
   │  ├─ app.cpp             # setup + VFO UI + input keyboard
   │  └─ si5351.hpp          # driver Si5351 (header-only)
   └─ managed_components/    # BSP, LVGL, dll (component manager)
```

---

## Kontrol (Key Map)

| Tombol              | Fungsi                                        |
|---------------------|-----------------------------------------------|
| `,` (kiri)          | Tuning **turun** sebesar step                 |
| `/` (kanan)         | Tuning **naik** sebesar step                  |
| `;` (atas)          | **Step dinaikkan** (10Hz → 10MHz)             |
| `.` (bawah)         | **Step diturunkan** (10MHz → 10Hz)            |
| `0`–`9` lalu `Enter`| **Entry langsung** (nilai dalam Hz)          |
| `` ` `` (Esc)       | Batalkan entry langsung                       |
| `m`                 | Memori **berikutnya** (memuat frekuensinya)   |
| `n`                 | Memori **sebelumnya**                         |
| `s`                 | **Simpan** frekuensi saat ini ke memori aktif |
| `c`                 | **Siklus carrier**: OFF → USB → LSB → OFF      |
| `o`                 | Set **IF offset** (Hz, boleh negatif lalu Enter) |
| `i`                 | Set **frekuensi carrier/BFO** (Hz lalu Enter)  |

Aransemen fisik `, . /` (bawah) dan `;` (atas) membentuk pola D-pad.

---

## Konfigurasi Penting (`sdkconfig`)

| Opsi | Nilai | Alasan |
|------|-------|--------|
| `CONFIG_IDF_TARGET`            | `esp32s3` | target SoC |
| Flash size                     | 8 MB      | modul M5Stamp S3 |
| Console output                 | `USB Serial/JTAG Controller` | monitor via USB |
| `CONFIG_ESP_MAIN_TASK_STACK_SIZE` | `16384` | LVGL butuh stack besar |
| `CONFIG_LV_FONT_MONTSERRAT_28` | `y`       | angka frekuensi besar |
| `CONFIG_LV_FONT_UNSCII_16`     | `y`       | font monospace retro |
| `CONFIG_ESPP_I2C_USE_NEW_API`  | `y`       | driver I2C baru |
| `CONFIG_FREERTOS_HZ`           | `100`     | hindari `pdMS_TO_TICKS(5)==0` |

---

## Build & Flash

```powershell
# 1. Aktifkan environment ESP-IDF
. C:\esp\v6.1\esp-idf\export.ps1

# 2. (sekali saja) set target
idf.py set-target esp32s3

# 3. Build + flash + monitor
idf.py -p COMxx build flash monitor
```

Keluar dari monitor dengan `Ctrl + ]`.

---

## Spesifikasi Sinyal Output

- **Output:** `CLK0` Si5351 (kotak, 50 Ω drive).
- **Rentang:** 1 MHz – 150 MHz (dibatasi di firmware).
- **Referensi:** kristal 25 MHz (asumsi modul Si5351A generik).
- **Resolusi:** ~0.1 Hz (MultiSynth fraksional, penyebut 20-bit).

---

## Sumber & Referensi

- Ide UI & fungsi VFO (retro dial analog):
  `github.com/RSZ-Nld/VFO-1.8-Inch-Retro-scale`
  (berbasis JF3HZB `Digital_VFO_with_analog_dial_V2`).
- BSP: `github.com/esp-cpp/espp` → komponen `m5stack-cardputer`.
- LVGL: `lvgl.io` (v9.x).

---

## Lisensi

Tambahkan lisensi sesuai kebutuhan proyek (mis. MIT).
