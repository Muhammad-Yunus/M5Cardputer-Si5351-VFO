#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include "i2c.hpp"
#include "lvgl.h"
#include "m5stack-cardputer.hpp"
#include "si5351.hpp"

// ---------------------------------------------------------------------------
// Retro DDS VFO (Si5351) for the M5Stack Cardputer
//   UI: large frequency readout + a sliding "analog dial" scale + status bar.
//   Keys (bottom-right keyboard cluster):
//     ,  = tune down        /  = tune up        (Fn + , / . / ; / also work)
//     ;  = step up          .  = step down
//     0-9 = direct frequency entry (Hz), Enter = apply, `(Esc) = cancel
//     m  = next memory      n  = previous memory  s = store into memory
//     c  = cycle carrier (OFF -> USB -> LSB)   o = set IF offset (Hz, +/-)
//     i  = set carrier/BFO frequency (Hz)
//     p  = toggle CH2 carrier phase: 90 deg (I/Q quadrature) <-> 0 deg
//     t  = toggle theme: dark <-> light (panel color inversion)
//   LO output = displayed frequency + IF offset; carrier I/Q on CLK1/CLK2.
// ---------------------------------------------------------------------------

namespace {

constexpr int32_t MIN_FREQ = 8000;       // 8 kHz
constexpr int32_t MAX_FREQ = 150000000;  // 150 MHz (Si5351 usable ceiling here)
constexpr int NUM_STEPS = 7;
constexpr int32_t STEP_HZ[NUM_STEPS] = {10, 100, 1000, 10000, 100000, 1000000, 10000000};
constexpr const char *STEP_NAME[NUM_STEPS] = {"10Hz", "100Hz", "1kHz", "10kHz",
                                              "100kHz", "1MHz", "10MHz"};
constexpr int NUM_MEM = 20;

// Preset band centre frequencies (slots 0..NUM_PRESETS-1). The memory chip
// shows MEM_ALIAS; the remaining slots stay free for user memories.
// Kept at or below MAX_FREQ (150 MHz): MAR/LMR (159/165 MHz) are out of range.
constexpr int NUM_PRESETS = 13;
constexpr int32_t MEM_PRESET_HZ[NUM_PRESETS] = {
    1900000,   3700000,   7050000,   10125000, 14175000, 18110000, 21225000,
    24940000,  28850000,  52000000,  98000000, 122500000, 146000000};
constexpr const char *MEM_ALIAS[NUM_PRESETS] = {
    "160M", "80M", "40M", "30M", "20M", "17M", "15M", "12M",
    "10M",  "6M",  "AIRM", "ATC", "2M"};

// Carrier / BFO (CLK1 = I, CLK2 = Q). USB/LSB shift the BFO off the carrier.
constexpr int32_t SB_OFFSET_HZ = 1500;
constexpr int32_t CAR_MIN_FREQ = 4800000;   // quadrature phase trick lower bound
constexpr int32_t CAR_MAX_FREQ = 100000000; // ... upper bound

// Layout (240 x 135)
constexpr int W = 240;
constexpr int H = 135;
constexpr int FOOTER_H = 22; // bottom status bar height
constexpr int CH_ROW_H = 14; // channel indicator row (CH2 CH1 CH0) at the very top
constexpr int CH_SEP_Y = 17; // separator line, with margin below the channel row
constexpr int TOP_SHIFT = 23; // existing UI pushed down by the channel row + margin
constexpr int SCALE_CENTER_X = 120;
constexpr int SCALE_PITCH = 20;
constexpr int NTICKS = 13; // odd -> symmetric around the center index
constexpr int TICK_TOP = 77; // was 62; grid shortened to make room for CH row
constexpr int TICK_MINOR_BOTTOM = 84;
constexpr int TICK_MAJOR_BOTTOM = 90;
constexpr int TICK_LABEL_Y = 92;
constexpr int POINTER_W = 22; // red triangle marker, points up
constexpr int POINTER_H = 16;

// Retro amber-on-black palette
constexpr uint32_t COL_READOUT = 0xFFCC66;
constexpr uint32_t COL_TICK_MINOR = 0x8A6A2A;
constexpr uint32_t COL_TICK_MAJOR = 0xFFCC66;
constexpr uint32_t COL_POINTER = 0xFF3B1E;
constexpr uint32_t COL_STATUS = 0x66CCAA;
constexpr uint32_t COL_FOOTER_BG = 0x101A19; // dark teal footer bar

struct KeyMsg {
  char value;
  uint8_t special;
  bool shift;
};

// What the numeric keypad is currently editing (none == normal tuning).
enum EntryTarget { ENTRY_NONE, ENTRY_FREQ, ENTRY_IF, ENTRY_CAR };

struct VfoState {
  int32_t freq = 7074000; // start at 7.074 MHz
  int step_index = 3;     // 10 kHz
  int mem_index = 0;
  int32_t memories[NUM_MEM] = {};
  EntryTarget entry_target = ENTRY_NONE;
  std::string entry;
  int32_t if_offset = 0;   // added to the display frequency to form the LO
  int32_t car_freq = 10000000; // carrier / BFO centre frequency (10 MHz)
  int car_mode = 0;        // 0 = off, 1 = CW, 2 = USB, 3 = LSB
};

VfoState vfo;
bool theme_light = false; // false = dark panel (INVON), true = swapped/light (INVOFF)
QueueHandle_t key_queue = nullptr;
Si5351 *syn = nullptr;

lv_obj_t *lbl_freq = nullptr;
lv_obj_t *freq_row = nullptr;
lv_obj_t *lbl_prefix = nullptr;
lv_obj_t *lbl_unit = nullptr;
lv_obj_t *step_chip = nullptr;
lv_obj_t *step_line = nullptr;
lv_obj_t *lbl_step = nullptr;
lv_obj_t *mem_chip = nullptr;
lv_obj_t *lbl_mem_icon = nullptr;
lv_obj_t *lbl_mem = nullptr;
lv_obj_t *status_bar = nullptr;
lv_obj_t *lbl_aux = nullptr;
lv_obj_t *car_chip = nullptr;
lv_obj_t *wave_line = nullptr;
lv_obj_t *flat_line = nullptr;
lv_obj_t *lbl_car = nullptr;
lv_obj_t *help_chip = nullptr;
lv_obj_t *lbl_help = nullptr;
lv_obj_t *batt_box = nullptr;
lv_obj_t *lbl_batt = nullptr;  // "NN%" text, sits left of the icon
lv_obj_t *batt_body = nullptr; // battery outline
lv_obj_t *batt_fill = nullptr; // proportional fill inside the outline
lv_obj_t *batt_tip = nullptr;  // little terminal nub
int batt_pct_last = -1;
lv_obj_t *help_overlay = nullptr;
bool help_open = false;

// save-name popup state (opened by 's' on a non-preset frequency)
bool save_open = false;
std::string save_name;
int save_slot = -1;
lv_obj_t *save_overlay = nullptr;
lv_obj_t *lbl_save_name = nullptr;
// custom 4-char labels for the user memory slots (16..20); empty == unnamed
char mem_label[NUM_MEM][5] = {};

lv_obj_t *lbl_entry = nullptr;
lv_obj_t *pointer_tri = nullptr;
lv_obj_t *ch_labels[3] = {}; // CH2 (left), CH1 (mid), CH0 (right)
bool ch2_quad = true;        // CH2 in quadrature (90 deg) vs in phase (0 deg)
lv_obj_t *ch_sep = nullptr;
lv_point_precise_t ch_sep_pts[2] = {};
lv_obj_t *tick_lines[NTICKS] = {};

// Little travelling sine glyph shown left of each CHx label. The icon spans
// exactly one wavelength, so advancing the phase makes the wave travel while
// its shape and width stay constant. Off channels show a flat line instead.
constexpr int CHW_W = 14;
constexpr int CHW_H = 10;
constexpr int CHW_N = 11;
constexpr float CHW_AMP = 3.0f;
lv_obj_t *ch_wave[3] = {};
lv_point_precise_t ch_wave_pts[3][CHW_N] = {};
bool ch_wave_active[3] = {};
float ch_wave_phase[3] = {};
lv_obj_t *tick_labels[NTICKS] = {};
lv_point_precise_t tick_pts[NTICKS][2] = {};

// Little sine-wave glyph for the carrier chip (drawn, not a font symbol).
constexpr int WAVE_W = 22;
constexpr int WAVE_H = 14;
constexpr int WAVE_N = 13;
constexpr float WAVE_PI = 3.14159265f;
lv_point_precise_t wave_pts[WAVE_N] = {};
lv_point_precise_t flat_pts[2] = {}; // flatline glyph shown when the carrier is OFF

// Little staircase glyph for the step chip (drawn, not a font symbol).
constexpr int STEP_W = 14;
constexpr int STEP_H = 10;
constexpr int STEP_N = 5;
lv_point_precise_t step_pts[STEP_N] = {};

// Battery icon geometry (fill spans BATT_FILL_MAX pixels at 100%).
constexpr int BATT_W = 20;
constexpr int BATT_H = 11;
constexpr int BATT_FILL_MAX = 15;

// ---- helpers --------------------------------------------------------------

// Draw the per-channel icon: one wavelength of sine at the current phase, or a
// flat line while the channel is disabled.
void ch_wave_render(int i) {
  for (int j = 0; j < CHW_N; j++) {
    float u = (float)j / (float)(CHW_N - 1);
    float y = (float)CHW_H / 2.0f;
    if (ch_wave_active[i]) {
      y -= CHW_AMP * sinf(2.0f * WAVE_PI * u - ch_wave_phase[i]);
    }
    ch_wave_pts[i][j].x = (lv_coord_t)(u * (float)CHW_W + 0.5f);
    ch_wave_pts[i][j].y = (lv_coord_t)(y + 0.5f);
  }
  lv_line_set_points(ch_wave[i], ch_wave_pts[i], CHW_N);
}

// Advance only the enabled channels so the active sines keep travelling.
void ch_wave_tick(lv_timer_t *) {
  for (int i = 0; i < 3; i++) {
    if (!ch_wave_active[i]) {
      continue;
    }
    ch_wave_phase[i] += 0.5f;
    if (ch_wave_phase[i] >= 2.0f * WAVE_PI) {
      ch_wave_phase[i] -= 2.0f * WAVE_PI;
    }
    ch_wave_render(i);
  }
}

void format_freq(int32_t f, char *buf, size_t n) {
  if (f < 0) {
    f = 0;
  }
  snprintf(buf, n, "%d.%03d.%03d", (int)(f / 1000000), (int)((f / 1000) % 1000),
           (int)(f % 1000));
}

// Group a string of digits with '.' every 3 digits from the right, e.g.
// "11000" -> "11.000" and "1100000" -> "1.100.000".
std::string group_digits(const std::string &digits) {
  std::string out;
  int n = (int)digits.size();
  for (int i = 0; i < n; i++) {
    if (i > 0 && ((n - i) % 3) == 0) {
      out.push_back('.');
    }
    out.push_back(digits[i]);
  }
  return out;
}

// Recover a bus that a slave may be holding low (e.g. after a warm reset in the
// middle of a transaction): clock SCL up to 9 times, then issue a STOP.
void i2c_bus_recover(int sda, int scl) {
  gpio_config_t io = {};
  io.pin_bit_mask = (1ULL << sda) | (1ULL << scl);
  io.mode = GPIO_MODE_INPUT_OUTPUT_OD;
  io.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io);
  gpio_set_level((gpio_num_t)scl, 1);
  gpio_set_level((gpio_num_t)sda, 1);
  esp_rom_delay_us(10);
  if (gpio_get_level((gpio_num_t)sda) == 0) {
    for (int i = 0; i < 9 && gpio_get_level((gpio_num_t)sda) == 0; i++) {
      gpio_set_level((gpio_num_t)scl, 0);
      esp_rom_delay_us(10);
      gpio_set_level((gpio_num_t)scl, 1);
      esp_rom_delay_us(10);
    }
    gpio_set_level((gpio_num_t)sda, 0);
    esp_rom_delay_us(10);
    gpio_set_level((gpio_num_t)scl, 1);
    esp_rom_delay_us(10);
    gpio_set_level((gpio_num_t)sda, 1);
    esp_rom_delay_us(10);
  }
}

void format_tick(int32_t f, int32_t minor, char *buf, size_t n) {
  (void)minor;
  // Dial tick labels are always shown in MHz with one decimal place, e.g. "7.0M".
  snprintf(buf, n, "%d.%dM", (int)(f / 1000000), (int)((f % 1000000) / 100000));
}

// Signed IF offset in MHz: '.' thousands grouping, ',' decimal (4 places), e.g.
// -455000 -> "-0,4550 MHz" and 45000000 -> "+45,0000 MHz".
void format_mhz_offset(int32_t hz, char *buf, size_t n) {
  char sign = (hz < 0) ? '-' : '+';
  uint32_t a = (uint32_t)(hz < 0 ? -(int64_t)hz : (int64_t)hz);
  uint32_t whole = a / 1000000u;
  uint32_t frac = (a / 100u) % 10000u; // 4 decimals -> 100 Hz resolution
  char grp[16];
  snprintf(grp, sizeof(grp), "%u", (unsigned)whole);
  snprintf(buf, n, "%c%s,%04u MHz", sign, group_digits(std::string(grp)).c_str(),
           (unsigned)frac);
}

// The LO actually runs at (display frequency + IF offset); the display never
// shows the offset, matching the reference VFO behaviour.
void apply_lo() {
  if (syn) {
    syn->set_frequency((uint32_t)(vfo.freq + vfo.if_offset), 0);
  }
}

// Carrier / BFO: CW is the bare carrier, USB/LSB shift it +/- 1500 Hz, OFF
// disables the carrier output entirely.
void apply_carrier() {
  if (!syn) {
    return;
  }
  int32_t f = vfo.car_freq;
  if (vfo.car_mode == 2) {
    f += SB_OFFSET_HZ;
  } else if (vfo.car_mode == 3) {
    f -= SB_OFFSET_HZ;
  }
  syn->set_carrier((uint32_t)f, vfo.car_mode != 0, ch2_quad);
}

void set_freq(int32_t f) {
  if (f < MIN_FREQ) {
    f = MIN_FREQ;
  }
  if (f > MAX_FREQ) {
    f = MAX_FREQ;
  }
  vfo.freq = f;
  apply_lo();
}

// Render the current numeric entry, grouped for plain frequencies and with an
// optional leading '-' preserved when editing the signed IF offset.
std::string entry_text() {
  const std::string &e = vfo.entry;
  if (vfo.entry_target == ENTRY_FREQ) {
    return group_digits(e);
  }
  if (!e.empty() && e[0] == '-') {
    return "-" + group_digits(e.substr(1));
  }
  return group_digits(e);
}

void update_scale() {
  int32_t minor = STEP_HZ[vfo.step_index];
  if (minor < 50000) {
    minor = 50000;  // minimum dial ruler so MHz labels stay on screen at fine steps
  }
  int32_t base = (vfo.freq / minor) * minor;
  int32_t major_interval = minor * 2;
  float phase = (float)(vfo.freq % minor) / (float)minor;
  int center = NTICKS / 2;
  for (int i = 0; i < NTICKS; i++) {
    int s = i - center;
    float x = (float)SCALE_CENTER_X + ((float)s - phase) * (float)SCALE_PITCH;
    int32_t tf = base + (int32_t)s * minor;
    bool major = (tf >= 0) && ((tf % major_interval) == 0);
    int bottom = major ? TICK_MAJOR_BOTTOM : TICK_MINOR_BOTTOM;

    tick_pts[i][0].x = (lv_coord_t)(x + 0.5f);
    tick_pts[i][0].y = TICK_TOP;
    tick_pts[i][1].x = (lv_coord_t)(x + 0.5f);
    tick_pts[i][1].y = bottom;
    lv_line_set_points(tick_lines[i], tick_pts[i], 2);

    if (major) {
      char buf[16];
      format_tick(tf, minor, buf, sizeof(buf));
      lv_label_set_text(tick_labels[i], buf);
      lv_obj_set_pos(tick_labels[i], (lv_coord_t)(x + 0.5f) - 26, TICK_LABEL_Y);
      lv_obj_remove_flag(tick_labels[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(tick_labels[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
}

const char *car_mode_name() {
  switch (vfo.car_mode) {
  case 1:
    return "CW";
  case 2:
    return "USB";
  case 3:
    return "LSB";
  default:
    return "OFF";
  }
}

// Colour of the carrier chip, per mode.
uint32_t car_mode_color() {
  switch (vfo.car_mode) {
  case 1:
    return 0x66CCFF; // CW - cyan
  case 2:
    return 0x33CC66; // USB - green
  case 3:
    return 0xFF8C1A; // LSB - orange
  default:
    return 0x808080; // OFF - grey
  }
}

void update_ui() {
  char buf[24];
  const char *prefix = "";
  if (vfo.entry_target == ENTRY_IF) {
    prefix = "IF";
  } else if (vfo.entry_target == ENTRY_CAR) {
    prefix = "CAR";
  }
  lv_label_set_text(lbl_prefix, prefix);
  if (vfo.entry_target != ENTRY_NONE) {
    std::string shown = entry_text();
    snprintf(buf, sizeof(buf), "%s_", shown.c_str());
    lv_label_set_text(lbl_freq, buf);
  } else {
    format_freq(vfo.freq, buf, sizeof(buf));
    lv_label_set_text(lbl_freq, buf);
  }
  lv_label_set_text(lbl_step, STEP_NAME[vfo.step_index]);
  // Memory chip: preset slots show the band alias, the rest the slot number.
  // A trailing '*' marks a populated slot the dial has moved away from.
  if (vfo.mem_index < NUM_PRESETS) {
    int32_t center = MEM_PRESET_HZ[vfo.mem_index];
    int32_t tol = center / 25; // ~4%
    if (tol < 10000) tol = 10000;
    if (tol > 50000) tol = 50000;
    if (llabs((long long)vfo.freq - (long long)center) <= tol) {
      lv_label_set_text(lbl_mem, MEM_ALIAS[vfo.mem_index]);
    } else {
      lv_label_set_text(lbl_mem, "---");
    }
  } else {
    bool mem_dirty = (vfo.memories[vfo.mem_index] != 0 &&
                      vfo.memories[vfo.mem_index] != vfo.freq);
    if (mem_label[vfo.mem_index][0] != '\0') {
      lv_label_set_text_fmt(lbl_mem, "%s%s", mem_label[vfo.mem_index],
                            mem_dirty ? "*" : " ");
    } else {
      lv_label_set_text_fmt(lbl_mem, "%02d%s", vfo.mem_index + 1,
                            mem_dirty ? "*" : " ");
    }
  }
  char off[32];
  format_mhz_offset(vfo.if_offset, off, sizeof(off));
  lv_label_set_text_fmt(lbl_aux, "IF %s", off);
  lv_label_set_text(lbl_car, car_mode_name());
  lv_color_t car_col = lv_color_hex(car_mode_color());
  lv_obj_set_style_line_color(wave_line, car_col, 0);
  lv_obj_set_style_line_color(flat_line, car_col, 0);
  lv_obj_set_style_text_color(lbl_car, car_col, 0);
  lv_obj_set_style_border_color(car_chip, car_col, 0);
  lv_obj_set_style_bg_color(car_chip, car_col, 0);
  lv_obj_set_style_bg_opa(car_chip, vfo.car_mode ? LV_OPA_20 : LV_OPA_TRANSP, 0);
  // OFF means no carrier, so show a flatline (dead signal) instead of a wave.
  if (vfo.car_mode == 0) {
    lv_obj_add_flag(wave_line, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(flat_line, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_remove_flag(wave_line, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(flat_line, LV_OBJ_FLAG_HIDDEN);
  }
  // Channel indicator: CH0 = LO (CLK0), CH1/CH2 = carrier I/Q (CLK1/CLK2).
  // Bright when the matching Si5351 output is enabled, dim when powered down.
  {
    lv_color_t ch_on = lv_color_hex(COL_STATUS);
    lv_color_t ch_off = lv_color_hex(0x404040);
    for (int i = 0; i < 3; i++) {
      uint8_t clk = static_cast<uint8_t>(2 - i); // CH2->CLK2 .. CH0->CLK0
      bool on = syn != nullptr && syn->clock_enabled(clk);
      lv_obj_set_style_text_color(ch_labels[i], on ? ch_on : ch_off, 0);
      lv_obj_set_style_line_color(ch_wave[i], on ? ch_on : ch_off, 0);
      ch_wave_active[i] = on;
      ch_wave_render(i); // switch sine <-> flatline immediately
    }
    // CH2 label shows (Q) only while it really is the quadrature output.
    lv_label_set_text(ch_labels[0], ch2_quad ? "CH2 (Q)" : "CH2");
  }
  update_scale();
}

// Battery indicator: read the pack voltage (GPIO 10 through the BSP's 2:1
// divider) and refresh the percent label and the fill of the battery icon.
// Called only occasionally since the state of charge changes slowly; hides the
// widget if the ADC is unreadable (e.g. no pack attached).
void update_battery() {
  auto &cp = espp::M5StackCardputer::get();
  float v = cp.battery_voltage();
  if (v < 0.5f) {
    lv_obj_add_flag(batt_box, LV_OBJ_FLAG_HIDDEN);
    batt_pct_last = -1;
    return;
  }
  lv_obj_remove_flag(batt_box, LV_OBJ_FLAG_HIDDEN);
  int pct = (int)(cp.battery_soc() + 0.5f);
  pct = pct < 0 ? 0 : (pct > 100 ? 100 : pct);
  if (pct == batt_pct_last) {
    return;
  }
  batt_pct_last = pct;
  lv_label_set_text_fmt(lbl_batt, "%d%%", pct);
  lv_obj_set_width(batt_fill, (lv_coord_t)(BATT_FILL_MAX * pct / 100));
  lv_color_t col = pct <= 15   ? lv_color_hex(0xFF3B1E)
                   : pct <= 40 ? lv_color_hex(0xFFCC66)
                               : lv_color_hex(0x5CD65C);
  lv_obj_set_style_bg_color(batt_fill, col, 0);
}

void memorize() {
  if (vfo.mem_index >= NUM_PRESETS) {
    vfo.memories[vfo.mem_index] = vfo.freq;
  } else {
    vfo.memories[vfo.mem_index] = MEM_PRESET_HZ[vfo.mem_index];
  }
}

void load_memory() {
  if (vfo.mem_index >= NUM_PRESETS) {
    if (vfo.memories[vfo.mem_index] != 0) {
      set_freq(vfo.memories[vfo.mem_index]);
    }
  } else {
    set_freq(MEM_PRESET_HZ[vfo.mem_index]);
  }
}

// True when the dial frequency sits close to one of the preset band centres,
// so pressing 's' can just store it without asking for a custom name.
bool freq_in_preset() {
  for (int i = 0; i < NUM_PRESETS; i++) {
    int32_t center = MEM_PRESET_HZ[i];
    int32_t tol = center / 25; // ~4%
    if (tol < 10000) tol = 10000;
    if (tol > 50000) tol = 50000;
    if (llabs((long long)vfo.freq - (long long)center) <= tol) {
      return true;
    }
  }
  return false;
}

// Next empty user memory slot (indices NUM_PRESETS..NUM_MEM-1), searching just
// after the current slot and wrapping; falls back to the next user slot when
// every one of them is already occupied.
int next_free_slot() {
  for (int k = 1; k <= NUM_MEM; k++) {
    int idx = (vfo.mem_index + k) % NUM_MEM;
    if (idx >= NUM_PRESETS && vfo.memories[idx] == 0) {
      return idx;
    }
  }
  for (int k = 1; k <= NUM_MEM; k++) {
    int idx = (vfo.mem_index + k) % NUM_MEM;
    if (idx >= NUM_PRESETS) {
      return idx;
    }
  }
  return NUM_PRESETS;
}

void refresh_save_name() {
  if (save_name.empty()) {
    lv_label_set_text(lbl_save_name, "_");
  } else {
    lv_label_set_text_fmt(lbl_save_name, "%s_", save_name.c_str());
  }
}

void close_save_popup() {
  save_open = false;
  lv_obj_add_flag(save_overlay, LV_OBJ_FLAG_HIDDEN);
}

void open_save_popup() {
  save_open = true;
  save_name.clear();
  save_slot = next_free_slot();
  refresh_save_name();
  lv_obj_remove_flag(save_overlay, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(save_overlay);
}

// Store the dial frequency plus its 4-char label into the chosen user slot.
void commit_save() {
  if (save_slot >= NUM_PRESETS && !save_name.empty()) {
    vfo.memories[save_slot] = vfo.freq;
    snprintf(mem_label[save_slot], sizeof(mem_label[save_slot]), "%s",
             save_name.c_str());
    vfo.mem_index = save_slot;
  }
  close_save_popup();
  update_ui();
}

// Commit the number currently in the entry buffer to whatever is being edited.
void apply_entry() {
  if (vfo.entry.empty() || vfo.entry == "-") {
    vfo.entry_target = ENTRY_NONE;
    return;
  }
  int32_t val = (int32_t)strtol(vfo.entry.c_str(), nullptr, 10);
  switch (vfo.entry_target) {
  case ENTRY_FREQ:
    set_freq(val);
    break;
  case ENTRY_IF:
    vfo.if_offset = val;
    apply_lo();
    break;
  case ENTRY_CAR:
    if (val < CAR_MIN_FREQ) {
      val = CAR_MIN_FREQ;
    }
    if (val > CAR_MAX_FREQ) {
      val = CAR_MAX_FREQ;
    }
    vfo.car_freq = val;
    apply_carrier();
    break;
  default:
    break;
  }
  vfo.entry_target = ENTRY_NONE;
}

// ---- key handling ---------------------------------------------------------

// Distinct click tone per key. Notes follow a C-major pentatonic scale so any
// combination of successive key presses sounds pleasant; each key maps to its
// own step so the pitch is unique and recognisable.
float tone_hz_for_key(char c, uint8_t special) {
  using SK = espp::M5StackCardputer::SpecialKey;
  int idx;
  if (c >= '0' && c <= '9') {
    idx = c - '0'; // 0..9
  } else {
    switch (c) {
    case ',': idx = 10; break;
    case '/': idx = 11; break;
    case ';': idx = 12; break;
    case '.': idx = 13; break;
    case 'm': idx = 14; break;
    case 'n': idx = 15; break;
    case 's': idx = 16; break;
    case 'c': idx = 17; break;
    case 'o': idx = 18; break;
    case 'i': idx = 19; break;
    case 'p': idx = 20; break;
    case 't': idx = 21; break;
    case 'h': idx = 22; break;
    case '\n': idx = 23; break;
    case '\b': idx = 24; break;
    case '-': idx = 25; break;
    case '`': idx = 26; break;
    default:
      // value 0 means "no character"; fall back to the special key so
      // Fn+arrows / Esc still get their own tone.
      switch (static_cast<SK>(special)) {
      case SK::LEFT: idx = 10; break;
      case SK::RIGHT: idx = 11; break;
      case SK::UP: idx = 12; break;
      case SK::DOWN: idx = 13; break;
      case SK::ESC: idx = 26; break;
      default: idx = 27; break;
      }
      break;
    }
  }
  // Each key gets its own pitch, climbing chromatically from A4 (440 Hz).
  // The Cardputer's NS4168 speaker has almost no output below ~400 Hz and gets
  // harsh above ~4 kHz, so the range is kept in that band: every key is
  // distinct and clearly audible.
  if (idx < 0) {
    idx = 0;
  }
  if (idx > 27) {
    idx = 27;
  }
  return 440.0f * std::pow(2.0f, idx / 12.0f); // 440 Hz .. ~2.1 kHz
}

// Synthesise a short sine tone with a small attack/release envelope (removes
// the click you otherwise get from abruptly starting/stopping a sample stream)
// and hand it to the BSP's non-blocking I2S player.
void play_tone(float freq_hz, uint16_t ms) {
  auto &cp = espp::M5StackCardputer::get();
  const uint32_t sr = cp.audio_sample_rate();
  if (sr == 0 || cp.is_muted()) {
    return;
  }
  static int16_t buf[44100 * 80 / 1000]; // up to 80 ms @ 44.1 kHz
  size_t n = static_cast<size_t>(sr) * ms / 1000;
  const size_t max_n = sizeof(buf) / sizeof(buf[0]);
  if (n > max_n) {
    n = max_n;
  }
  const float two_pi = 6.2831853f;
  const float amp = 0.6f * 32767.0f;
  const size_t ramp = sr / 250; // ~4 ms attack / release
  if (ramp == 0) {
    return;
  }
  for (size_t i = 0; i < n; i++) {
    float env = 1.0f;
    if (i < ramp) {
      env = static_cast<float>(i) / static_cast<float>(ramp);
    } else if (i + ramp > n) {
      env = static_cast<float>(n - i) / static_cast<float>(ramp);
    }
    buf[i] = static_cast<int16_t>(amp * env *
                                  std::sin(two_pi * freq_hz * i / sr));
  }
  cp.play_audio(reinterpret_cast<const uint8_t *>(buf),
                static_cast<uint32_t>(n * sizeof(int16_t)));
}

void handle_key(const KeyMsg &msg) {
  play_tone(tone_hz_for_key(msg.value, msg.special), 35);
  char c = msg.value;
  auto special = static_cast<espp::M5StackCardputer::SpecialKey>(msg.special);
  using SK = espp::M5StackCardputer::SpecialKey;

  // Help popup: 'h' toggles it, Esc closes it. While open, swallow other keys.
  if (help_open) {
    if (special == SK::ESC || c == '`' || c == 'h') {
      help_open = false;
      lv_obj_add_flag(help_overlay, LV_OBJ_FLAG_HIDDEN);
    }
    return;
  }
  if (c == 'h') {
    help_open = true;
    lv_obj_remove_flag(help_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(help_overlay);
    return;
  }

  // save-name popup: collect up to 4 alphanumerics (shown uppercase),
  // Enter = store into the next free user slot, Esc/` = cancel.
  if (save_open) {
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
        (c >= 'a' && c <= 'z')) {
      if (save_name.size() < 4) {
        save_name.push_back((char)toupper((unsigned char)c));
      }
    } else if (c == '\b') {
      if (!save_name.empty()) {
        save_name.pop_back();
      }
    } else if (c == '\n') {
      commit_save();
      return;
    } else if (special == SK::ESC || c == '`') {
      close_save_popup();
      return;
    }
    refresh_save_name();
    return;
  }

  // Fn + arrows report value 0 with a special key; map them onto the same
  // characters as the (unshifted) , . ; / keys so both paths tune/step.
  if (c == 0) {
    switch (special) {
    case SK::LEFT:
      c = ',';
      break;
    case SK::RIGHT:
      c = '/';
      break;
    case SK::UP:
      c = ';';
      break;
    case SK::DOWN:
      c = '.';
      break;
    default:
      break;
    }
  }

  // numeric-entry mode: gather digits (plus a leading '-' for the IF offset)
  if (vfo.entry_target != ENTRY_NONE) {
    if (c >= '0' && c <= '9') {
      if (vfo.entry.size() < 10) {
        vfo.entry.push_back(c);
      }
    } else if (c == '-' && vfo.entry_target == ENTRY_IF && vfo.entry.empty()) {
      vfo.entry.push_back('-');
    } else if (c == '\b') {
      if (!vfo.entry.empty()) {
        vfo.entry.pop_back();
      }
    } else if (c == '\n') {
      apply_entry();
    } else if (special == SK::ESC || c == '`') {
      vfo.entry_target = ENTRY_NONE;
    }
    update_ui();
    return;
  }

  switch (c) {
  case ',': // LEFT
    set_freq(vfo.freq - STEP_HZ[vfo.step_index]);
    break;
  case '/': // RIGHT
    set_freq(vfo.freq + STEP_HZ[vfo.step_index]);
    break;
  case ';': // UP
    if (vfo.step_index < NUM_STEPS - 1) {
      vfo.step_index++;
    }
    break;
  case '.': // DOWN
    if (vfo.step_index > 0) {
      vfo.step_index--;
    }
    break;
  case 'm':
    vfo.mem_index = (vfo.mem_index + 1) % NUM_MEM;
    load_memory();
    break;
  case 'n':
    vfo.mem_index = (vfo.mem_index + NUM_MEM - 1) % NUM_MEM;
    load_memory();
    break;
  case 's':
    if (freq_in_preset()) {
      memorize();
    } else {
      open_save_popup();
      return; // popup draws itself; refresh happens on Enter/Esc
    }
    break;
  case 'c': // cycle carrier: OFF -> USB -> LSB -> OFF
    vfo.car_mode = (vfo.car_mode + 1) % 4;
    apply_carrier();
    break;
  case 'o': // edit the IF offset (Hz, signed)
    vfo.entry_target = ENTRY_IF;
    vfo.entry.clear();
    break;
  case 'i': // edit the carrier / BFO centre frequency
    vfo.entry_target = ENTRY_CAR;
    vfo.entry.clear();
    break;
  case 'p': // toggle CH2 phase: 90 deg (quadrature) <-> 0 deg (in phase)
    ch2_quad = !ch2_quad;
    apply_carrier();
    break;
  case 't': // toggle dark <-> light theme by inverting the panel colors
    theme_light = !theme_light;
    espp::M5StackCardputer::get().set_panel_invert(theme_light);
    break;
  default:
    if (c >= '0' && c <= '9') {
      vfo.entry_target = ENTRY_FREQ;
      vfo.entry.clear();
      vfo.entry.push_back(c);
    } else {
      return; // nothing to update
    }
    break;
  }
  update_ui();
}

// ---- keyboard callback (runs in the scanner task) -------------------------

void on_key(const espp::M5StackCardputer::KeyEvent &e) {
  if (!e.pressed || key_queue == nullptr) {
    return;
  }
  KeyMsg msg{.value = e.value, .special = (uint8_t)e.special, .shift = e.modifiers.shift};
  xQueueSend(key_queue, &msg, 0);
}

} // namespace

extern "C" void app_main(void) {
  auto &cardputer = espp::M5StackCardputer::get();

  if (!cardputer.initialize_lcd()) {
    printf("Gagal initialize_lcd!\n");
    return;
  }
  static constexpr size_t pixel_buffer_size = cardputer.lcd_width() * 50;
  if (!cardputer.initialize_display(pixel_buffer_size)) {
    printf("Gagal initialize_display!\n");
    return;
  }
  cardputer.brightness(100.0f);

  if (!cardputer.initialize_led()) {
    printf("Gagal initialize_led!\n");
    return;
  }
  cardputer.led(espp::Hsv(30.0f, 1.0f, 0.5f)); // warm dial-lamp amber

  // Speaker for the per-key click tones (safe: the microphone is not used, so
  // there is no GPIO 43 conflict with the NS4168 I2S amp).
  if (!cardputer.initialize_sound()) {
    printf("Gagal initialize_sound (tone)!\n");
  } else {
    cardputer.volume(60.0f);
  }

  // Force the BSP's variant detection to run now: it briefly opens its internal
  // I2C bus on GPIO 8/9 (I2C port 0), then releases the port. Doing this before
  // we claim I2C port 0 for the Grove connector avoids a bus-acquire conflict.
  cardputer.variant();

  // Free the Grove bus in case a previous run/reset left the Si5351 holding SDA
  // low (which would NACK every transaction until a full power cycle).
  i2c_bus_recover(espp::M5StackCardputer::grove_sda_gpio(),
                  espp::M5StackCardputer::grove_scl_gpio());

  // --- software key queue (callback -> main loop hand-off) ---
  key_queue = xQueueCreate(16, sizeof(KeyMsg));

  // --- I2C + Si5351 ---
  espp::I2c i2c({
      .port = I2C_NUM_0,
      .sda_io_num = espp::M5StackCardputer::grove_sda_gpio(),
      .scl_io_num = espp::M5StackCardputer::grove_scl_gpio(),
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .clk_speed = 100000,
  });

  static Si5351 synth(i2c, 0x60);
  syn = &synth;
  bool si_ok = i2c.initialized() && i2c.probe_device(0x60) && synth.init();

  // preset memories: band centre frequencies, shown by alias
  for (int i = 0; i < NUM_PRESETS; i++) {
    vfo.memories[i] = MEM_PRESET_HZ[i];
  }
  vfo.mem_index = 2;                 // 40 m
  vfo.freq = MEM_PRESET_HZ[2];

  // --- build UI ---
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // channel indicator row across the very top: CH2 (left) CH1 (mid) CH0 (right),
  // each preceded by a travelling sine glyph, followed by a separator line.
  // Bright + moving = output enabled, dim + flat = disabled.
  {
    const char *ch_text[3] = {"CH2 (Q)", "CH1 (I)", "CH0"};
    const lv_align_t ch_align[3] = {LV_ALIGN_TOP_LEFT, LV_ALIGN_TOP_MID,
                                    LV_ALIGN_TOP_RIGHT};
    const lv_coord_t ch_dx[3] = {6, 0, -6};
    for (int i = 0; i < 3; i++) {
      lv_obj_t *box = lv_obj_create(scr);
      lv_obj_set_size(box, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
      lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
      lv_obj_set_style_bg_opa(box, LV_OPA_TRANSP, 0);
      lv_obj_set_style_border_width(box, 0, 0);
      lv_obj_set_style_pad_all(box, 0, 0);
      lv_obj_set_style_pad_column(box, 3, 0);
      lv_obj_set_flex_flow(box, LV_FLEX_FLOW_ROW);
      lv_obj_set_flex_align(box, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                            LV_FLEX_ALIGN_CENTER);
      lv_obj_align(box, ch_align[i], ch_dx[i], 0);

      ch_wave[i] = lv_line_create(box);
      lv_obj_set_size(ch_wave[i], CHW_W, CHW_H);
      lv_obj_set_style_line_width(ch_wave[i], 1, 0);
      lv_obj_set_style_line_color(ch_wave[i], lv_color_hex(0x404040), 0);
      ch_wave_render(i);

      ch_labels[i] = lv_label_create(box);
      lv_obj_set_style_text_font(ch_labels[i], &lv_font_montserrat_14, 0);
      lv_label_set_text(ch_labels[i], ch_text[i]);
    }
    ch_sep_pts[0] = {0, 0};
    ch_sep_pts[1] = {W, 0};
    ch_sep = lv_line_create(scr);
    lv_obj_set_style_line_width(ch_sep, 1, 0);
    lv_obj_set_style_line_color(ch_sep, lv_color_hex(COL_STATUS), 0);
    lv_obj_set_style_line_opa(ch_sep, LV_OPA_40, 0);
    lv_line_set_points(ch_sep, ch_sep_pts, 2);
    lv_obj_align(ch_sep, LV_ALIGN_TOP_LEFT, 0, CH_SEP_Y);

    lv_timer_create(ch_wave_tick, 60, nullptr);
  }

  // step-size badge (pill chip): [staircase glyph] 1kHz
  step_chip = lv_obj_create(scr);
  lv_obj_set_size(step_chip, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_clear_flag(step_chip, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(step_chip, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(step_chip, 1, 0);
  lv_obj_set_style_border_color(step_chip, lv_color_hex(COL_STATUS), 0);
  lv_obj_set_style_bg_opa(step_chip, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_hor(step_chip, 6, 0);
  lv_obj_set_style_pad_ver(step_chip, 0, 0);
  lv_obj_set_style_pad_column(step_chip, 3, 0);
  lv_obj_set_flex_flow(step_chip, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(step_chip, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_align(step_chip, LV_ALIGN_TOP_LEFT, 4, 2 + TOP_SHIFT);

  step_pts[0] = {0, 9};
  step_pts[1] = {0, 5};
  step_pts[2] = {7, 5};
  step_pts[3] = {7, 1};
  step_pts[4] = {14, 1};
  step_line = lv_line_create(step_chip);
  lv_obj_set_size(step_line, STEP_W, STEP_H);
  lv_line_set_points(step_line, step_pts, STEP_N);
  lv_obj_set_style_line_width(step_line, 2, 0);
  lv_obj_set_style_line_color(step_line, lv_color_hex(COL_STATUS), 0);

  lbl_step = lv_label_create(step_chip);
  lv_obj_set_style_text_font(lbl_step, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_step, lv_color_hex(COL_STATUS), 0);

  // memory-slot badge (pill chip): [floppy] 01*
  mem_chip = lv_obj_create(scr);
  lv_obj_set_size(mem_chip, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_clear_flag(mem_chip, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(mem_chip, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(mem_chip, 1, 0);
  lv_obj_set_style_border_color(mem_chip, lv_color_hex(COL_STATUS), 0);
  lv_obj_set_style_bg_opa(mem_chip, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_hor(mem_chip, 6, 0);
  lv_obj_set_style_pad_ver(mem_chip, 0, 0);
  lv_obj_set_style_pad_column(mem_chip, 3, 0);
  lv_obj_set_flex_flow(mem_chip, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(mem_chip, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_align(mem_chip, LV_ALIGN_TOP_RIGHT, -4, 2 + TOP_SHIFT);

  lbl_mem_icon = lv_label_create(mem_chip);
  lv_obj_set_style_text_font(lbl_mem_icon, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_mem_icon, lv_color_hex(COL_STATUS), 0);
  lv_label_set_text(lbl_mem_icon, LV_SYMBOL_SAVE);

  lbl_mem = lv_label_create(mem_chip);
  lv_obj_set_style_text_font(lbl_mem, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_mem, lv_color_hex(COL_STATUS), 0);

  // carrier / BFO badge (pill chip): [icon] USB|LSB|OFF, colour-coded
  car_chip = lv_obj_create(scr);
  lv_obj_set_size(car_chip, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_clear_flag(car_chip, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(car_chip, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(car_chip, 1, 0);
  lv_obj_set_style_pad_hor(car_chip, 6, 0);
  lv_obj_set_style_pad_ver(car_chip, 0, 0);
  lv_obj_set_style_pad_column(car_chip, 3, 0);
  lv_obj_set_flex_flow(car_chip, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(car_chip, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_align(car_chip, LV_ALIGN_TOP_MID, 0, 2 + TOP_SHIFT);

  for (int i = 0; i < WAVE_N; i++) {
    float t = (float)i / (float)(WAVE_N - 1);
    wave_pts[i].x = (lv_coord_t)(t * (float)WAVE_W + 0.5f);
    wave_pts[i].y =
        (lv_coord_t)((float)WAVE_H / 2.0f - 4.0f * sinf(2.0f * WAVE_PI * t) + 0.5f);
  }
  wave_line = lv_line_create(car_chip);
  lv_obj_set_size(wave_line, WAVE_W, WAVE_H);
  lv_line_set_points(wave_line, wave_pts, WAVE_N);
  lv_obj_set_style_line_width(wave_line, 2, 0);

  // flatline glyph: a dead/empty signal, shown when the carrier is OFF
  flat_pts[0] = {6, WAVE_H / 2};
  flat_pts[1] = {16, WAVE_H / 2};
  flat_line = lv_line_create(car_chip);
  lv_obj_set_size(flat_line, WAVE_W, WAVE_H);
  lv_line_set_points(flat_line, flat_pts, 2);
  lv_obj_set_style_line_width(flat_line, 2, 0);
  lv_obj_add_flag(flat_line, LV_OBJ_FLAG_HIDDEN);

  lbl_car = lv_label_create(car_chip);
  lv_obj_set_style_text_font(lbl_car, &lv_font_montserrat_14, 0);
  lv_label_set_text(lbl_car, "OFF");

  // frequency readout: big digits + a half-size "Hz" unit sat on the baseline
  freq_row = lv_obj_create(scr);
  lv_obj_set_size(freq_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(freq_row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(freq_row, 0, 0);
  lv_obj_set_style_pad_all(freq_row, 0, 0);
  lv_obj_set_style_pad_column(freq_row, 4, 0);
  lv_obj_clear_flag(freq_row, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(freq_row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(freq_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
  lv_obj_align(freq_row, LV_ALIGN_TOP_MID, 0, 24 + TOP_SHIFT);

  lbl_prefix = lv_label_create(freq_row);
  lv_obj_set_style_text_font(lbl_prefix, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_prefix, lv_color_hex(COL_STATUS), 0);
  lv_label_set_text(lbl_prefix, "");

  lbl_freq = lv_label_create(freq_row);
  lv_obj_set_style_text_font(lbl_freq, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(lbl_freq, lv_color_hex(COL_READOUT), 0);

  lbl_unit = lv_label_create(freq_row);
  lv_obj_set_style_text_font(lbl_unit, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_unit, lv_color_hex(COL_READOUT), 0);
  lv_label_set_text(lbl_unit, "Hz");

  // sliding analog scale
  for (int i = 0; i < NTICKS; i++) {
    tick_lines[i] = lv_line_create(scr);
    lv_obj_set_style_line_width(tick_lines[i], 2, 0);
    lv_obj_set_style_line_color(
        tick_lines[i], lv_color_hex(((i - NTICKS / 2) % 5 == 0) ? COL_TICK_MAJOR : COL_TICK_MINOR),
        0);
    tick_labels[i] = lv_label_create(scr);
    lv_obj_set_style_text_font(tick_labels[i], &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(tick_labels[i], lv_color_hex(COL_TICK_MAJOR), 0);
    lv_obj_set_style_text_align(tick_labels[i], LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(tick_labels[i], LV_LABEL_LONG_CLIP);
    lv_obj_set_width(tick_labels[i], 52);
    lv_obj_add_flag(tick_labels[i], LV_OBJ_FLAG_HIDDEN);
  }

  // red triangle marker (points up) at the current-frequency position,
  // like a speedometer needle; drawn into a canvas so the color is exact
  static lv_draw_buf_t *pointer_buf =
      lv_draw_buf_create(POINTER_W, POINTER_H, LV_COLOR_FORMAT_ARGB8888, 0);
  pointer_tri = lv_canvas_create(scr);
  lv_canvas_set_draw_buf(pointer_tri, pointer_buf);
  lv_obj_clear_flag(pointer_tri, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(pointer_tri, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(pointer_tri, LV_ALIGN_TOP_LEFT, SCALE_CENTER_X - POINTER_W / 2,
               TICK_MAJOR_BOTTOM + 1 - POINTER_H);
  lv_canvas_fill_bg(pointer_tri, lv_color_black(), LV_OPA_TRANSP);
  lv_color_t tri_red = lv_color_hex(0xFF0000);
  for (int y = 0; y < POINTER_H; y++) {
    int half = (POINTER_W / 2) * y / (POINTER_H - 1);
    for (int x = POINTER_W / 2 - half; x <= POINTER_W / 2 + half; x++) {
      if (x >= 0 && x < POINTER_W) {
        lv_canvas_set_px(pointer_tri, x, y, tri_red, LV_OPA_COVER);
      }
    }
  }

  // bottom status bar: full-width footer with IF offset (left) and key hint (right)
  status_bar = lv_obj_create(scr);
  lv_obj_set_size(status_bar, W, FOOTER_H);
  lv_obj_clear_flag(status_bar, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(status_bar, 0, 0);
  lv_obj_set_style_bg_color(status_bar, lv_color_hex(COL_FOOTER_BG), 0);
  lv_obj_set_style_bg_opa(status_bar, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(status_bar, 1, 0);
  lv_obj_set_style_border_side(status_bar, LV_BORDER_SIDE_TOP, 0);
  lv_obj_set_style_border_color(status_bar, lv_color_hex(COL_STATUS), 0);
  lv_obj_set_style_border_opa(status_bar, LV_OPA_30, 0);
  lv_obj_set_style_pad_hor(status_bar, 5, 0);
  lv_obj_set_style_pad_ver(status_bar, 0, 0);
  lv_obj_align(status_bar, LV_ALIGN_BOTTOM_MID, 0, 0);

  lbl_aux = lv_label_create(status_bar);
  lv_obj_set_style_text_font(lbl_aux, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_aux, lv_color_hex(COL_STATUS), 0);
  lv_obj_align(lbl_aux, LV_ALIGN_LEFT_MID, 0, 0);

  help_chip = lv_obj_create(status_bar);
  lv_obj_set_size(help_chip, 18, 18);
  lv_obj_clear_flag(help_chip, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(help_chip, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(help_chip, 1, 0);
  lv_obj_set_style_border_color(help_chip, lv_color_hex(COL_STATUS), 0);
  lv_obj_set_style_bg_opa(help_chip, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(help_chip, 0, 0);
  lv_obj_align(help_chip, LV_ALIGN_RIGHT_MID, 0, 0);
  lbl_help = lv_label_create(help_chip);
  lv_obj_set_style_text_font(lbl_help, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_help, lv_color_hex(COL_STATUS), 0);
  lv_label_set_text(lbl_help, "?");
  lv_obj_center(lbl_help);

  // battery indicator: percent text left of a battery icon whose fill tracks
  // the state of charge, placed just left of the help chip
  batt_box = lv_obj_create(status_bar);
  lv_obj_set_size(batt_box, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_clear_flag(batt_box, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(batt_box, 0, 0);
  lv_obj_set_style_border_width(batt_box, 0, 0);
  lv_obj_set_style_bg_opa(batt_box, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(batt_box, 0, 0);
  lv_obj_set_style_pad_column(batt_box, 3, 0);
  lv_obj_set_flex_flow(batt_box, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(batt_box, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  // Right-align at a fixed offset that leaves room for the help chip (18px +
  // 5px bar pad) and the icon's terminal nub, so the label grows leftwards.
  lv_obj_align(batt_box, LV_ALIGN_RIGHT_MID, -26, 0);

  lbl_batt = lv_label_create(batt_box);
  lv_obj_set_style_text_font(lbl_batt, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_batt, lv_color_hex(COL_STATUS), 0);
  lv_label_set_text(lbl_batt, "100%");

  batt_body = lv_obj_create(batt_box);
  lv_obj_set_size(batt_body, BATT_W, BATT_H);
  lv_obj_clear_flag(batt_body, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(batt_body, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_set_style_radius(batt_body, 2, 0);
  lv_obj_set_style_border_width(batt_body, 1, 0);
  lv_obj_set_style_border_color(batt_body, lv_color_hex(COL_STATUS), 0);
  lv_obj_set_style_bg_opa(batt_body, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(batt_body, 0, 0);

  batt_tip = lv_obj_create(batt_body);
  lv_obj_set_size(batt_tip, 2, 5);
  lv_obj_clear_flag(batt_tip, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(batt_tip, 0, 0);
  lv_obj_set_style_border_width(batt_tip, 0, 0);
  lv_obj_set_style_bg_color(batt_tip, lv_color_hex(COL_STATUS), 0);
  lv_obj_set_style_bg_opa(batt_tip, LV_OPA_COVER, 0);
  lv_obj_align(batt_tip, LV_ALIGN_RIGHT_MID, 3, 0);

  batt_fill = lv_obj_create(batt_body);
  lv_obj_set_size(batt_fill, 0, 7);
  lv_obj_clear_flag(batt_fill, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(batt_fill, 1, 0);
  lv_obj_set_style_border_width(batt_fill, 0, 0);
  lv_obj_set_style_bg_opa(batt_fill, LV_OPA_COVER, 0);
  lv_obj_align(batt_fill, LV_ALIGN_LEFT_MID, 1, 0);

  // help popup: dim the screen and show a panel of shortcuts ('h' to toggle,
  // Esc to close). Created hidden and raised to the foreground when opened.
  help_overlay = lv_obj_create(scr);
  lv_obj_set_size(help_overlay, W, H);
  lv_obj_clear_flag(help_overlay, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(help_overlay, 0, 0);
  lv_obj_set_style_border_width(help_overlay, 0, 0);
  lv_obj_set_style_bg_color(help_overlay, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(help_overlay, LV_OPA_70, 0);
  lv_obj_set_style_pad_all(help_overlay, 0, 0);
  lv_obj_set_flex_flow(help_overlay, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(help_overlay, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_align(help_overlay, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_flag(help_overlay, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *help_panel = lv_obj_create(help_overlay);
  lv_obj_set_width(help_panel, LV_PCT(96));
  lv_obj_set_height(help_panel, LV_SIZE_CONTENT);
  lv_obj_clear_flag(help_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(help_panel, 6, 0);
  lv_obj_set_style_border_width(help_panel, 1, 0);
  lv_obj_set_style_border_color(help_panel, lv_color_hex(COL_STATUS), 0);
  lv_obj_set_style_bg_color(help_panel, lv_color_hex(COL_FOOTER_BG), 0);
  lv_obj_set_style_bg_opa(help_panel, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_hor(help_panel, 6, 0);
  lv_obj_set_style_pad_ver(help_panel, 3, 0);

  // lay it out as a grid: 4 columns (15% / 35% / 15% / 35%); the first two
  // rows only use the first key column and let the value span the rest.
  lv_obj_set_layout(help_panel, LV_LAYOUT_GRID);
  static lv_coord_t help_cols[] = {LV_GRID_FR(3), LV_GRID_FR(7), LV_GRID_FR(3),
                                   LV_GRID_FR(7), LV_GRID_TEMPLATE_LAST};
  static lv_coord_t help_rows[] = {
      LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT,
      LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT,
      LV_GRID_TEMPLATE_LAST};
  lv_obj_set_grid_dsc_array(help_panel, help_cols, help_rows);
  lv_obj_set_style_pad_row(help_panel, 0, 0);
  lv_obj_set_style_pad_column(help_panel, 4, 0);

  auto help_cell = [&](const char *txt, int col, int span, int row) {
    lv_obj_t *l = lv_label_create(help_panel);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(COL_STATUS), 0);
    lv_label_set_text(l, txt);
    lv_obj_set_grid_cell(l, LV_GRID_ALIGN_STRETCH, col, span, LV_GRID_ALIGN_CENTER, row, 1);
  };
  help_cell("0-9", 0, 1, 0);
  help_cell("input freq", 1, 3, 0);
  help_cell("c", 0, 1, 1);
  help_cell("carrier OFF/CW/USB/LSB", 1, 3, 1);
  help_cell(", /", 0, 1, 2);
  help_cell("tune -/+", 1, 1, 2);
  help_cell("; .", 2, 1, 2);
  help_cell("step +/-", 3, 1, 2);
  help_cell("m n", 0, 1, 3);
  help_cell("mem -/+", 1, 1, 3);
  help_cell("s", 2, 1, 3);
  help_cell("save", 3, 1, 3);
  help_cell("o", 0, 1, 4);
  help_cell("IF offset", 1, 1, 4);
  help_cell("i", 2, 1, 4);
  help_cell("car freq", 3, 1, 4);
  help_cell("h", 0, 1, 5);
  help_cell("help", 1, 1, 5);
  help_cell("Esc", 2, 1, 5);
  help_cell("close", 3, 1, 5);
  help_cell("p", 0, 1, 6);
  help_cell("CH2 phase 90/0", 1, 3, 6);
  help_cell("t", 0, 1, 7);
  help_cell("theme dark/light", 1, 3, 7);

  // save-name popup: dim the screen and prompt for a 4-char label (Enter to
  // store into the next free user memory slot, Esc to cancel). Created hidden.
  save_overlay = lv_obj_create(scr);
  lv_obj_set_size(save_overlay, W, H);
  lv_obj_clear_flag(save_overlay, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(save_overlay, 0, 0);
  lv_obj_set_style_border_width(save_overlay, 0, 0);
  lv_obj_set_style_bg_color(save_overlay, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(save_overlay, LV_OPA_70, 0);
  lv_obj_set_style_pad_all(save_overlay, 0, 0);
  lv_obj_set_flex_flow(save_overlay, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(save_overlay, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(save_overlay, 8, 0);
  lv_obj_align(save_overlay, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_flag(save_overlay, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *save_panel = lv_obj_create(save_overlay);
  lv_obj_set_size(save_panel, LV_PCT(86), LV_SIZE_CONTENT);
  lv_obj_clear_flag(save_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(save_panel, 6, 0);
  lv_obj_set_style_border_width(save_panel, 1, 0);
  lv_obj_set_style_border_color(save_panel, lv_color_hex(COL_STATUS), 0);
  lv_obj_set_style_bg_color(save_panel, lv_color_hex(COL_FOOTER_BG), 0);
  lv_obj_set_style_bg_opa(save_panel, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_all(save_panel, 8, 0);
  lv_obj_set_style_pad_row(save_panel, 4, 0);
  lv_obj_set_flex_flow(save_panel, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(save_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_t *save_title = lv_label_create(save_panel);
  lv_obj_set_style_text_font(save_title, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(save_title, lv_color_hex(COL_STATUS), 0);
  lv_label_set_text(save_title, "SAVE MEMORY");

  lbl_save_name = lv_label_create(save_panel);
  lv_obj_set_style_text_font(lbl_save_name, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(lbl_save_name, lv_color_hex(COL_READOUT), 0);
  lv_obj_set_style_text_align(lbl_save_name, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(lbl_save_name, "_");

  lv_obj_t *save_hint = lv_label_create(save_panel);
  lv_obj_set_style_text_font(save_hint, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(save_hint, lv_color_hex(COL_STATUS), 0);
  lv_label_set_text(save_hint, "Enter=save  Esc=cancel");

  // --- keyboard ---
  cardputer.initialize_keyboard(on_key);

  // --- push the initial LO (and disable the carrier) and draw everything ---
  if (si_ok) {
    apply_lo();
    apply_carrier();
  }
  update_battery();
  update_ui();

  // --- main loop: service LVGL and process key events ---
  uint32_t last_log = 0;
  uint32_t last_si_retry = 0;
  uint32_t last_batt = 0;
  while (true) {
    lv_timer_handler();
    KeyMsg msg;
    while (key_queue && xQueueReceive(key_queue, &msg, 0) == pdTRUE) {
      handle_key(msg);
    }
    uint32_t now = xTaskGetTickCount();
    // The first I2C transaction after boot can time out before the Si5351 bus
    // settles; keep re-initialising until it answers, then push the initial LO
    // and carrier so the CH0 indicator comes up on its own.
    if (!si_ok && (now - last_si_retry) > pdMS_TO_TICKS(500)) {
      last_si_retry = now;
      if (synth.init()) {
        si_ok = true;
        apply_lo();
        apply_carrier();
        update_ui();
      }
    }
    if (now - last_batt > pdMS_TO_TICKS(2000)) {
      last_batt = now;
      update_battery();
    }
    if (now - last_log > pdMS_TO_TICKS(3000)) {
      last_log = now;
      printf("VFO %ld Hz  step=%s  mem=%02d  IF%+ld  car=%s@%ld  si5351=%s  "
             "batt=%.2fV\n",
             (long)vfo.freq, STEP_NAME[vfo.step_index], vfo.mem_index + 1,
             (long)vfo.if_offset, car_mode_name(), (long)vfo.car_freq,
             si_ok ? "OK" : "FAIL", cardputer.battery_voltage());
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
