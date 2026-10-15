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
//   LO output = displayed frequency + IF offset; carrier I/Q on CLK1/CLK2.
// ---------------------------------------------------------------------------

namespace {

constexpr int32_t MIN_FREQ = 1000000;    // 1 MHz
constexpr int32_t MAX_FREQ = 150000000;  // 150 MHz
constexpr int NUM_STEPS = 7;
constexpr int32_t STEP_HZ[NUM_STEPS] = {10, 100, 1000, 10000, 100000, 1000000, 10000000};
constexpr const char *STEP_NAME[NUM_STEPS] = {"10Hz", "100Hz", "1kHz", "10kHz",
                                              "100kHz", "1MHz", "10MHz"};
constexpr int NUM_MEM = 20;

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
lv_obj_t *help_overlay = nullptr;
bool help_open = false;
lv_obj_t *lbl_entry = nullptr;
lv_obj_t *pointer_tri = nullptr;
lv_obj_t *ch_labels[3] = {}; // CH2 (left), CH1 (mid), CH0 (right)
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
  syn->set_carrier((uint32_t)f, vfo.car_mode != 0);
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
  lv_label_set_text_fmt(lbl_mem, "%02d%s", vfo.mem_index + 1,
                        vfo.memories[vfo.mem_index] ? "*" : " ");
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
  }
  update_scale();
}

void memorize() {
  vfo.memories[vfo.mem_index] = vfo.freq;
}

void load_memory() {
  if (vfo.memories[vfo.mem_index] != 0) {
    set_freq(vfo.memories[vfo.mem_index]);
  }
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

void handle_key(const KeyMsg &msg) {
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
    memorize();
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

  // preset memories (common HF/VHF frequencies)
  const int32_t presets[10] = {3573000, 5357000, 7074000, 10100000, 14074000,
                               18100000, 21074000, 24890000, 28400000, 50100000};
  for (int i = 0; i < 10; i++) {
    vfo.memories[i] = presets[i];
  }
  vfo.freq = 7074000;

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
  lv_obj_set_style_pad_all(help_panel, 6, 0);

  // lay it out as a grid: 4 columns (15% / 35% / 15% / 35%); the first two
  // rows only use the first key column and let the value span the rest.
  lv_obj_set_layout(help_panel, LV_LAYOUT_GRID);
  static lv_coord_t help_cols[] = {LV_GRID_FR(3), LV_GRID_FR(7), LV_GRID_FR(3),
                                   LV_GRID_FR(7), LV_GRID_TEMPLATE_LAST};
  static lv_coord_t help_rows[] = {
      LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT,
      LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};
  lv_obj_set_grid_dsc_array(help_panel, help_cols, help_rows);
  lv_obj_set_style_pad_row(help_panel, 1, 0);
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

  // --- keyboard ---
  cardputer.initialize_keyboard(on_key);

  // --- push the initial LO (and disable the carrier) and draw everything ---
  if (si_ok) {
    apply_lo();
    apply_carrier();
  }
  update_ui();

  // --- main loop: service LVGL and process key events ---
  uint32_t last_log = 0;
  uint32_t last_si_retry = 0;
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
    if (now - last_log > pdMS_TO_TICKS(3000)) {
      last_log = now;
      printf("VFO %ld Hz  step=%s  mem=%02d  IF%+ld  car=%s@%ld  si5351=%s\n",
             (long)vfo.freq, STEP_NAME[vfo.step_index], vfo.mem_index + 1,
             (long)vfo.if_offset, car_mode_name(), (long)vfo.car_freq,
             si_ok ? "OK" : "FAIL");
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
