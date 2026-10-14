#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

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
// ---------------------------------------------------------------------------

namespace {

constexpr int32_t MIN_FREQ = 1000000;    // 1 MHz
constexpr int32_t MAX_FREQ = 150000000;  // 150 MHz
constexpr int NUM_STEPS = 7;
constexpr int32_t STEP_HZ[NUM_STEPS] = {10, 100, 1000, 10000, 100000, 1000000, 10000000};
constexpr const char *STEP_NAME[NUM_STEPS] = {"10Hz", "100Hz", "1kHz", "10kHz",
                                              "100kHz", "1MHz", "10MHz"};
constexpr int NUM_MEM = 20;

// Layout (240 x 135)
constexpr int W = 240;
constexpr int H = 135;
constexpr int SCALE_CENTER_X = 120;
constexpr int SCALE_PITCH = 20;
constexpr int NTICKS = 13; // odd -> symmetric around the center index
constexpr int TICK_TOP = 62;
constexpr int TICK_MINOR_BOTTOM = 80;
constexpr int TICK_MAJOR_BOTTOM = 88;
constexpr int TICK_LABEL_Y = 90;

// Retro amber-on-black palette
constexpr uint32_t COL_READOUT = 0xFFCC66;
constexpr uint32_t COL_TICK_MINOR = 0x8A6A2A;
constexpr uint32_t COL_TICK_MAJOR = 0xFFCC66;
constexpr uint32_t COL_POINTER = 0xFF3B1E;
constexpr uint32_t COL_STATUS = 0x66CCAA;

struct KeyMsg {
  char value;
  uint8_t special;
  bool shift;
};

struct VfoState {
  int32_t freq = 7074000; // start at 7.074 MHz
  int step_index = 2;     // 1 kHz
  int mem_index = 0;
  int32_t memories[NUM_MEM] = {};
  bool entering = false;
  std::string entry;
};

VfoState vfo;
QueueHandle_t key_queue = nullptr;
Si5351 *syn = nullptr;

lv_obj_t *lbl_freq = nullptr;
lv_obj_t *lbl_step = nullptr;
lv_obj_t *lbl_mem = nullptr;
lv_obj_t *lbl_hint = nullptr;
lv_obj_t *lbl_entry = nullptr;
lv_obj_t *pointer_line = nullptr;
lv_obj_t *tick_lines[NTICKS] = {};
lv_obj_t *tick_labels[NTICKS] = {};
lv_point_precise_t tick_pts[NTICKS][2] = {};
lv_point_precise_t pointer_pts[2] = {};

// ---- helpers --------------------------------------------------------------

void format_freq(int32_t f, char *buf, size_t n) {
  if (f < 0) {
    f = 0;
  }
  snprintf(buf, n, "%d.%03d.%03d", (int)(f / 1000000), (int)((f / 1000) % 1000),
           (int)(f % 1000));
}

void format_tick(int32_t f, int32_t minor, char *buf, size_t n) {
  if (minor >= 1000) {
    snprintf(buf, n, "%d", (int)(f / 1000)); // kHz
  } else {
    int dec = (minor >= 100) ? 1 : ((minor >= 10) ? 2 : 3);
    long div = (dec == 1) ? 100 : ((dec == 2) ? 10 : 1);
    snprintf(buf, n, "%d.%0*ld", (int)(f / 1000), dec, (long)((f % 1000) / div));
  }
}

void set_freq(int32_t f) {
  if (f < MIN_FREQ) {
    f = MIN_FREQ;
  }
  if (f > MAX_FREQ) {
    f = MAX_FREQ;
  }
  vfo.freq = f;
  if (syn) {
    syn->set_frequency((uint32_t)f, 0);
  }
}

void update_scale() {
  int32_t minor = STEP_HZ[vfo.step_index];
  int32_t base = (vfo.freq / minor) * minor;
  float phase = (float)(vfo.freq % minor) / (float)minor;
  int center = NTICKS / 2;
  for (int i = 0; i < NTICKS; i++) {
    int s = i - center;
    float x = (float)SCALE_CENTER_X + ((float)s - phase) * (float)SCALE_PITCH;
    int32_t tf = base + (int32_t)s * minor;
    bool major = (((base / minor) + s) % 5) == 0;
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
      lv_obj_set_pos(tick_labels[i], (lv_coord_t)(x + 0.5f) - 28, TICK_LABEL_Y);
      lv_obj_remove_flag(tick_labels[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(tick_labels[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
}

void update_ui() {
  char buf[24];
  if (vfo.entering) {
    snprintf(buf, sizeof(buf), "%s_", vfo.entry.c_str());
    lv_label_set_text(lbl_freq, buf);
  } else {
    format_freq(vfo.freq, buf, sizeof(buf));
    lv_label_set_text(lbl_freq, buf);
  }
  lv_label_set_text_fmt(lbl_step, "STEP %s", STEP_NAME[vfo.step_index]);
  lv_label_set_text_fmt(lbl_mem, "MEM %02d%s", vfo.mem_index + 1,
                        vfo.memories[vfo.mem_index] ? "*" : " ");
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

// ---- key handling ---------------------------------------------------------

void handle_key(const KeyMsg &msg) {
  char c = msg.value;
  auto special = static_cast<espp::M5StackCardputer::SpecialKey>(msg.special);
  using SK = espp::M5StackCardputer::SpecialKey;

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

  // direct-entry mode: gather digits until Enter / Esc
  if (vfo.entering) {
    if (c >= '0' && c <= '9') {
      if (vfo.entry.size() < 9) {
        vfo.entry.push_back(c);
      }
    } else if (c == '\b') {
      if (!vfo.entry.empty()) {
        vfo.entry.pop_back();
      }
    } else if (c == '\n') {
      if (vfo.entry.empty()) {
        vfo.entering = false;
      } else {
        set_freq((int32_t)strtol(vfo.entry.c_str(), nullptr, 10));
        vfo.entering = false;
      }
    } else if (special == SK::ESC || c == '`') {
      vfo.entering = false;
    }
    update_ui();
    lv_label_set_text(lbl_hint,
                      vfo.entering ? "Ketik Hz, Enter=OK, Esc=batal" : "");
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
  default:
    if (c >= '0' && c <= '9') {
      vfo.entering = true;
      vfo.entry.clear();
      vfo.entry.push_back(c);
    } else {
      return; // nothing to update
    }
    break;
  }
  lv_label_set_text(lbl_hint, "");
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

  lbl_step = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_step, &lv_font_unscii_16, 0);
  lv_obj_set_style_text_color(lbl_step, lv_color_hex(COL_STATUS), 0);
  lv_obj_align(lbl_step, LV_ALIGN_TOP_LEFT, 4, 2);

  lbl_mem = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_mem, &lv_font_unscii_16, 0);
  lv_obj_set_style_text_color(lbl_mem, lv_color_hex(COL_STATUS), 0);
  lv_obj_align(lbl_mem, LV_ALIGN_TOP_RIGHT, -4, 2);

  lbl_freq = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_freq, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(lbl_freq, lv_color_hex(COL_READOUT), 0);
  lv_obj_align(lbl_freq, LV_ALIGN_TOP_MID, 0, 20);

  // sliding analog scale
  for (int i = 0; i < NTICKS; i++) {
    tick_lines[i] = lv_line_create(scr);
    lv_obj_set_style_line_width(tick_lines[i], 2, 0);
    lv_obj_set_style_line_color(
        tick_lines[i], lv_color_hex(((i - NTICKS / 2) % 5 == 0) ? COL_TICK_MAJOR : COL_TICK_MINOR),
        0);
    tick_labels[i] = lv_label_create(scr);
    lv_obj_set_style_text_font(tick_labels[i], &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(tick_labels[i], lv_color_hex(COL_TICK_MAJOR), 0);
    lv_obj_set_style_text_align(tick_labels[i], LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(tick_labels[i], 56);
    lv_obj_add_flag(tick_labels[i], LV_OBJ_FLAG_HIDDEN);
  }

  pointer_line = lv_line_create(scr);
  lv_obj_set_style_line_width(pointer_line, 3, 0);
  lv_obj_set_style_line_color(pointer_line, lv_color_hex(COL_POINTER), 0);
  pointer_pts[0] = {(lv_coord_t)SCALE_CENTER_X, TICK_TOP - 4};
  pointer_pts[1] = {(lv_coord_t)SCALE_CENTER_X, TICK_MAJOR_BOTTOM + 2};
  lv_line_set_points(pointer_line, pointer_pts, 2);

  lbl_hint = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_hint, &lv_font_unscii_16, 0);
  lv_obj_set_style_text_color(lbl_hint, lv_color_hex(COL_TICK_MINOR), 0);
  lv_obj_align(lbl_hint, LV_ALIGN_BOTTOM_MID, 0, -2);
  lv_label_set_text(lbl_hint, ",/ = tune   ;. = step");

  // --- keyboard ---
  cardputer.initialize_keyboard(on_key);

  // --- push the initial frequency to the Si5351 and draw everything ---
  if (si_ok) {
    syn->set_frequency((uint32_t)vfo.freq, 0);
  }
  update_ui();

  // --- main loop: service LVGL and process key events ---
  uint32_t last_log = 0;
  while (true) {
    lv_timer_handler();
    KeyMsg msg;
    while (key_queue && xQueueReceive(key_queue, &msg, 0) == pdTRUE) {
      handle_key(msg);
    }
    uint32_t now = xTaskGetTickCount();
    if (now - last_log > pdMS_TO_TICKS(3000)) {
      last_log = now;
      printf("VFO %ld Hz  step=%s  mem=%02d  si5351=%s\n", (long)vfo.freq,
             STEP_NAME[vfo.step_index], vfo.mem_index + 1, si_ok ? "OK" : "FAIL");
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
