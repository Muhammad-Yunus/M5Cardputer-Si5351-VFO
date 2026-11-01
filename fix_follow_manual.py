import re

with open(r'C:\D\DOCUMENT_BCK\GitHub\M5Cardputer-Si5351-VFO\app\main\app.cpp', 'r') as f:
    content = f.read()

# 1. Fix apply_carrier() to use correct freq based on car_follows_lo
old_apply = '''void apply_carrier() {
  if (!syn) {
    return;
  }
  int32_t f = vfo.car_freq;
  if (vfo.car_mode == 2) {'''

new_apply = '''void apply_carrier() {
  if (!syn) {
    return;
  }
  int32_t f;
  if (vfo.car_follows_lo) {
    // FOLLOW mode: carrier = LO frequency
    f = vfo.freq + vfo.if_offset;
  } else {
    // MANUAL mode: carrier = user-defined frequency
    f = vfo.car_freq;
  }
  if (vfo.car_mode == 2) {'''

content = content.replace(old_apply, new_apply)

# 2. Fix 'u' key handler to save freq correctly when toggling
old_case = '''  case 'u': // toggle carrier FOLLOW/MANUAL mode
    vfo.car_follows_lo = !vfo.car_follows_lo;
    if (!vfo.car_follows_lo) {
      vfo.car_freq = vfo.freq + vfo.if_offset;
    }
    apply_carrier();
    show_carrier_info();
    break;'''

new_case = '''  case 'u': // toggle carrier FOLLOW/MANUAL mode
    if (!vfo.car_follows_lo) {
      // Switching to FOLLOW: save current manual freq before switching
      vfo.car_freq = vfo.freq + vfo.if_offset;
    }
    vfo.car_follows_lo = !vfo.car_follows_lo;
    if (vfo.car_follows_lo) {
      // Switching to MANUAL: save current LO freq as manual baseline
      vfo.car_freq = vfo.freq + vfo.if_offset;
    }
    apply_carrier();
    show_carrier_info();
    break;'''

content = content.replace(old_case, new_case)

with open(r'C:\D\DOCUMENT_BCK\GitHub\M5Cardputer-Si5351-VFO\app\main\app.cpp', 'w') as f:
    f.write(content)

print('Done - fixed FOLLOW/MANUAL logic')
