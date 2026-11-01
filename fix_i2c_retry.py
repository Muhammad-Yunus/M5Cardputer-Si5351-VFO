import re

with open(r'C:\D\DOCUMENT_BCK\GitHub\M5Cardputer-Si5351-VFO\app\main\si5351.hpp', 'r') as f:
    content = f.read()

# 1. Add retry logic to write_reg
old_write_reg = '''    bool write_reg(uint8_t reg, uint8_t value) {
        uint8_t data[2] = {reg, value};
        return i2c_.write(address_, data, 2);
    }'''

new_write_reg = '''    // Retry I2C writes up to 3 times for transient bus errors
    bool write_reg(uint8_t reg, uint8_t value) {
        uint8_t data[2] = {reg, value};
        for (int retry = 0; retry < 3; ++retry) {
            if (i2c_.write(address_, data, 2)) {
                return true;
            }
            // Small delay between retries to let bus recover
            ets_delay_us(100);
        }
        return false;
    }'''

content = content.replace(old_write_reg, new_write_reg)

# 2. Add retry logic to write_div (8-byte bulk writes)
old_write_div = '''        return i2c_.write(address_, buf, 9);
    }'''

new_write_div = '''        for (int retry = 0; retry < 3; ++retry) {
            if (i2c_.write(address_, buf, 9)) {
                return true;
            }
            ets_delay_us(100);
        }
        return false;
    }'''

content = content.replace(old_write_div, new_write_div)

with open(r'C:\D\DOCUMENT_BCK\GitHub\M5Cardputer-Si5351-VFO\app\main\si5351.hpp', 'w') as f:
    f.write(content)

print('Done - added retry logic to Si5351 driver')
