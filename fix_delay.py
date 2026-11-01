import re

with open(r'C:\D\DOCUMENT_BCK\GitHub\M5Cardputer-Si5351-VFO\app\main\si5351.hpp', 'r') as f:
    content = f.read()

# Replace ets_delay_us with esp_rom_delay_us
content = content.replace('ets_delay_us(100)', 'esp_rom_delay_us(100)')

with open(r'C:\D\DOCUMENT_BCK\GitHub\M5Cardputer-Si5351-VFO\app\main\si5351.hpp', 'w') as f:
    f.write(content)

print('Fixed: using esp_rom_delay_us instead of ets_delay_us')
