from smbus2 import SMBus
import time

# Команда записи в DAC-регистр (без EEPROM)
WRITE_DAC = 0x40

# Адреса MCP4725
ADDR_A1 = 0x60  # i2c-1, A0=GND
ADDR_A2 = 0x61  # i2c-1, A0=VCC
ADDR_B1 = 0x60  # i2c-14, A0=GND
ADDR_B2 = 0x61  # i2c-14, A0=VCC

def set_dac(bus, addr, value):
    """Установить значение (0..4095) на MCP4725"""
    value = max(0, min(4095, value))
    high = (value >> 4) & 0xFF
    low = (value << 4) & 0xFF
    bus.write_i2c_block_data(addr, WRITE_DAC, [high, low])

# Открываем обе шины
bus1 = SMBus(1)    # GPIO2/3
bus2 = SMBus(14)   # GPIO23/24

try:
    print("Управление четырьмя ЦАП. Нажми Ctrl+C для выхода.")
    while True:
        for v in range(0, 4096):
            # Шина 1: зеркальное управление
            set_dac(bus1, ADDR_A1, v)         # возрастает
            set_dac(bus1, ADDR_A2, 4095 - v)  # убывает

            # Шина 2: синхронная пила, но быстрее
            set_dac(bus2, ADDR_B1, v)         # возрастает
            set_dac(bus2, ADDR_B2, 4095 - v)

            time.sleep(0.01)

        for v in range(0, 4096):
            # Шина 1: зеркальное управление
            set_dac(bus1, ADDR_A2, v)         # возрастает
            set_dac(bus1, ADDR_A1, 4095 - v)  # убывает

            # Шина 2: синхронная пила, но быстрее
            set_dac(bus2, ADDR_B2, v)         # возрастает
            set_dac(bus2, ADDR_B1, 4095 - v)

            time.sleep(0.01)
except KeyboardInterrupt:
    print("Завершение...")
    for bus in (bus1, bus2):
        for addr in (0x60, 0x61):
            try:
                set_dac(bus, addr, 0)
            except:
                pass
    bus1.close()
    bus2.close()
