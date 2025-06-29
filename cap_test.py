from smbus2 import SMBus
import time

# Адреса ЦАП
ADDR1 = 0x60  # A0 = GND
ADDR2 = 0x61  # A0 = VCC
WRITE_DAC = 0x40

def set_dac(bus, addr, value):
    """Установить значение на MCP4725"""
    value = max(0, min(4095, value))  # ограничение
    high = (value >> 4) & 0xFF
    low = (value << 4) & 0xFF
    bus.write_i2c_block_data(addr, WRITE_DAC, [high, low])

with SMBus(1) as bus:
    try:
        print("Управление двумя ЦАП. Нажми Ctrl+C для выхода.")
        while True:
            for v in range(0, 4096):
                set_dac(bus, ADDR1, v)         # возрастает
                set_dac(bus, ADDR2, 4095 - v)  # убывает
                time.sleep(0.001)
    except KeyboardInterrupt:
        print("Завершение.")
        set_dac(bus, ADDR1, 0)
        set_dac(bus, ADDR2, 0)
