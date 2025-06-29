import time
import board
import busio
from adafruit_mcp4725 import MCP4725

# Настройка I2C
i2c = busio.I2C(board.SCL, board.SDA)
dac = MCP4725(i2c, address=0x60)

try:
    print("Нажмите Ctrl+C для выхода")
    while True:
        # Вверх
        for value in range(0, 4096):  # 12-битный диапазон: 0–4095
            dac.value = value
            time.sleep(0.001)  # скорость изменения — можно регулировать

        # Вниз
        for value in range(4095, -1, -1):
            dac.value = value
            time.sleep(0.001)

except KeyboardInterrupt:
    print("\nПрограмма завершена пользователем.")
    dac.value = 0  # на выходе 0 В при завершении
