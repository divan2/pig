import cv2
import numpy as np
from smbus2 import SMBus
import time

# Настройки камеры
show_image = input("Показывать изображение на экране? (1 - да, 0 - нет): ").strip()
show_image = show_image == "1"

# Начальные значения HSV для зелёного
lower_hsv = np.array([35, 40, 40])
upper_hsv = np.array([85, 255, 255])

# Настройки I2C
WRITE_DAC = 0x40
ADDR_A1 = 0x60  # Правый газ
ADDR_A2 = 0x61  # Левый газ
ADDR_B1 = 0x60  # Правый тормоз (шина 2)
ADDR_B2 = 0x61  # Левый тормоз (шина 2)

# Настройки управления
DEADZONE = 0.1
SMOOTHING_FACTOR = 0.2  # Коэффициент экспоненциального сглаживания

# Инициализация I2C
bus1 = SMBus(1)  # GPIO2/3 (шина 1)
bus2 = SMBus(14)  # GPIO23/24 (шина 2)

# Текущие значения для сглаживания
current_values = {
    'right_gas': 0,
    'left_gas': 0,
    'right_brake': 0,
    'left_brake': 0
}


def set_dac(bus, addr, value):
    """Установить значение (0..4095) на MCP4725"""
    value = max(0, min(4095, int(value * 40.95)))  # Конвертация % в 12-битное значение
    high = (value >> 4) & 0xFF
    low = (value << 4) & 0xFF
    bus.write_i2c_block_data(addr, WRITE_DAC, [high, low])


def apply_smoothing(new_values):
    """Применяет экспоненциальное сглаживание к значениям"""
    for key in current_values:
        current_values[key] = (SMOOTHING_FACTOR * new_values[key] +
                               (1 - SMOOTHING_FACTOR) * current_values[key])
    return current_values


def normalize_coordinates(x, y, width, height):
    """Нормализует координаты в диапазон [-1, 1]"""
    nx = 2 * (x / width) - 1
    ny = 1 - 2 * (y / height)  # Инвертируем ось Y
    return nx, ny


def calculate_controls(nx, ny):
    """Вычисляет управляющие сигналы на основе нормализованных координат"""
    # Инициализация значений
    controls = {
        'right_gas': 0,
        'left_gas': 0,
        'right_brake': 0,
        'left_brake': 0
    }

    # Применяем deadzone
    if abs(nx) < DEADZONE:
        nx = 0
    if abs(ny) < DEADZONE:
        ny = 0

    # Приоритет поворота
    if nx > 0.5:  # Поворот вправо
        controls['left_gas'] = 100
        controls['right_brake'] = 100
    elif nx < -0.5:  # Поворот влево
        controls['right_gas'] = 100
        controls['left_brake'] = 100
    else:
        # Линейное управление движением
        if ny < -0.2:  # Движение вперед
            gas = min(100, 100 * (-ny))
            controls['right_gas'] = gas
            controls['left_gas'] = gas
        elif ny > 0.2:  # Торможение
            brake = min(100, 100 * ny)
            controls['right_brake'] = brake
            controls['left_brake'] = brake

        # Добавляем поворот если не в deadzone
        if nx != 0:
            turn_factor = abs(nx) * 100
            if nx > 0:  # Поворот вправо
                controls['left_gas'] = min(100, controls['left_gas'] + turn_factor)
                controls['right_gas'] = max(0, controls['right_gas'] - turn_factor)
            else:  # Поворот влево
                controls['right_gas'] = min(100, controls['right_gas'] + turn_factor)
                controls['left_gas'] = max(0, controls['left_gas'] - turn_factor)

    return controls


# Основной цикл
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Камера не обнаружена")
    exit()

print("Нажмите 'q' в окне изображения или Ctrl+C в терминале для выхода.")
print("Введите 3 и нажмите Enter для калибровки цвета по центру кадра.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Ошибка захвата кадра")
            break

        # Получаем размеры кадра
        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2

        # Преобразуем в HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Маска по текущим границам HSV
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Поиск контуров
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = False
        for contour in contours:
            x, y, rw, rh = cv2.boundingRect(contour)
            if rw >= 10 and rh >= 10:  # Минимальный размер объекта
                # Нормализуем координаты
                nx, ny = normalize_coordinates(x + rw / 2, y + rh / 2, w, h)

                # Вычисляем управляющие сигналы
                controls = calculate_controls(nx, ny)
                smoothed = apply_smoothing(controls)

                # Управляем ЦАП
                set_dac(bus1, ADDR_A1, smoothed['right_gas'])  # Правый газ
                set_dac(bus1, ADDR_A2, smoothed['left_gas'])  # Левый газ
                set_dac(bus2, ADDR_B1, smoothed['right_brake'])  # Правый тормоз
                set_dac(bus2, ADDR_B2, smoothed['left_brake'])  # Левый тормоз

                print(
                    f"Координаты: ({nx:.2f}, {ny:.2f}) | Управление: Газ R/L: {smoothed['right_gas']:.0f}/{smoothed['left_gas']:.0f}% | Тормоз R/L: {smoothed['right_brake']:.0f}/{smoothed['left_brake']:.0f}%")
                found = True

                if show_image:
                    cv2.rectangle(frame, (x, y), (x + rw, y + rh), (0, 255, 0), 2)
                    cv2.putText(frame, f"X: {nx:.2f}, Y: {ny:.2f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                break

        if not found:
            # Если объект не найден - останавливаем все
            controls = {'right_gas': 0, 'left_gas': 0, 'right_brake': 0, 'left_brake': 0}
            smoothed = apply_smoothing(controls)
            set_dac(bus1, ADDR_A1, 0)
            set_dac(bus1, ADDR_A2, 0)
            set_dac(bus2, ADDR_B1, 0)
            set_dac(bus2, ADDR_B2, 0)
            print("0")

        # Рисуем центр и отображаем изображение
        if show_image:
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.imshow("Frame", frame)

        # Обработка клавиш
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        # Калибровка цвета
        import sys, select

        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            cmd = input().strip()
            if cmd == "3":
                center_hsv = hsv[cy, cx]
                h_val, s_val, v_val = int(center_hsv[0]), int(center_hsv[1]), int(center_hsv[2])
                print(f"🔧 Калибровка по цвету HSV: {center_hsv}")

                delta_h, delta_s, delta_v = 10, 60, 60
                lower_hsv = np.array([
                    max(0, h_val - delta_h),
                    max(0, s_val - delta_s),
                    max(0, v_val - delta_v)
                ])
                upper_hsv = np.array([
                    min(179, h_val + delta_h),
                    min(255, s_val + delta_s),
                    min(255, v_val + delta_v)
                ])
                print(f"🎯 Новый HSV диапазон:")
                print(f"   Нижний: {lower_hsv}")
                print(f"   Верхний: {upper_hsv}")

except KeyboardInterrupt:
    print("\nЗавершено пользователем (Ctrl+C)")

# Остановка всех выходов
set_dac(bus1, ADDR_A1, 0)
set_dac(bus1, ADDR_A2, 0)
set_dac(bus2, ADDR_B1, 0)
set_dac(bus2, ADDR_B2, 0)
bus1.close()
bus2.close()
cap.release()
if show_image:
    cv2.destroyAllWindows()