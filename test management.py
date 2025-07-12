import cv2
import numpy as np
from smbus2 import SMBus
import time
import sys
import select

# === Глобальные настройки ===
WRITE_DAC = 0x40

# Адреса ЦАП
ADDR_A1 = 0x60  # тормоз левого
ADDR_A2 = 0x61  # газ левого
ADDR_B1 = 0x60  # тормоз правого
ADDR_B2 = 0x61  # газ правого

# Камера и HSV
show_image = True
lower_hsv = np.array([35, 40, 40])
upper_hsv = np.array([85, 255, 255])
calibrated = False

# Управление
DEADZONE = 0.1
SMOOTHING_FACTOR = 0.2

# I2C шины
bus1 = SMBus(1)   # Левое колесо
bus2 = SMBus(14)  # Правое колесо

# Значения сглаживания
current_values = {
    'right_gas': 0,
    'left_gas': 0,
    'right_brake': 0,
    'left_brake': 0
}

# === Кнопка ===
button_rect = (10, 10, 150, 40)  # x, y, w, h
calibration_requested = False

def mouse_callback(event, x, y, flags, param):
    global calibration_requested
    if event == cv2.EVENT_LBUTTONDOWN:
        bx, by, bw, bh = button_rect
        if bx <= x <= bx + bw and by <= y <= by + bh:
            calibration_requested = True


def set_dac(bus, addr, value):
    value = max(0, min(4095, int(value * 40.95)))  # 12-бит
    high = (value >> 4) & 0xFF
    low = (value << 4) & 0xFF
    bus.write_i2c_block_data(addr, WRITE_DAC, [high, low])


def apply_smoothing(new_values):
    for key in current_values:
        current_values[key] = (SMOOTHING_FACTOR * new_values[key] +
                               (1 - SMOOTHING_FACTOR) * current_values[key])
    return current_values


def normalize_coordinates(x, y, width, height):
    nx = 2 * (x / width) - 1
    ny = 1 - 2 * (y / height)
    return nx, ny


def calculate_controls_v2(nx, ny):
    controls = {'right_gas': 0, 'left_gas': 0, 'right_brake': 0, 'left_brake': 0}
    ACC_Y_THRESHOLD = -0.3
    BRAKE_Y_THRESHOLD = -0.3
    TURN_X_BORDER = 0.7
    MAX_BRAKE_TURN = 25

    if ny >= ACC_Y_THRESHOLD:
        base_gas = (ny - ACC_Y_THRESHOLD) / (1 - ACC_Y_THRESHOLD) * 100
        if -TURN_X_BORDER < nx < TURN_X_BORDER:
            correction = abs(nx) / TURN_X_BORDER
            if nx < 0:
                controls['left_gas'] = base_gas * (1 - correction)
                controls['right_gas'] = base_gas
            else:
                controls['right_gas'] = base_gas * (1 - correction)
                controls['left_gas'] = base_gas
        else:
            if nx < 0:
                controls['right_gas'] = base_gas
                controls['left_brake'] = (abs(nx) - TURN_X_BORDER) / (1 - TURN_X_BORDER) * MAX_BRAKE_TURN
            else:
                controls['left_gas'] = base_gas
                controls['right_brake'] = (abs(nx) - TURN_X_BORDER) / (1 - TURN_X_BORDER) * MAX_BRAKE_TURN
    elif ny < BRAKE_Y_THRESHOLD:
        base_brake = (abs(ny) - abs(BRAKE_Y_THRESHOLD)) / (1 - abs(BRAKE_Y_THRESHOLD)) * 100
        controls['left_brake'] = base_brake
        controls['right_brake'] = base_brake
        if abs(nx) > DEADZONE:
            extra = abs(nx) * 25
            if nx > 0:
                controls['left_brake'] = min(100, controls['left_brake'] + extra)
            else:
                controls['right_brake'] = min(100, controls['right_brake'] + extra)
    return controls


# === Основной цикл ===
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Камера не обнаружена")
    exit()

cv2.namedWindow("Camera Tracking")
cv2.setMouseCallback("Camera Tracking", mouse_callback)

print("=== Управление ===")
print("Нажмите кнопку 'Kalibrovka' в окне для калибровки по центру")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Ошибка захвата кадра")
            break

        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Калибровка по кнопке
        if calibration_requested:
            calibration_requested = False
            center_hsv = hsv[cy, cx]
            h_val, s_val, v_val = int(center_hsv[0]), int(center_hsv[1]), int(center_hsv[2])
            delta_h, delta_s, delta_v = 10, 60, 60
            lower_hsv = np.array([max(0, h_val - delta_h), max(0, s_val - delta_s), max(0, v_val - delta_v)])
            upper_hsv = np.array([min(179, h_val + delta_h), min(255, s_val + delta_s), min(255, v_val + delta_v)])
            calibrated = True
            print(f"🎯 HSV диапазон установлен: {lower_hsv} - {upper_hsv}")

        # Обнаружение объекта
        if calibrated:
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            found = False
            for contour in contours:
                x, y, rw, rh = cv2.boundingRect(contour)
                if rw >= 10 and rh >= 10:
                    nx, ny = normalize_coordinates(x + rw / 2, y + rh / 2, w, h)
                    controls = calculate_controls_v2(nx, ny)
                    smoothed = apply_smoothing(controls)

                    # Запись на ЦАП
                    set_dac(bus1, ADDR_A1, smoothed['left_brake'])
                    set_dac(bus1, ADDR_A2, smoothed['left_gas'])
                    set_dac(bus2, ADDR_B1, smoothed['right_brake'])
                    set_dac(bus2, ADDR_B2, smoothed['right_gas'])

                    print(f"L: gas {smoothed['left_gas']:.0f}%, brake {smoothed['left_brake']:.0f}% | "
                          f"R: gas {smoothed['right_gas']:.0f}%, brake {smoothed['right_brake']:.0f}%")

                    cv2.rectangle(frame, (x, y), (x + rw, y + rh), (0, 255, 0), 2)
                    found = True
                    break

            if not found:
                for addr in [ADDR_A1, ADDR_A2]:
                    set_dac(bus1, addr, 0)
                for addr in [ADDR_B1, ADDR_B2]:
                    set_dac(bus2, addr, 0)

        # Центр
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # Кнопка калибровки
        cv2.rectangle(frame, (button_rect[0], button_rect[1]),
                      (button_rect[0] + button_rect[2], button_rect[1] + button_rect[3]),
                      (50, 200, 50), -1)
        cv2.putText(frame, "Kalibrovka", (button_rect[0] + 10, button_rect[1] + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        cv2.imshow("Camera Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nОстановка пользователем")

# Завершение
for addr in [ADDR_A1, ADDR_A2]:
    set_dac(bus1, addr, 0)
for addr in [ADDR_B1, ADDR_B2]:
    set_dac(bus2, addr, 0)
bus1.close()
bus2.close()
cap.release()
cv2.destroyAllWindows()
