import cv2
import numpy as np
from smbus2 import SMBus
import time

# Настройки камеры
show_image = True

# Начальные значения HSV
lower_hsv = np.array([35, 40, 40])
upper_hsv = np.array([85, 255, 255])
calibrated = False

# Настройки I2C
WRITE_DAC = 0x40
ADDR_A1 = 0x60  # Правый газ
ADDR_A2 = 0x61  # Левый газ
ADDR_B1 = 0x60  # Правый тормоз
ADDR_B2 = 0x61  # Левый тормоз

# Настройки управления
SMOOTHING_FACTOR = 0.2

# Инициализация I2C
bus1 = SMBus(1)  # Шина 1
bus2 = SMBus(14)  # Шина 2

# Текущие значения
current_values = {
    'right_gas': 0,
    'left_gas': 0,
    'right_brake': 0,
    'left_brake': 0
}

# Кнопка калибровки
calib_button = {'x1': 20, 'y1': 400, 'x2': 200, 'y2': 450}
button_pressed = False


def set_dac(bus, addr, value):
    value = max(0, min(4095, int(value * 40.95)))
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


def calculate_controls(nx, ny):
    controls = {
        'right_gas': 0,
        'left_gas': 0,
        'right_brake': 0,
        'left_brake': 0
    }

    # Управление по Y (газ/тормоз)
    if ny <= -0.3:  # Зона ускорения (y = [-0.3, 1])
        # Интенсивность ускорения от 0 (при y = -0.3) до 100% (при y = 1)
        acceleration = (ny + 0.3) / 1.3  # нормализация от 0 до 1
        acceleration = max(0, min(1, acceleration))  # ограничение
        gas_value = int(100 * acceleration)
        controls['right_gas'] = gas_value
        controls['left_gas'] = gas_value

        # Управление поворотом при ускорении
        if nx < 0:  # Поворот влево
            if -0.7 <= nx <= 0:
                # Левый мотор уменьшается от максимума до 0
                left_factor = (nx + 0.7) / 0.7
                controls['left_gas'] = int(gas_value * left_factor)
            elif nx < -0.7:
                # Левый мотор тормозит от 0 до 25%
                brake_factor = (-nx - 0.7) / 0.3
                controls['left_brake'] = int(25 * brake_factor)

        elif nx > 0:  # Поворот вправо
            if 0 <= nx <= 0.7:
                # Правый мотор уменьшается от максимума до 0
                right_factor = (0.7 - nx) / 0.7
                controls['right_gas'] = int(gas_value * right_factor)
            elif nx > 0.7:
                # Правый мотор тормозит от 0 до 25%
                brake_factor = (nx - 0.7) / 0.3
                controls['right_brake'] = int(25 * brake_factor)

    else:  # Зона торможения (y = [-1, -0.3])
        # Интенсивность торможения от 0 (при y = -0.3) до 100% (при y = -1)
        braking = (-ny - 0.3) / 0.7  # нормализация от 0 до 1
        braking = max(0, min(1, braking))  # ограничение
        brake_value = int(100 * braking)
        controls['right_brake'] = brake_value
        controls['left_brake'] = brake_value

        # Управление поворотом при торможении
        if nx < 0:  # Поворот влево
            # Увеличиваем торможение левого мотора
            turn_factor = min(1, -nx)  # от 0 до 1
            additional_brake = int(100 * turn_factor)
            controls['left_brake'] = min(100, brake_value + additional_brake)

        elif nx > 0:  # Поворот вправо
            # Увеличиваем торможение правого мотора
            turn_factor = min(1, nx)  # от 0 до 1
            additional_brake = int(100 * turn_factor)
            controls['right_brake'] = min(100, brake_value + additional_brake)

    return controls


def draw_button(frame, pressed=False):
    color = (0, 200, 0) if pressed else (0, 120, 0)
    cv2.rectangle(frame,
                  (calib_button['x1'], calib_button['y1']),
                  (calib_button['x2'], calib_button['y2']),
                  color, -1)
    cv2.putText(frame, "KALIBROVKA",
                (calib_button['x1'] + 10, calib_button['y1'] + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


def check_button_click(x, y):
    return (calib_button['x1'] <= x <= calib_button['x2'] and
            calib_button['y1'] <= y <= calib_button['y2'])


cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Camera not found")
    exit()


# Функция обработки кликов мыши
def mouse_callback(event, x, y, flags, param):
    global button_pressed, calibrated, lower_hsv, upper_hsv

    if event == cv2.EVENT_LBUTTONDOWN:
        if check_button_click(x, y):
            button_pressed = True
    elif event == cv2.EVENT_LBUTTONUP:
        if button_pressed and check_button_click(x, y):
            # Калибровка по центру кадра
            _, frame = cap.read()
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h, w = frame.shape[:2]
            center_hsv = hsv_frame[h // 2, w // 2]

            h_val, s_val, v_val = int(center_hsv[0]), int(center_hsv[1]), int(center_hsv[2])
            print(f"Calibration HSV: {center_hsv}")

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

            print(f"New HSV range:")
            print(f"  Lower: {lower_hsv}")
            print(f"  Upper: {upper_hsv}")
            calibrated = True
        button_pressed = False


cv2.namedWindow("Camera Tracking")
cv2.setMouseCallback("Camera Tracking", mouse_callback)

print("=== Instructions ===")
print("1. Click 'KALIBROVKA' button to calibrate")
print("2. Control will work after calibration")
print("3. Press 'q' to quit")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame capture error")
            break

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Рисуем кнопку
        draw_button(frame, button_pressed)

        status_text = "Ready for calibration" if not calibrated else "Calibration done"
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if calibrated:
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            found = False
            for contour in contours:
                x, y, rw, rh = cv2.boundingRect(contour)
                if rw >= 10 and rh >= 10:
                    nx, ny = normalize_coordinates(x + rw / 2, y + rh / 2, w, h)
                    controls = calculate_controls(nx, ny)
                    smoothed = apply_smoothing(controls)

                    # Управление ЦАПами
                    set_dac(bus1, ADDR_A1, smoothed['right_gas'])
                    set_dac(bus1, ADDR_A2, smoothed['left_gas'])
                    set_dac(bus2, ADDR_B1, smoothed['right_brake'])
                    set_dac(bus2, ADDR_B2, smoothed['left_brake'])

                    control_text = (f"Right: gas {smoothed['right_gas']}%, brake {smoothed['right_brake']}% | "
                                    f"Left: gas {smoothed['left_gas']}%, brake {smoothed['left_brake']}%")
                    print(control_text)

                    cv2.putText(frame, control_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    found = True

                    cv2.rectangle(frame, (x, y), (x + rw, y + rh), (0, 255, 0), 2)
                    cv2.putText(frame, f"X: {nx:.2f}, Y: {ny:.2f}", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    break

            if not found:
                # Обнуляем все выходы если объект не найден
                controls = {'right_gas': 0, 'left_gas': 0, 'right_brake': 0, 'left_brake': 0}
                smoothed = apply_smoothing(controls)
                set_dac(bus1, ADDR_A1, 0)
                set_dac(bus1, ADDR_A2, 0)
                set_dac(bus2, ADDR_B1, 0)
                set_dac(bus2, ADDR_B2, 0)
                print("Object not found - all outputs 0%")

        # Отображение центра и выходного изображения
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.imshow("Camera Tracking", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

except KeyboardInterrupt:
    print("\nStopped by user (Ctrl+C)")

# Остановка всех выходов
set_dac(bus1, ADDR_A1, 0)
set_dac(bus1, ADDR_A2, 0)
set_dac(bus2, ADDR_B1, 0)
set_dac(bus2, ADDR_B2, 0)
bus1.close()
bus2.close()
cap.release()
cv2.destroyAllWindows()