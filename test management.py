import cv2
import numpy as np
from smbus2 import SMBus
from multiprocessing import Process, Pipe
import time

# === ЦАП ===
WRITE_DAC = 0x40
ADDR_A1 = 0x60  # тормоз левого
ADDR_A2 = 0x61  # газ левого
ADDR_B1 = 0x60  # тормоз правого
ADDR_B2 = 0x61  # газ правого

# === I2C ===
bus1 = SMBus(1)
bus2 = SMBus(14)

def set_dac(bus, addr, value):
    value = max(0, min(4095, int(value * 40.95)))
    high = (value >> 4) & 0xFF
    low = (value << 4) & 0xFF
    bus.write_i2c_block_data(addr, WRITE_DAC, [high, low])


# === Глобальные параметры управления ===
DEADZONE = 0.1
SMOOTHING_FACTOR = 0.2
current_values = {
    'right_gas': 0,
    'left_gas': 0,
    'right_brake': 0,
    'left_brake': 0
}

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

# === Процесс обработки изображения и управления ===
def processing_loop(pipe, frame_shape):
    lower_hsv = np.array([35, 40, 40])
    upper_hsv = np.array([85, 255, 255])
    calibrated = False
    h, w = frame_shape

    while True:
        msg = pipe.recv()
        if msg[0] == 'calibrate':
            hsv_center = msg[1]
            delta_h, delta_s, delta_v = 10, 60, 60
            lower_hsv = np.array([
                max(0, hsv_center[0] - delta_h),
                max(0, hsv_center[1] - delta_s),
                max(0, hsv_center[2] - delta_v)
            ])
            upper_hsv = np.array([
                min(179, hsv_center[0] + delta_h),
                min(255, hsv_center[1] + delta_s),
                min(255, hsv_center[2] + delta_v)
            ])
            calibrated = True
            print(f"[🔧] Калибровано: lower={lower_hsv}, upper={upper_hsv}")
        elif msg[0] == 'frame':
            frame = msg[1]
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv) if calibrated else None

            found = False
            if calibrated:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    x, y, rw, rh = cv2.boundingRect(contour)
                    if rw >= 10 and rh >= 10:
                        nx, ny = normalize_coordinates(x + rw / 2, y + rh / 2, w, h)
                        controls = calculate_controls_v2(nx, ny)
                        smoothed = apply_smoothing(controls)

                        set_dac(bus1, ADDR_A1, smoothed['left_brake'])
                        set_dac(bus1, ADDR_A2, smoothed['left_gas'])
                        set_dac(bus2, ADDR_B1, smoothed['right_brake'])
                        set_dac(bus2, ADDR_B2, smoothed['right_gas'])

                        print(f"[🚗] LEFT: gas={smoothed['left_gas']:.1f}%, brake={smoothed['left_brake']:.1f}% | "
                              f"RIGHT: gas={smoothed['right_gas']:.1f}%, brake={smoothed['right_brake']:.1f}%")
                        found = True
                        break

            if not found and calibrated:
                # Стоп
                for addr in [ADDR_A1, ADDR_A2]:
                    set_dac(bus1, addr, 0)
                for addr in [ADDR_B1, ADDR_B2]:
                    set_dac(bus2, addr, 0)
                print("[🛑] Объект не найден — все выходы 0%")

# === Главный процесс (камера + окно) ===
def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Камера не обнаружена")
        return

    ret, frame = cap.read()
    if not ret:
        print("❌ Не удалось получить кадр")
        return

    h, w, _ = frame.shape
    cx, cy = w // 2, h // 2
    button_rect = (10, 10, 150, 40)

    parent_conn, child_conn = Pipe()
    proc = Process(target=processing_loop, args=(child_conn, (h, w)))
    proc.start()

    print("=== Управление ===\nНажмите кнопку 'Kalibrovka' в окне")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Отправляем кадр в обработчик
        parent_conn.send(('frame', frame.copy()))

        # Рисуем кнопку
        cv2.rectangle(frame, (button_rect[0], button_rect[1]),
                      (button_rect[0] + button_rect[2], button_rect[1] + button_rect[3]),
                      (50, 200, 50), -1)
        cv2.putText(frame, "Kalibrovka", (button_rect[0] + 10, button_rect[1] + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # Центр
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        def mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                bx, by, bw, bh = button_rect
                if bx <= x <= bx + bw and by <= y <= by + bh:
                    hsv_center = hsv[cy, cx]
                    parent_conn.send(('calibrate', hsv_center))
                    print(f"[🎯] Калибровка по HSV: {hsv_center}")

        cv2.namedWindow("Camera Tracking")
        cv2.setMouseCallback("Camera Tracking", mouse)
        cv2.imshow("Camera Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    parent_conn.close()
    proc.terminate()
    cv2.destroyAllWindows()
    for addr in [ADDR_A1, ADDR_A2]:
        set_dac(bus1, addr, 0)
    for addr in [ADDR_B1, ADDR_B2]:
        set_dac(bus2, addr, 0)
    bus1.close()
    bus2.close()
    print("👋 Завершение")

if __name__ == "__main__":
    main()
