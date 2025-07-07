import cv2
import numpy as np
from smbus2 import SMBus
import time
import sys
import select

# Настройки камеры
show_image = True

# Начальные значения HSV
lower_hsv = np.array([35, 40, 40])
upper_hsv = np.array([85, 255, 255])
calibrated = False

# Настройки I2C
WRITE_DAC = 0x40
ADDR_A1 = 0x60  # Pravyi gaz
ADDR_A2 = 0x61  # Levyi gaz
ADDR_B1 = 0x60  # Pravyi tormoz (shina 2)
ADDR_B2 = 0x61  # Levyi tormoz (shina 2)

# Настройки управления
SMOOTHING_FACTOR = 0.2

# Инициализация I2C
bus1 = SMBus(1)  # GPIO2/3 (shina 1)
bus2 = SMBus(14)  # GPIO23/24 (shina 2)

# Текущие значения
current_values = {
    'right_gas': 0,
    'left_gas': 0,
    'right_brake': 0,
    'left_brake': 0
}


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

    # Upravlenie po Y (gaz/tormoz)
    if ny < -0.8:  # Maksimalnyi gaz
        gas = 100
        brake = 0
    elif ny > 0.5:  # Maksimalnyi tormoz
        gas = 0
        brake = 100
    else:  # Lineinoe upravlenie
        gas = max(0, min(100, 100 * (-ny)))
        brake = max(0, min(100, 100 * ny))

        # Upravlenie po X (povoroty)
        if nx > 0.9:  # Povorot vpravo
            controls['left_gas'] = gas
            controls['right_brake'] = brake
        elif nx < -0.9:  # Povorot vlevo
            controls['right_gas'] = gas
            controls['left_brake'] = brake
        else:  # Lineinoe smeshenie
            turn_factor = abs(nx) * gas if gas > 0 else abs(nx) * brake

        if nx > 0:  # Vpravo
            controls['left_gas'] = min(100, gas + turn_factor)
            controls['right_gas'] = max(0, gas - turn_factor)
            controls['right_brake'] = min(100, brake + turn_factor)
            controls['left_brake'] = max(0, brake - turn_factor)
        else:  # Vlevo
            controls['right_gas'] = min(100, gas + turn_factor)
            controls['left_gas'] = max(0, gas - turn_factor)
            controls['left_brake'] = min(100, brake + turn_factor)
            controls['right_brake'] = max(0, brake - turn_factor)

    return controls


cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Kamera ne obnaruzhena")
    exit()

print("=== Instruktsiya ===")
print("1. Navedite kameru na ob'ekt i vvedite '3' dlya kalibrovki cveta")
print("2. Upravlenie budet aktivno tol'ko posle kalibrovki")
print("3. Nazhmite 'q' v okne izobrazheniya dlya vykhoda")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Oshibka zakhvata kadra")
            break

        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        status_text = "Gotov k kalibrovke (vvedite '3')" if not calibrated else "Kalibrovka zavershena"
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

                    set_dac(bus1, ADDR_A1, smoothed['right_gas'])
                    set_dac(bus1, ADDR_A2, smoothed['left_gas'])
                    set_dac(bus2, ADDR_B1, smoothed['right_brake'])
                    set_dac(bus2, ADDR_B2, smoothed['left_brake'])

                    control_text = (
                        f"Pravyi: gaz {smoothed['right_gas']:.0f}%, tormoz {smoothed['right_brake']:.0f}% | "
                        f"Levyi: gaz {smoothed['left_gas']:.0f}%, tormoz {smoothed['left_brake']:.0f}%")
                    print(control_text)

                    cv2.putText(frame, control_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    found = True

                    cv2.rectangle(frame, (x, y), (x + rw, y + rh), (0, 255, 0), 2)
                    cv2.putText(frame, f"X: {nx:.2f}, Y: {ny:.2f}", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    break

            if not found:
                controls = {'right_gas': 0, 'left_gas': 0, 'right_brake': 0, 'left_brake': 0}
                smoothed = apply_smoothing(controls)
                set_dac(bus1, ADDR_A1, 0)
                set_dac(bus1, ADDR_A2, 0)
                set_dac(bus2, ADDR_B1, 0)
                set_dac(bus2, ADDR_B2, 0)
                print("Ob'ekt ne naiden - vse vykhody 0%")

        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.imshow("Camera Tracking", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            cmd = input().strip()
            if cmd == "3":
                center_hsv = hsv[cy, cx]
                h_val, s_val, v_val = int(center_hsv[0]), int(center_hsv[1]), int(center_hsv[2])
                print(f"Kalibrovka po tsvetu HSV: {center_hsv}")

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

                print(f"Novyi HSV diapazon:")
                print(f"   Nizhnii: {lower_hsv}")
                print(f"   Verhnii: {upper_hsv}")
                calibrated = True

except KeyboardInterrupt:
    print("\nZaversheno pol'zovatelem (Ctrl+C)")

set_dac(bus1, ADDR_A1, 0)
set_dac(bus1, ADDR_A2, 0)
set_dac(bus2, ADDR_B1, 0)
set_dac(bus2, ADDR_B2, 0)
bus1.close()
bus2.close()
cap.release()
cv2.destroyAllWindows()