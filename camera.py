import cv2
import numpy as np

# Показывать ли окно с изображением
show_image = input("Показывать изображение на экране? (1 - да, 0 - нет): ").strip()
show_image = show_image == "1"

# Стартовое значение HSV для зелёного
lower_hsv = np.array([35, 40, 40])
upper_hsv = np.array([85, 255, 255])

# Камера
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

        # Центр кадра
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
            if rw >= 10 and rh >= 100:
                print(f"Координаты области: x={x}, y={y}, w={rw}, h={rh}")
                found = True
                if show_image:
                    cv2.rectangle(frame, (x, y), (x + rw, y + rh), (0, 255, 0), 2)
                break

        if not found:
            print("0")

        # Рисуем красную точку в центре
        if show_image:
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.imshow("Frame", frame)

        # Проверка ввода с клавиатуры
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        # Проверка на ввод из терминала
        import sys, select
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            cmd = input().strip()
            if cmd == "3":
                # Считываем HSV цвет по центру
                center_hsv = hsv[cy, cx]
                h_val, s_val, v_val = int(center_hsv[0]), int(center_hsv[1]), int(center_hsv[2])
                print(f"🔧 Калибровка по цвету HSV: {center_hsv}")

                # Устанавливаем диапазон с допуском
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

cap.release()
if show_image:
    cv2.destroyAllWindows()
