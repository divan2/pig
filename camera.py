import cv2
import numpy as np

# Захват изображения с камеры (если камера одна — обычно это индекс 0)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Камера не обнаружена")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Не удалось получить кадр")
        break

    # Преобразуем в оттенки серого
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Применяем порог (255 - белый, 0 - чёрный)
    _, thresh = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)

    # Поиск контуров
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Находим самый большой контур по площади
        largest_contour = max(contours, key=cv2.contourArea)

        # Получаем координаты ограничивающего прямоугольника
        x, y, w, h = cv2.boundingRect(largest_contour)
        print(f"Координаты самой большой белой области: x={x}, y={y}, w={w}, h={h}")

        # Отрисуем прямоугольник (для проверки)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Показать окно (если используете дисплей или удалённый доступ)
    cv2.imshow("Frame", frame)

    # Выход по нажатию клавиши 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
