from ultralytics import YOLO
import cv2
import math
import time
from test_angle_correction import angles_correction
from test_arduino_servo_commands import arduino_connect, arduino_disconnect, arduino_send_command

model = YOLO("../yolov8l-worldv2.pt")

custom_classes = ["cup3"]
model.set_classes(custom_classes)

cap_resolution = (1280, 720)
camera_fov_x = 78
camera_fov = (camera_fov_x, camera_fov_x * cap_resolution[1] / cap_resolution[0])

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(3, cap_resolution[0])
cap.set(4, cap_resolution[1])   

arduino_connect()

fps_target = 60
frame_time = 1 / fps_target
prev_time = time.time()

while True:
    current_time = time.time()
    elapsed = current_time - prev_time
    if elapsed < frame_time:
        time.sleep(frame_time - elapsed)
    prev_time = time.time()

    success, img = cap.read()
    if not success:
        break
    
    img = cv2.flip(img, 1) 
    results = model(img, stream=True)

    for r in results:
        boxes = r.boxes
        names = r.names
        probs = r.probs

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            xc = int((x2 - x1) / 2 + x1)
            yc = int((y2 - y1) / 2 + y1)
            obj_center_coords = (xc, yc)

            motor_angles_correction  = angles_correction(obj_center_coords, cap_resolution, camera_fov)
            arduino_send_command(motor_angles_correction)

            confidence = round(float(box.conf[0]), 2)
            cls_id = int(box.cls[0])
            cls_name = custom_classes[cls_id] if cls_id < len(custom_classes) else "N/A"

            print(f"Class: {cls_name} | Confidence: {confidence} | Coordinates: {x1, y1, x2, y2} | Center: {obj_center_coords} | Angle Correction: X: {motor_angles_correction[0]} Y: {motor_angles_correction[1]}")

            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.circle(img, obj_center_coords, 5, (0, 0, 255), -1)
            cv2.putText(img, f"{cls_name} {confidence}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

    cv2.imshow('YOLO-World Webcam', img)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
arduino_disconnect()
