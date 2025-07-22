ultralytics[export]import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

# Picam setup
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1280, 1280)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Init YOLOv8
model = YOLO("yolov11n.pt")

while True:
    frame = picam2.capture_array()
    results = model(frame)
    annotated_frame = results[0].plot()
    inference_time = results[0].speed['inference']
    fps = 1000 / inference_time 
    text = f'FPS: {fps:.1f}'

    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size = cv2.getTextSize(text, font, 1, 2)[0]
    text_x = annotated_frame.shape[1] - text_size[0] - 10
    text_y = text_size[1] + 10 

    cv2.putText(annotated_frame, text, (text_x, text_y), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow("Camera", annotated_frame)

    if cv2.waitKey(1) == ord("q"):
        break

cv2.destroyAllWindows()