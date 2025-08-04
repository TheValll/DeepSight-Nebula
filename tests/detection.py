import cv2
import time
import torch
import threading
from ultralytics import YOLO

class Detection:
    def __init__(self):
        self.device = 0 if torch.cuda.is_available() else 1
        print(f"Use : {self.device}")
        self.model = YOLO("yolov8x.engine")
        self.camera = 0
        self.camera_resolution = (640, 480)
        self.camera_refresh_rate = 60
        self.camera_fov_diag = 78
        self.classes = [32]
        self.latest_frame = None
        self.lock = threading.Lock()
        self.capturing = True   
        self.predicting = True   
        self.cap = cv2.VideoCapture(self.camera, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_resolution[1])
        self.on_detection_callback = None

        if not self.cap.isOpened():
            print("Error: Impossible to open the camera.")
            exit()

    def grapper(self):
        while self.capturing:
            ret, frame = self.cap.read()
            if not ret:
                continue
            with self.lock:
                self.latest_frame = frame.copy()

    def predict(self):
        prev_time = 0
        while self.predicting:
            with self.lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None
            if frame is None:
                continue

            results = self.model.predict(source=frame, device=self.device, stream=False, verbose=False, classes=self.classes)

            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    xc = int((x2 - x1) / 2 + x1)
                    yc = int((y2 - y1) / 2 + y1)

                    box_center_coords = (xc, yc)

                    if self.on_detection_callback:
                        self.on_detection_callback(box_center_coords)

            annotated_frame = results[0].plot()

            # Calculate fps
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if prev_time else 0
            prev_time = curr_time

            # Display predicted fps
            text = f"FPS : {fps:.0f}/{self.camera_refresh_rate}"
            font, scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 1, 2
            color_text, color_bg = (255, 255, 255), (0, 0, 0)
            pos = (10, 30)
            (text_w, text_h), base = cv2.getTextSize(text, font, scale, thickness)

            cv2.rectangle(annotated_frame, (pos[0], pos[1] - text_h - base), (pos[0] + text_w, pos[1] + base), color_bg, cv2.FILLED)
            cv2.putText(annotated_frame, text, pos, font, scale, color_text, thickness)
            cv2.imshow("DeepSight Nebula Detection", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cleanup()
                break

    def start(self):
        print("Turn on detection")
        grapper_thread = threading.Thread(target=self.grapper)
        predict_thread = threading.Thread(target=self.predict)

        grapper_thread.start()
        predict_thread.start()

        grapper_thread.join()
        predict_thread.join()

    def stop_prediction(self):
        print("Stopping prediction")
        self.predicting = False

    def cleanup(self):
        print("Turn off and cleanup detection")
        self.capturing = False
        self.predicting = False
        self.cap.release()
        cv2.destroyAllWindows()
