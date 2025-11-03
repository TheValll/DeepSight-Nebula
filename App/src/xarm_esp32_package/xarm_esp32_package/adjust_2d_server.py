import os
import cv2
import rclpy
import time
import threading
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from get_angle_correction_package.srv import GetAngleCorrection
from xarm_esp32_interfaces.action import Adjust2D
from ultralytics import YOLO
from cv_bridge import CvBridge

model_path = os.path.join(os.path.dirname(__file__), "yolov8s.onnx")

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class CameraYoloNode(Node):
    def __init__(self):
        super().__init__("camera_yolo_node")
        self.bridge = CvBridge()
        self.get_logger().info("Loading YOLO model (ONNX / CPU)...")
        self.model = YOLO(model_path)
        self.classes = [32]
        self.client = self.create_client(GetAngleCorrection, "get_angle_correction")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_angle_correction service...")
        self.adjust_2d = ActionClient(self, Adjust2D, "adjust_2d")
        while not self.adjust_2d.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for adjust 2d server service...")
        self.latest_frame = None
        self.lock = threading.Lock()
        self.capturing = True
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            self.get_logger().error("Impossible to open the camera.")
            exit()
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        self.frame_count = 0
        self.fps = 0.0
        self.last_time = time.time()
        self.pid_x = PIDController(kp=0.5, ki=0.01, kd=0.1)
        self.pid_y = PIDController(kp=0.5, ki=0.01, kd=0.1)
        self.get_logger().info("Camera + YOLO node started.")

    def _capture_loop(self):
        while self.capturing:
            ret, frame = self.cap.read()
            if not ret:
                continue
            with self.lock:
                self.latest_frame = frame.copy()

    def process_frame(self):
        frame = None
        with self.lock:
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()
        if frame is None:
            return
        results = self.model.predict(
            source=frame,
            stream=False,
            verbose=False,
            classes=self.classes
        )
        detected = False
        if len(results[0].boxes) > 0:
            box = results[0].boxes[0]
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            xc = (x2 - x1) / 2 + x1
            yc = (y2 - y1) / 2 + y1
            detected = True
            self.send_angle_request(xc, yc)
        annotated_frame = results[0].plot()
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_time
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_time = current_time
        fps_text = f"FPS: {self.fps:.2f}"
        cv2.rectangle(annotated_frame, (5, 5), (120, 35), (0, 0, 0), -1)
        cv2.putText(
            annotated_frame,
            fps_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )
        cv2.imshow("DeppSight Nebula Detection", annotated_frame)
        cv2.waitKey(1)
        if not detected:
            self.get_logger().warn("No object detected.")

    def send_angle_request(self, x, y):
        try:
            request = GetAngleCorrection.Request()
            request.x_object = x
            request.y_object = y
            future = self.client.call_async(request)
            future.add_done_callback(self.angle_response_callback)
        except Exception as e:
            self.get_logger().error(f"Error sending angle correction request: {e}")

    def angle_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Angle correction: x={response.x_correction}, y={response.y_correction}"
                )
                if abs(response.x_correction) < 0.5 and abs(response.y_correction) < 0.5:
                    self.get_logger().error("Launching the distance package...")
                    pass # For later
                else:
                    self.send_goal(response.x_correction, response.y_correction)
            else:
                self.get_logger().warn("No angle correction available.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def send_goal(self, x_correction, y_correction):
        goal = Adjust2D.Goal()
        dt = 1.0  # Time step for PID controller
        x_correction = self.pid_x.compute(x_correction, dt)
        y_correction = self.pid_y.compute(y_correction, dt)
        goal.x_correction = x_correction
        goal.y_correction = y_correction
        self.adjust_2d.send_goal_async(goal).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle: ClientGoalHandle = future.result()
        if self.goal_handle.accepted:
            self.goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(str(result.success))

    def cleanup(self):
        self.capturing = False
        self.capture_thread.join()
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CameraYoloNode()
    try:
        while rclpy.ok():
            node.process_frame()
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
