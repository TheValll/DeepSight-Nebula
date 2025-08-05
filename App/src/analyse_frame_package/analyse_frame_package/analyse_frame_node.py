import rclpy
from rclpy.node import Node
from get_analysed_frame_package.srv import GetAnlysedFrame
from get_angle_correction_package.srv import GetAngleCorrection
import torch
from ultralytics import YOLO
from cv_bridge import CvBridge

class AnalyseFrameNode(Node):
    def __init__(self):
        super().__init__("analyse_frame_async")
        self.bridge = CvBridge()
        self.device = 0 if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")
        self.model = YOLO("yolov8n")
        self.classes = [32]
        self.service = self.create_service(
            GetAnlysedFrame,
            "get_analysed_frame",
            self.analyse_frame_callback
        )
        self.client = self.create_client(GetAngleCorrection, "get_angle_correction")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_angle_correction service...")

    def analyse_frame_callback(self, request, response):
        try:
            frame = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")

            results = self.model.predict(
                source=frame,
                device=self.device,
                stream=False,
                verbose=False,
                classes=self.classes
            )

            if len(results[0].boxes) > 0:
                box = results[0].boxes[0]
                x1, y1, x2, y2 = map(float, box.xyxy[0])
                xc = (x2 - x1) / 2 + x1
                yc = (y2 - y1) / 2 + y1
                response.success = True
                self.send_request(xc, yc)
            else:
                response.success = False
                self.get_logger().warn(f"Object not detected:")

            annotated_frame = results[0].plot()
            response.analysed_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

        except Exception as e:
            self.get_logger().error(f"Error in detection: {e}")
            response.success = False

        return response
    
    def send_request(self, x, y):
        try:
            request = GetAngleCorrection.Request()
            request.x_object = x
            request.y_object = y
            self.future = self.client.call_async(request)
            self.future.add_done_callback(self.response_callback)
        except Exception as e:
            self.get_logger().error(f"Error with the 2d coords resquest: {e}")

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Angle correction: x={response.x_correction}, y={response.y_correction}")
            else:
                self.get_logger().warn("No angle correction for now.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    service_node = AnalyseFrameNode()
    rclpy.spin(service_node)
    service_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
