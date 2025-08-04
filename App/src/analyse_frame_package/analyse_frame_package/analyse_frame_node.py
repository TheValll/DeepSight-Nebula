import rclpy
from rclpy.node import Node
from get_analysed_frame_package.srv import GetAnlysedFrame
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

            box_center_coords = (float(-0), float(-0))
            if len(results[0].boxes) > 0:
                box = results[0].boxes[0]
                x1, y1, x2, y2 = map(float, box.xyxy[0])
                xc = (x2 - x1) / 2 + x1
                yc = (y2 - y1) / 2 + y1
                box_center_coords = (xc, yc)
                response.success = True
            else:
                response.success = False

            annotated_frame = results[0].plot()
            response.analysed_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            response.x, response.y = box_center_coords

        except Exception as e:
            self.get_logger().error(f"Error in detection: {e}")
            response.success = False
            response.x, response.y = float(-0), float(-0)

        return response

def main(args=None):
    rclpy.init(args=args)
    service_node = AnalyseFrameNode()
    rclpy.spin(service_node)
    service_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
