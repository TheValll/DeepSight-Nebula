import rclpy
from rclpy.node import Node
from get_analysed_frame_package.srv import GetAnlysedFrame
import cv2
import threading
from cv_bridge import CvBridge

class CaptureFrameNode(Node):
    def __init__(self):
        super().__init__("capture_frame_async")
        self.bridge = CvBridge()
        self.client = self.create_client(GetAnlysedFrame, "get_analysed_frame")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        self.future = None
        self.current_frame = None
    def send_request(self, image_cv):
        try:
            request = GetAnlysedFrame.Request()
            request.image = self.bridge.cv2_to_imgmsg(image_cv, encoding="bgr8")
            self.current_frame = image_cv
            self.future = self.client.call_async(request)
            self.future.add_done_callback(self.response_callback)
        except Exception as e:
            self.get_logger().error(f"Error with the image conversion: {e}")

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Coords: x={response.x}, y={response.y}")
                analysed_frame = self.bridge.imgmsg_to_cv2(response.analysed_image, desired_encoding="bgr8")
                cv2.imshow("Analysed Frame", analysed_frame)
                cv2.waitKey(1)
            else:
                self.get_logger().warn("No detection in this frame.")
                if self.current_frame is not None:
                    cv2.imshow("Analysed Frame", self.current_frame)
                    cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

class CaptureFrame:
    def __init__(self):
        self.camera = 0
        self.camera_resolution = (640, 480)
        self.camera_refresh_rate = 60
        self.latest_frame = None
        self.lock = threading.Lock()
        self.capturing = True

        self.cap = cv2.VideoCapture(self.camera, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_resolution[1])

        if not self.cap.isOpened():
            print("Error: Impossible to open the camera.")
            exit()

        self.grapper_thread = None

    def grapper(self):
        while self.capturing:
            ret, frame = self.cap.read()
            if not ret:
                continue
            with self.lock:
                self.latest_frame = frame.copy()

    def start(self):
        print("Turn on detection")
        self.grapper_thread = threading.Thread(target=self.grapper, daemon=True)
        self.grapper_thread.start()

    def cleanup(self):
        print("Turn off and cleanup detection")
        self.capturing = False
        if self.grapper_thread:
            self.grapper_thread.join()
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    get_analysed_frame_client = CaptureFrameNode()
    robot_detection = CaptureFrame()
    robot_detection.start()

    try:
        while rclpy.ok() and robot_detection.capturing:
            with robot_detection.lock:
                if robot_detection.latest_frame is not None:
                    frame_to_send = robot_detection.latest_frame.copy()
                    get_analysed_frame_client.send_request(frame_to_send)

            rclpy.spin_once(get_analysed_frame_client, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass

    robot_detection.cleanup()
    get_analysed_frame_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
