import rclpy
from rclpy.node import Node
import math
from get_angle_correction_package.srv import GetAngleCorrection

class AngleCorrectionNode(Node):
    def __init__(self):
        super().__init__("angle_correction_async")
        self.service = self.create_service(
            GetAngleCorrection,
            "get_angle_correction",
            self.angle_correction_callback
        )
    
    def angle_correction_callback(self, request, response):
        try:
            self.x = float(request.x_object)
            self.y = float(request.y_object)
            self.width, self.height = float(640), float(480)
            self.camera_fov_diag = 78
            self.camera_fov = self.fov_calculation()
            fov_x, fov_y = self.camera_fov

            center_x = self.width / 2
            center_y = self.height / 2

            dx = self.x - center_x
            dy = self.y - center_y

            angle_per_pixel_x = fov_x / self.width
            angle_per_pixel_y = fov_y / self.height

            angle_x = dx * angle_per_pixel_x
            angle_y = dy * angle_per_pixel_y

            response.x_correction = angle_x
            response.y_correction = angle_y
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error in angle correction: {e}")
            response.success = False
            response.x_correction, response.y_correction = float(0), float(0)

        return response

    def fov_calculation(self):
        diagonal_fov_rad = math.radians(self.camera_fov_diag)
        aspect_ratio = self.width / self.height

        diag_factor = math.sqrt(1 + (1 / aspect_ratio**2))
        fov_x_rad = 2 * math.atan(math.tan(diagonal_fov_rad / 2) / diag_factor)
        fov_y_rad = 2 * math.atan(math.tan(diagonal_fov_rad / 2) / math.sqrt(1 + aspect_ratio**2))

        fov_x_deg = math.degrees(fov_x_rad)
        fov_y_deg = math.degrees(fov_y_rad)

        return (fov_x_deg, fov_y_deg)

def main(args=None):
    rclpy.init(args=args)
    service_node = AngleCorrectionNode()
    rclpy.spin(service_node)
    service_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
