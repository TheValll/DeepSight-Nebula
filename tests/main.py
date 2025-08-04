from detection import Detection
from angle_calculation import AngleCalculation

class Main:
    def __init__(self):
        self.robot_detection = Detection()
        self.robot_detection.on_detection_callback = self.handle_detection
        self.camera_resolution = self.robot_detection.camera_resolution
        self.camera_fov_diag = self.robot_detection.camera_fov_diag

    # Wait for a detection to calculate Robot angles adjustement
    def handle_detection(self, coords):
        angle_calc = AngleCalculation(coords, self.camera_resolution, self.camera_fov_diag)
        angles = angle_calc.calculate_angle_correction ()
        print(f"Detected object coords : {coords}, Robot angles adjustement : {angles}")

        if angle_calc.is_close_to_zero(angles, threshold=0.5):
            print("Z aze")
            self.robot_detection.stop_prediction() # Stop prediction
            return

        print("Move X Y robot axex") # Dev later

    def start_detection(self):
        self.robot_detection.start()

if __name__ == "__main__":
    main = Main()
    main.start_detection()
