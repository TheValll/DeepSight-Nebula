import math

class AngleCalculation:
    def __init__(self, obj_coords, img_resolution, camera_fov_diag):
        self.obj_coords = obj_coords
        self.img_resolution = img_resolution
        self.camera_fov_diag = camera_fov_diag
        self.camera_fov = self.fov_calculation()

    def calculate_angle_correction(self):
        # Type verification
        try:
            x, y = float(self.obj_coords[0]), float(self.obj_coords[1])
            width, height = float(self.img_resolution[0]), float(self.img_resolution[1])
            fov_x, fov_y = self.camera_fov
        except (ValueError, TypeError, IndexError):
            return (None, None)
        
        # Calculate the image center
        center_x = width / 2
        center_y = height / 2

        # Calculate the distance bewteen the object and the image center
        dx = x - center_x
        dy = y - center_y

        # Calculate the fov/pixel
        angle_per_pixel_x = fov_x / width
        angle_per_pixel_y = fov_y / height

        # Calculate the angle correction with the fov/pixel
        angle_x = dx * angle_per_pixel_x
        angle_y = dy * angle_per_pixel_y

        return (angle_x, angle_y)
    
    # Trigonometry
    def fov_calculation(self):
        diagonal_fov_rad = math.radians(self.camera_fov_diag)

        width, height = self.img_resolution
        aspect_ratio = width / height

        diag_factor = math.sqrt(1 + (1 / aspect_ratio**2))
        fov_x_rad = 2 * math.atan(math.tan(diagonal_fov_rad / 2) / diag_factor)
        fov_y_rad = 2 * math.atan(math.tan(diagonal_fov_rad / 2) / math.sqrt(1 + aspect_ratio**2))

        fov_x_deg = math.degrees(fov_x_rad)
        fov_y_deg = math.degrees(fov_y_rad)

        return (fov_x_deg, fov_y_deg)

    # Is clone to zero ?
    def is_close_to_zero(self, angles, threshold = 0.2):
        angle_x, angle_y = angles
        return abs(angle_x) < threshold and abs(angle_y) < threshold 