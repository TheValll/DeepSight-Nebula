def angles_correction(obj_coords, img_resolution, camera_fov_deg):
    try:
        x, y = float(obj_coords[0]), float(obj_coords[1])
        width, height = float(img_resolution[0]), float(img_resolution[1])
        fov_x, fov_y = float(camera_fov_deg[0]), float(camera_fov_deg[1])
    except (ValueError, TypeError, IndexError):
        return (0, 0)

    center_x = width / 2
    center_y = height / 2

    dx = x - center_x
    dy = y - center_y

    angle_per_pixel_x = fov_x / width
    angle_per_pixel_y = fov_y / height

    angle_x = int(dx * angle_per_pixel_x)
    angle_y = int(dy * angle_per_pixel_y)

    return (angle_x, angle_y)
