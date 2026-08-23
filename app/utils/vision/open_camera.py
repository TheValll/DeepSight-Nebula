import cv2

CAMERA_INDEX = 2

WINDOW_WIDTH = 640
WINDOW_HEIGHT = 360

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 60)

    if not cap.isOpened():
        print(f"Error with the camera: {CAMERA_INDEX}")
        return

    window_left = "Left Camera"
    window_right = "Right Camera"
    cv2.namedWindow(window_left, cv2.WINDOW_NORMAL)
    cv2.namedWindow(window_right, cv2.WINDOW_NORMAL)

    while True:
        success, frame = cap.read()
        if not success:
            print("Can't read the frame")
            break

        height, width, _ = frame.shape
        mid = width // 2
        left_frame = frame[:, :mid]
        right_frame = frame[:, mid:]

        left_frame_resized = cv2.resize(left_frame, (WINDOW_WIDTH, WINDOW_HEIGHT))
        right_frame_resized = cv2.resize(right_frame, (WINDOW_WIDTH, WINDOW_HEIGHT))

        cv2.imshow(window_left, left_frame_resized)
        cv2.imshow(window_right, right_frame_resized)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
