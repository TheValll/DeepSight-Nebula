import cv2

CAMERA_INDEX = 2

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 60)

    if not cap.isOpened():
        print(f"Error with the camera : {CAMERA_INDEX} ")
        return

    window_name = "Open camera"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    while True:
        success, frame = cap.read()
        if not success:
            print("Can't read the frame")
            break

        cv2.imshow(window_name, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
