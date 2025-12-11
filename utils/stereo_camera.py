import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

if not cap.isOpened():
    print("Cannot open stereo camera")
    exit()

cv2.namedWindow("Left", cv2.WINDOW_NORMAL)
cv2.namedWindow("Right", cv2.WINDOW_NORMAL)

cv2.setWindowProperty("Left", cv2.WND_PROP_TOPMOST, 1)
cv2.setWindowProperty("Right", cv2.WND_PROP_TOPMOST, 1)

cv2.resizeWindow("Left", 640, 640)
cv2.resizeWindow("Right", 640, 640)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Read error")
        break

    h, w, _ = frame.shape
    mid = w // 2

    left  = frame[:, :mid]
    right = frame[:, mid:]

    cv2.imshow("Left", left)
    cv2.imshow("Right", right)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
