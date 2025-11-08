import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3200)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)

if not cap.isOpened():
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape

    mid = w // 2
    left = frame[:, :mid]
    right = frame[:, mid:]

    cv2.imshow("Left", left)
    cv2.imshow("Right", right)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
