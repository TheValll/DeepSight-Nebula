import cv2

def find_streamcam(max_devices=10):
    for i in range(max_devices):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW) 
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"Camera trouvée à l'index {i}")
                cv2.imshow(f"Camera {i}", frame)
                print("Appuie sur une touche pour continuer...")
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            else:
                print(f"Camera à l'index {i} ne fonctionne pas correctement.")
            cap.release()
        else:
            print(f"Aucune camera à l'index {i}")
    print("Scan terminé.")

if __name__ == "__main__":
    find_streamcam()
