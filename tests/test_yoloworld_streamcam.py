import cv2
from ultralytics import YOLO

# Webcam setup (UVC-compatible)
cap = cv2.VideoCapture(0)  # 0 = première caméra trouvée
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1280)

# Init YOLOv8
model = YOLO("yolo11n.torchscript")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Erreur : impossible de lire depuis la webcam.")
        break

    results = model(frame)
    annotated_frame = results[0].plot()

    # Calcul FPS à partir du temps d'inférence
    inference_time = results[0].speed["inference"]  # en millisecondes
    fps = 1000 / inference_time if inference_time else 0
    text = f"FPS: {fps:.1f}"

    # Afficher le texte dans le coin supérieur droit
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size = cv2.getTextSize(text, font, 1, 2)[0]
    text_x = annotated_frame.shape[1] - text_size[0] - 10
    text_y = text_size[1] + 10

    cv2.putText(annotated_frame, text, (text_x, text_y), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow("Camera", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Libération des ressources
cap.release()
cv2.destroyAllWindows()