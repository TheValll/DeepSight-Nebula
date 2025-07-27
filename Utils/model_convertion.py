from ultralytics import YOLO

# Convert a yolo model to a specific format
# Yolov8x to TensorRT format
model = YOLO("yolov8x.pt")
model.export(format="engine")