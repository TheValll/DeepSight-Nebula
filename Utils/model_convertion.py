from ultralytics import YOLO

# Convert a yolo model to a specific format
# Yolov8x to TensorRT format
model = YOLO("yolov8m.pt")
model.export(format="onnx") 