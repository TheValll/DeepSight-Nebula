from ultralytics import YOLO

# Load a YOLO
model = YOLO("yolov8s-worldv2.pt")

# Export the model to NCNN format
model.export(format="onnx", imgsz=640) 