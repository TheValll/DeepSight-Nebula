from ultralytics import YOLO

# Load a YOLO
model = YOLO("yolo11n.pt")

# Export the model to NCNN format
model.export(format="ncnn", imgsz=640) 