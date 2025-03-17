from ultralytics import YOLO

# Load the pretrained YOLOv5 Nano model
model = YOLO("yolov5n.pt")

# Train the model on your custom waffle dataset
results = model.train(data="waffle_data.yaml", epochs=50, imgsz=640, batch=16)

# Optional: Validate the model
model.val(data="path/to/dataset.yaml", imgsz=640)
