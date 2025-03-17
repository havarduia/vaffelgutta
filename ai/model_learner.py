from ultralytics import YOLO

# Load a pretrained YOLO model (use a lightweight model for the Nano)
model = YOLO("yolov8n.pt")

# Train on your custom dataset (adjust epochs, image size, and batch as needed)
results = model.train(data="path/to/dataset.yaml", epochs=50, imgsz=640, batch=4)

# Optional: Validate the model
model.val(data="path/to/dataset.yaml", imgsz=640)
