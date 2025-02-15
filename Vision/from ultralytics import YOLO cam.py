from ultralytics import YOLO

model = YOLO("C:/AI stuff/AC-640-640-yolov8n.pt")

results = model(source = 0, show = True, save = True)