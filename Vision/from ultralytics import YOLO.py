from ultralytics import YOLO

model = YOLO("C:/AI stuff/AC-640-640-yolov8n.pt")

model.export(format="rknn", name="rk3588") 