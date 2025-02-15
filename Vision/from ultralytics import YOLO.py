from ultralytics import YOLO

model = YOLO("AC-640-640-yolov8n.pt")

model.export(format="rknn", name="rk3588", imgsz = (640,640)) 
