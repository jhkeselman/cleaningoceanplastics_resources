from ultralytics import YOLO

model = YOLO('yolo11n.pt')
results = model.train(data='/home/jhkeselman/Downloads/dataset/data.yaml', epochs=100, imgsz=640)
model.save('/home/jhkeselman/cleaningoceanplastics_resources')