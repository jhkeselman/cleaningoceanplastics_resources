from ultralytics import YOLO

model = YOLO('yolo12n.pt')
results = model.train(data="/home/jhkeselman/cleaningoceanplastics_resources/datasets/version1/data.yaml", epochs=100, imgsz=640)