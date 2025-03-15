from ultralytics import YOLO

model = YOLO('yolo12n.pt')
# Josh's Turing
results = model.train(data='/home/jhkeselman/cleaningoceanplastics_resources/datasets/version1/data.yaml', epochs=100, imgsz=640)
# Josh's Windows
# results = model.train(data='D:/cleaningoceanplastics_resources/datasets/version1/data.yaml', epochs=100, imgsz=640)
