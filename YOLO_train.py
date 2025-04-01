from ultralytics import YOLO

model = YOLO('/home/jhkeselman/cleaningoceanplastics_resources/runs/detect/train11/weights/bestv11.pt')
# Josh's Turing
results = model.train(data='/home/jhkeselman/cleaningoceanplastics_resources/datasets/version6/data.yaml', epochs=500, imgsz=640, device=[0,1])
# Josh's Windows
# results = model.train(data='D:/cleaningoceanplastics_resources/datasets/version1/data.yaml', epochs=100, imgsz=640)
