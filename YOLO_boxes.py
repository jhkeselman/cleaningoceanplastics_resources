from ultralytics import YOLO
import cv2

model = YOLO('yolo11n.pt')

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)

    annotated_frame = results[0].plot()

    objectStrings = ""

    for box in results[0].boxes:
        class_label = model.names[int(box.cls[0])]
        raw_conf = box.conf[0]
        conf = 100 * round(raw_conf.item(), 2)
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        objectString = f"{class_label}, {conf}, {x1}, {y1}, {x2}, {y2}"
        if objectStrings:  # Check if the string is not empty
            objectStrings += " $ " + objectString
        else:
            objectStrings = objectString  # First element, no separator

    print(objectStrings)

    cv2.imshow("YOLO Real-Time Detection", annotated_frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()

