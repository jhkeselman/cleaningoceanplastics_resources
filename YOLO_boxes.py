from ultralytics import YOLO
import cv2

model = YOLO('yolo12n.pt')

cap = cv2.VideoCapture(2)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 0)

    results = model(frame)

    annotated_frame = results[0].plot()

    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        class_label = model.names[int(box.cls[0])]
        raw_conf = box.conf[0]
        conf = 100 * round(raw_conf.item(), 2)
        objectString = f"{class_label}, {conf}, {x1}, {y1}, {x2}, {y2}"
        cv2.circle(annotated_frame, (x1, y1), 5, (0,0,255), -1)
        cv2.circle(annotated_frame, (x2, y2), 5, (255,0,0), -1)
        print(objectString)

    cv2.imshow("YOLO Real-Time Detection", annotated_frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()

