from ultralytics import YOLO
import cv2

model = YOLO('/home/jhkeselman/cleaningoceanplastics_resources/runs/detect/train2/weights/best.pt')

cap = cv2.VideoCapture(0)

# Check if the webcam is opened correctly
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO model on the frame
    results = model(frame)

    # Draw bounding boxes on the frame
    annotated_frame = results[0].plot()  # YOLO outputs annotated images with .plot()

    # Display the resulting frame
    cv2.imshow("YOLO Real-Time Detection", annotated_frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()