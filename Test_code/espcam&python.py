import cv2
from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO("C:\\Users\\James\\OneDrive\\Desktop\\esp32-cam-server\\datasets\\tennis\\result\\weights\\best.pt")  # Replace with your trained YOLO model path

# ESP32-CAM HTTP stream URL
stream_url = "http://172.20.10.3:81/stream"  # Make sure this URL points to the ESP32-CAM video stream

# Open video stream
cap = cv2.VideoCapture(stream_url)

# Check if the stream was successfully opened
if not cap.isOpened():
    print("Unable to connect to ESP32-CAM video stream")
    exit()

while True:
    # Read a frame from the stream
    ret, frame = cap.read()
    if not ret:
        print("Failed to retrieve frame from video stream, trying to reconnect...")
        cap.release()
        cap = cv2.VideoCapture(stream_url)
        continue

    # Run YOLOv8 model for object detection
    results = model(frame)

    # Draw detection results
    annotated_frame = results[0].plot()

    # Display results
    cv2.imshow("ESP32-CAM Detection", annotated_frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
