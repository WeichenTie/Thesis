import pyrealsense2 as rs
import cv2
from ultralytics import YOLOWorld
import numpy as np

# Load YOLOvWorld model (adjust path and class names accordingly)
model = YOLOWorld("yolov8x-worldv2.pt")  # or path to your trained weights
model.set_classes(['navel orange'])

model.conf = 0.3  # Confidence threshold

# Configure RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert to numpy image
        color_image = np.asanyarray(color_frame.get_data())

        # Run YOLO inference
        results = model(color_image)

        # Draw results
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0]
                label = result.names[int(box.cls[0])]
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image, f"{label} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show image
        cv2.imshow("YOLOvWorld + RealSense", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()