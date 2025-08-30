import time
import pyrealsense2 as rs
import numpy as np
import cv2
import math
from ultralytics import YOLOWorld


class RemoteVisionController:
    def __init__(self) -> None:
        self.yolo_world = YOLOWorld("yolov8x-worldv2.pt")
        self.yolo_world.set_classes(['orange'])
        self.yolo_world.conf = 0.2

        # Start the pipeline once
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)

        # Get color sensor
        color_sensor = self.profile.get_device().query_sensors()[1]

        # Enable auto exposure and auto white balance
        color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        color_sensor.set_option(rs.option.enable_auto_white_balance, 1)

        # Warm-up
        print("[RemoteVisionController] Warming up camera...")
        for _ in range(30):
            self.pipeline.wait_for_frames()
        print("[RemoteVisionController] Ready.")

    def capture_frame(self):
        # Grab a frame
        frames = self.pipeline.wait_for_frames()
        self.depth_frame = frames.get_depth_frame()
        self.color_frame = frames.get_color_frame()

        if not self.depth_frame or not self.color_frame:
            print("[RemoteVisionController] Frame capture failed.")
            return False

        # Convert to numpy arrays
        depth_image = np.asanyarray(self.depth_frame.get_data())
        color_image = np.asanyarray(self.color_frame.get_data())

        # Save with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        np.savez_compressed(f"./images/capture_{timestamp}.npz", depth=depth_image, color=color_image)

        print(f"[RemoteVisionController] Frame captured and saved: capture_{timestamp}.npz")
        return True

    def detect_OOI_positions(self, retry=True):
        try:
            success = self.capture_frame()
            if not success:
                raise RuntimeError("No frame captured")

            img = np.asanyarray(self.color_frame.get_data())
            results = self.yolo_world(img, verbose=False)[0]
            centers = results.boxes.xywh[:, 0:2].cpu().numpy()
            return centers

        except Exception as e:
            print(f"[RemoteVisionController] Detection failed: {e}")
            if retry:
                return self.detect_OOI_positions()
            else:
                return False
            
    
    def detect_OOI_position(self, retry=True, center_radius=320):
        """
        Detects the first object of interest.
        Returns xywh center coordinates if within the specified radius from the image center.
        Raises ValueError if not within radius.
        """
        try:
            success = self.capture_frame()
            if not success:
                raise RuntimeError("No frame captured")

            img = np.asanyarray(self.color_frame.get_data())
            results = self.yolo_world(img, verbose=False)[0]

            if results.boxes.xywh.shape[0] == 0:
                raise ValueError("No detections found")

            # Get the center (x, y) of the first box
            xywh = results.boxes.xywh[0]
            center_x, center_y = xywh[0], xywh[1]

            # Image center
            img_height, img_width = img.shape[:2]
            image_center_x = img_width / 2
            image_center_y = img_height / 2

            # Distance to image center
            dist = math.hypot(center_x - image_center_x, center_y - image_center_y)

            if dist <= center_radius:
                return xywh
            else:
                raise ValueError(
                    f"Detected object is too far from center (distance={dist:.2f}, radius={center_radius})"
                )

        except Exception as e:
            print(f"[detect_OOI_position] {e}")
            if retry:
                return self.detect_OOI_position(retry=retry, center_radius=center_radius)
            else:
                raise


    def pixel_to_cam(self, u, v):
        depth = self.depth_frame.get_distance(u, v)
        intrin = self.depth_frame.profile.as_video_stream_profile().intrinsics
        point_cam = rs.rs2_deproject_pixel_to_point(intrin, [u, v], depth)
        return point_cam

    def destroy(self):
        self.pipeline.stop()
        print("[RemoteVisionController] Pipeline stopped.")


if __name__ == '__main__':
    controller = RemoteVisionController()
    while True:
        controller.capture_frame()
        time.sleep(1)
