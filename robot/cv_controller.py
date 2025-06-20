import time
import pyrealsense2 as rs
import numpy as np
import cv2
import base64
import numpy as np
from ultralytics import YOLOWorld
import math



class RemoteVisionController:
    def __init__(self) -> None:
        self.yolo_world = YOLOWorld("yolov8x-worldv2.pt")
        self.yolo_world.set_classes(['orange fruit', 'ball', 'oranges', 'orange', 'mandarin', 'tangerine', 'fruit', 'navel orange'])
        self.yolo_world.conf = 0.2
    def capture_frame(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        captures = 0;
        try:
            while True and captures < 5:
                # Wait for a coherent pair of frames: depth and color
                frames = self.pipeline.wait_for_frames()

                self.depth_frame = frames.get_depth_frame()
                self.color_frame = frames.get_color_frame()
                
                if not self.depth_frame or not self.color_frame:
                    continue
                captures += 1
            print('Frame Captured')
            # Convert to numpy arrays
            depth_image = np.asanyarray(self.depth_frame.get_data())
            color_image = np.asanyarray(self.color_frame.get_data())

            # Save with timestamp
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            np.savez_compressed(f"./images/capture_{timestamp}.npz", depth=depth_image, color=color_image)

            captures += 1
            print(f'Frame {captures} Captured and Saved: capture_{timestamp}.npz')

        finally:
            self.pipeline.stop()

    def detect_OOI_position(self, retry=True):
        try:
            self.capture_frame()
            img = np.asanyarray(self.color_frame.get_data())
            results = self.yolo_world(img, verbose=False)[0]
            return results.boxes.xywh[0,0:2]
        except:
            if retry:
                return self.detect_OOI_position()
            else:
                return False

    def pixel_to_cam(self, u, v):
        depth = self.depth_frame.get_distance(u, v)
        intrin = self.depth_frame.profile.as_video_stream_profile().intrinsics
        point_cam = rs.rs2_deproject_pixel_to_point(intrin, [u, v], depth)
        return point_cam
        
    def destroy(self):
        pass

    # def test(self):
        
if __name__ == '__main__':
    controller = RemoteVisionController()
    while True:
        controller.capture_frame()
        # controller.detect_OOI_position()