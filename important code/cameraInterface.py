from dataclasses import dataclass
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
from typing import Union
import warnings
from dataTypes import *
import constants


### We were attempting to use these to extract colors from bucket rather than training each color of bucket as it's own label
# BLUE_RATIO = [0.35, 0.35, 0.65]
# RED_RATIO = [0.65, 0.35, 0.35]
# YELLOW_RATIO = [0.375, 0.375, 0.25]

# class_map = json.load(open("coco_classes.json"))


# this should probably be in constants
CAMERA_OFFSET_FROM_LIDAR_CM: tuple[float, float] = (
    4.45,
    26,
)  # (offset to right, offset forward)


@dataclass
class Frame:
    """Simple way to abstract away the attributes of frame data coming from the camera"""

    depth_frame: rs.depth_frame
    color_frame: rs.video_frame
    depth_image: np.ndarray
    color_image: np.ndarray


class RealsenseCameraInterface:

    def __init__(self) -> None:
        self.RESOLUTION = [640, 480]
        while True:
            try:
                self.pipeline = self.get_pipline()
                self.get_frame()  # test to ensure we can get frames
                break
            except Exception as e:
                warnings.warn(f"Camera setup failed due to: {e}\nRetrying...")
                self.disconnect()
                continue

    # Configure depth and color streams
    def get_pipline(self):
        pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        print(f"Camera info: {device.get_info(rs.camera_info.name)}")

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == "RGB Camera":
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        pipeline.start(config)
        return pipeline

    def disconnect(self):
        self.pipeline.stop()

    @staticmethod
    def get_depth_from_pixel(frame: Frame, location: tuple[int, int]) -> float:
        return frame.depth_frame.get_distance(location[0], location[1])

    # def get_pixel_index_from_scan(
    #     self, angle_degrees: float, distance_cm: float
    # ) -> Union[int, None]:
    #     measurement_location: tuple[float, float] = (
    #         math.cos(math.radians(angle_degrees)) * distance_cm,
    #         math.sin(math.radians(angle_degrees)) * distance_cm,
    #     )

    #     delta_x = measurement_location[0] - CAMERA_OFFSET_FROM_LIDAR_CM[0]
    #     delta_y = measurement_location[1] - CAMERA_OFFSET_FROM_LIDAR_CM[1]

    #     # Avoid division by zero by adding a small epsilon
    #     if abs(delta_x) < 1e-6:
    #         delta_x = 1e-6

    #     angle_from_camera_center_degrees: float = math.degrees(
    #         math.atan2(delta_y, delta_x)
    #     )

    #     if abs(angle_from_camera_center_degrees) > CAMERA_FOV_DEGREES / 2:
    #         return None

    #     # Map angle [-FOV/2, FOV/2] to pixel index [0, resolution]
    #     pixel_index = int(
    #         (angle_from_camera_center_degrees + (CAMERA_FOV_DEGREES / 2))
    #         * (self.RESOLUTION[0] / CAMERA_FOV_DEGREES)
    #     )

    #     return pixel_index

    def get_frame(self) -> Union[Frame, None]:
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None

        depth_image: np.ndarray = np.asanyarray(depth_frame.get_data())
        color_image: np.ndarray = np.asanyarray(color_frame.get_data())

        depth_image = cv2.resize(depth_image, self.RESOLUTION)
        color_image = cv2.resize(color_image, self.RESOLUTION)

        return Frame(depth_frame, color_frame, depth_image, color_image)

    def get_pixel_horizontal_distance_from_nearest_edge(self, pixel_location_x: int):
        distance_from_left = pixel_location_x
        distance_from_right = self.RESOLUTION[1] - pixel_location_x
        return (
            distance_from_left
            if distance_from_left > distance_from_right
            else distance_from_right
        )
