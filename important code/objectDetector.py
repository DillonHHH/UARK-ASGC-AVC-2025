import math
import ultralytics
from torch import Tensor
from typing import Union, Self
from dataTypes import *
from cameraInterface import *
import constants


@dataclass
class Box:
    """Used to store coordinates of the box drawn around objects detected by YOLO"""

    x: int
    y: int
    width: int
    height: int

    def get_center(self) -> tuple[int, int]:
        return (
            math.floor(self.x + (self.width / 2)),
            math.floor(self.y + (self.height / 2)),
        )

    def is_overlapping(self, b: Self) -> bool:
        """Returns true if the box passed in overlaps with the current one"""
        return (
            self.x < (b.x + b.width)
            and b.x < (self.x + self.width)
            and self.y < (b.y + b.height)
            and b.y < (self.y + self.height)
        )


@dataclass
class Detection:
    """Stored the important attributes of a detected object"""

    classification: str
    confidence: float
    box: Box


class ObjectDetector:
    def __init__(self) -> None:
        self.model = ultralytics.YOLO("best.pt")
        self.model.to("cuda")

    def get_detected_objects(self, frame: Frame) -> list[Detection]:

        results = self.model(frame.color_image)

        # Get the image dimensions
        (H, W) = frame.color_image.shape[:2]

        # # Create a list to store the detected objects
        objects: list[Detection] = []

        # Loop through the outputs
        for result in results:
            names = [result.names[cls.item()] for cls in result.boxes.cls.int()]
            confs = result.boxes.conf
            boxes: list[Tensor] = result.boxes.xywh
            # Loop through the detections
            for index, box in enumerate(boxes):
                # Get the scores, class ID, and confidence
                # scores = box[5:]
                # classID = np.argmax(scores)
                # confidence = scores[classID]

                # Filter out weak predictions
                if confs[index] > 0.5:
                    # Get the bounding box coordinates
                    box = box[0:4].cpu().numpy()
                    (centerX, centerY, width, height) = box.astype("int")

                    # Calculate the top-left and bottom-right coordinates
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    # Append the detected object to the list
                    objects.append(
                        Detection(names[index], confs[index], Box(x, y, width, height))
                    )

        return objects

    @staticmethod
    def get_angle_degrees_from_pixel(
        frame: Frame, pixel: tuple[int, int]
    ) -> Union[float, None]:
        """Returns a value from -CAMERA_FOV_DEGREES/2 to CAMERA_FOV_DEGREES/2, where the center of the screen is 0 degrees"""
        resolution_x, _, _ = frame.color_image.shape

        depth: float = RealsenseCameraInterface.get_depth_from_pixel(frame, pixel)

        if depth == 0:  # Avoid invalid depth readings
            return None

        # Compute horizontal angle relative to optical center
        angle = (
            (pixel[0] - resolution_x / 2) / resolution_x
        ) * constants.CAMERA_FOV_DEGREES

        return angle

    @staticmethod
    def get_cartesian_location_from_pixel(
        frame: Frame, pixel: tuple[int, int]
    ) -> Union[tuple[float, float], None]:
        """Returns cartesian location (x,y) on plane parallel to the ground, returns None if depth is 0"""

        depth: float = RealsenseCameraInterface.get_depth_from_pixel(frame, pixel)

        angle = ObjectDetector.get_angle_degrees_from_pixel(frame, pixel)

        if not angle:
            return None

        # Convert from polar to Cartesian coordinates
        x = math.tan(math.radians(angle)) * depth  # Horizontal displacement
        y = depth  # Forward distance

        return (x, y)

    @staticmethod
    def get_landmark_color(classification: str) -> Union[str, None]:
        print(f"handling classification {classification}")
        if "red" in classification.lower():
            return "red"
        if "blue" in classification.lower():
            return "blue"
        if "yellow" in classification.lower():
            return "yellow"
        if "green" in classification.lower():
            return "green"

        return None
