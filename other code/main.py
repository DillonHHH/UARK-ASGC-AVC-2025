from cameraInterface import RealsenseCameraInterface
from lidarInterface import LidarInterface
from objectDetector import ObjectDetector, Detection
import carControls
import math
import time
from enum import Enum
import math
from typing import Callable, Union
from dataTypes import *
import traceback
import logging
import cv2
import pathlib
import shutil
import datetime

ANGLE_CENTERING_TOLERANCE_DEGREES = 3
ORBIT_DISTANCE_METERS = 1.5
BUCKET_START_ACTION_DISTANCE_METERS: float = (
    3  # once a bucket is this close the action for the given bucket will start
)

LIDAR_VALUE_MULTIPLIER = 1000  # mm to m

MIN_LIDAR_SCAN_BUFFER_LENGTH = (
    300  # at least this many scan must be made before processing any scans
)

MIN_WHEEL_ANGLE_DEGREES = 45
MAX_WHEEL_ANGLE_DEGREES = 135
STRAIGHT_WHEEL_ANGLE_DEGREES = 90  # wheels are straight

MIN_CAR_SPEED = 0
MIN_NONZERO_SPEED = 25  # closest allowed speed to 0
MAX_CAR_SPEED = 40


# car won't consider targets above or below these thresholds
MAX_TARGET_DISTANCE = 18
MIN_TARGET_DISTANCE_WHILE_ORBITING = 2

RAMP_TRIGGER_DISTANCE = 1  # once ramp is this close, wheels will straighten
SLEEP_TIME_WHEN_HITTING_RAMP = (
    2  # seconds between starting to hit the ramp and resuming navigation
)

HIT_RAMP: bool = True

DEBUG = False


camera = RealsenseCameraInterface()
# lidar = LidarInterface()
detector = ObjectDetector()
# https://www.geeksforgeeks.org/how-to-log-python-messages-to-both-stdout-and-files/
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.FileHandler("lastRun.log", mode="w"), logging.StreamHandler()],
)

current_speed: int = 0
current_wheel_angle: int = STRAIGHT_WHEEL_ANGLE_DEGREES
carControls.setSpeed(current_speed)
carControls.setRot(current_wheel_angle)

# ------DEBUG ONLY------
last_target_location: tuple[float, float] = (0, 0)
last_function: str = ""
# ----------------------


# TODO ramp down speed the longer an obstacle isn't found while circling a bucket


class OBSTAClE_TYPES(Enum):
    BLUE = "blue"
    RED = "red"
    YELLOW = "yellow"
    GREEN = "green"
    RAMP = "ramp"
    DUMMY = "DUMMY"  # used to add a dummy obstacle that will never be found


@dataclass
class TargetObstacle:
    type: OBSTAClE_TYPES
    max_distance: float


# Buckets in the order they will be encountered mapped to their max distance from the previous obstacle in meters
TARGET_OBSTACLES: list[TargetObstacle] = [
    TargetObstacle(OBSTAClE_TYPES.BLUE, MAX_TARGET_DISTANCE),
    TargetObstacle(OBSTAClE_TYPES.DUMMY, MAX_TARGET_DISTANCE),
]


class DebugVideo:
    def __init__(self, video_path: str, visualize: bool, write_to_file: bool) -> None:
        self.BOTTOM_PADDING_SIZE = (
            75  # Bottom padding is added so that text under boxes is readable
        )

        self.video_writer: cv2.VideoWriter = cv2.VideoWriter(
            video_path,
            cv2.VideoWriter.fourcc("m", "p", "4", "v"),
            30,
            (640, 480 + self.BOTTOM_PADDING_SIZE),
        )
        self.visualize: bool = visualize
        self.write_to_file: bool = write_to_file
        self.previous_frame_time: float = time.time()

    def close(self):
        self.video_writer.release()

    def add_frame(self, frame: Frame, object_detections: list[Detection]):
        image = frame.color_image
        # add padding to bottom
        image = cv2.copyMakeBorder(
            image, 0, self.BOTTOM_PADDING_SIZE, 0, 0, cv2.BORDER_CONSTANT
        )
        current_time = time.time()
        fps = 1 / (
            current_time - self.previous_frame_time
        )  # should probably change this to a moving average
        self.previous_frame_time = current_time
        if not DEBUG:
            cv2.putText(
                image,
                f"fps: {fps}",
                (10, 10),
                cv2.FONT_HERSHEY_COMPLEX,
                0.75,
                (0, 255, 0),
            )
            cv2.putText(
                image,
                f"time: {datetime.datetime.now()}",
                (10, 40),
                cv2.FONT_HERSHEY_COMPLEX,
                0.75,
                (0, 255, 0),
            )
            cv2.putText(
                image,
                f"function: {last_function}",
                (10, 70),
                cv2.FONT_HERSHEY_COMPLEX,
                0.75,
                (0, 255, 0),
            )
            cv2.putText(
                image,
                f"speed: {current_speed}",
                (10, 100),
                cv2.FONT_HERSHEY_COMPLEX,
                0.75,
                (0, 255, 0),
            )
            cv2.putText(
                image,
                f"angle: {current_wheel_angle}",
                (10, 130),
                cv2.FONT_HERSHEY_COMPLEX,
                0.75,
                (0, 255, 0),
            )
            cv2.putText(
                image,
                f"target location: {(round(last_target_location[0], 5), round(last_target_location[1], 5))}",
                (10, 150),
                cv2.FONT_HERSHEY_COMPLEX,
                0.75,
                (0, 255, 0),
            )

            for detection in object_detections:
                cv2.rectangle(
                    image,
                    (detection.box.x, detection.box.y),
                    (
                        detection.box.x + detection.box.width,
                        detection.box.y + detection.box.height,
                    ),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    image,
                    detection.classification,
                    (detection.box.x, detection.box.y + detection.box.height + 10),
                    cv2.FONT_HERSHEY_COMPLEX,
                    0.75,
                    (0, 255, 0),
                )
                location = detector.get_cartesian_location_from_pixel(
                    frame, detection.box.get_center()
                )
                if location:
                    cv2.putText(
                        image,
                        f"{(round(location[0], 5), round(location[1], 5))}",
                        (detection.box.x, detection.box.y + detection.box.height + 40),
                        cv2.FONT_HERSHEY_COMPLEX,
                        0.75,
                        (0, 255, 0),
                    )

        if self.write_to_file:
            self.video_writer.write(image)
        if self.visualize:
            cv2.imshow("DebugVideo", image)
            cv2.waitKey(1)


if DEBUG:
    debug_video = DebugVideo("extras 1.mp4", False, True)
else:
    debug_video: DebugVideo = DebugVideo("lastRun.mp4", False, True)

logger = logging.getLogger("main")


# def get_full_scan_buffer():

#     scan_buffer: list[DAQ] = []
#     while len(scan_buffer) < MIN_LIDAR_SCAN_BUFFER_LENGTH:
#         scan_buffer += lidar.get_scans(
#             callback_on_fail=lambda: carControls.setSpeed(0),
#             callback_on_restart=lambda: carControls.setSpeed(MIN_NONZERO_SPEED),
#         )
#     return [
#         scan for scan in scan_buffer if scan.distance > 0
#     ]  # throw out invalid scans


def orbit(next_obstacle: TargetObstacle, clockwise: bool, orbiting_ramp: bool = False):
    """Orbits an obstacle while looking for the next obstacle, returns upon finding next obstacle. Does not change car speed."""
    # if clockwise:
    #     set_wheel_angle(45)
    # else:
    #     set_wheel_angle(135)

    # LIDAR_ANGLE_RANGE = (80, 100) if clockwise else (260, 280)

    # while True:
    #     logger.debug("Filling scan Buffer")
    #     scan_buffer = get_full_scan_buffer()
    #     closest_scan_in_range: DAQ = DAQ(math.inf,125 0, 0)
    #     for scan in scan_buffer:
    #         if scan.distance < closest_scan_in_range.distance:
    #             closest_scan_in_range = scan
    #     if (
    #         closest_scan_in_range.angle_degrees > LIDAR_ANGLE_RANGE[0]
    #         and closest_scan_in_range.angle_degrees < LIDAR_ANGLE_RANGE[1]
    #     ):
    #         logger.debug(
    #             f"Closest scan was in range {LIDAR_ANGLE_RANGE[0]}-{LIDAR_ANGLE_RANGE[1]}: {closest_scan_in_range}, breaking"
    #         )
    #         break
    #     else:
    #         logger.debug(
    #             f"Closest scan was not in range {LIDAR_ANGLE_RANGE[0]}-{LIDAR_ANGLE_RANGE[1]}: {closest_scan_in_range}"
    #         )

    # while True:
    #     logger.debug("Filling scan Buffer")
    #     scan_buffer = get_full_scan_buffer()
    #     closest_scan_in_range: DAQ = DAQ(math.inf, 0, 0)
    #     for scan in scan_buffer:
    #         if (
    #             scan.angle_degrees > LIDAR_ANGLE_RANGE[0]
    #             and scan.angle_degrees < LIDAR_ANGLE_RANGE[1]
    #             and scan.distance < closest_scan_in_range.distance
    #         ):
    #             closest_scan_in_range = scan
    #     if closest_scan_in_range.distance < 5:
    #         logger.debug(
    #             f"Closest scan was in range {LIDAR_ANGLE_RANGE[0]}-{LIDAR_ANGLE_RANGE[1]}: {closest_scan_in_range}, breaking"
    #         )
    #         break
    #     else:
    #         logger.debug(
    #             f"Closest scan was not in range {LIDAR_ANGLE_RANGE[0]}-{LIDAR_ANGLE_RANGE[1]}: {closest_scan_in_range}"
    #         )

    # if not orbiting_ramp:
    #     start_time = time.time()
    #     while time.time() - start_time < 0.5:
    #         # set_wheel_angle(55 if clockwise else 125)
    #         set_wheel_angle(90)
    #         carControls.setRot(current_wheel_angle)
    #         time.sleep(0.05)

    while True:
        # scan_buffer = get_full_scan_buffer()
        # If the lidar has filled the buffer with all 360 degrees
        # Try to get a "frame" from the camera
        frame: Union[Frame, None] = camera.get_frame()
        if frame:
            # If the camera has provided a frame
            # Try to detect an object with the object detection
            target_object: Union[Detection, None] = get_target_object(
                frame,
                detector.get_detected_objects(frame),
                next_obstacle,
                min_target_distance=MIN_TARGET_DISTANCE_WHILE_ORBITING,
            )
            if target_object:
                # If there is an object and the object matches the next obstacle being looked for
                # Return the "frame"
                return frame

        # Else initialize 'closest_lidar_scan' to the max distance
        # closest_lidar_scan: DAQ = DAQ(math.inf, 0, 0)

        # for scan in scan_buffer:
        #     if not closest_lidar_scan or scan.distance < closest_lidar_scan.distance:
        #         closest_lidar_scan = scan

        # logger.debug(f"Closest lidar scan: {closest_lidar_scan}")

        # bucket_x, bucket_y = closest_lidar_scan.to_relative_location()

        # logger.debug(f"Closest lidar scan location: {(bucket_x, bucket_y)}")

        if not orbiting_ramp:
            set_wheel_angle(120 if clockwise else 65)
        else:
            set_wheel_angle(115)
        carControls.setRot(current_wheel_angle)

        # orbit_offset_degrees = -90 if clockwise else 90

        # # offset the angle of the bucket location relative to the car by 90 degrees to find the location on the circle we want to be
        # orbit_angle_degrees = (
        #     math.degrees(math.atan2(bucket_y, bucket_x)) + orbit_offset_degrees
        # )

        # target_x = bucket_x + ORBIT_DISTANCE_METERS * math.sin(
        #     math.radians(orbit_angle_degrees)
        # )

        # target_y = bucket_y + ORBIT_DISTANCE_METERS * math.cos(
        #     math.radians(orbit_angle_degrees)
        # )

        # logger.debug(f"Target location from cloest lidar scan: {(target_x, target_y)}")

        # _navigate_to_location((target_x, target_y))


def realistic_orbit(next_obstacle: TargetObstacle, clockwise: bool) -> Frame:
    """Orbits an obstacle and returns the frame where it find the object, does not change car speed\n
    Our original orbit function. This actually orbits the target like a planet around a star, resulting in an elliptical circle.\n
    Looks cool, but is not what we want for the course.
    """

    if clockwise:
        set_wheel_angle(45)
    else:
        set_wheel_angle(135)

    DEGREES_PER_ADJUSTMENT: int = 10

    rotate_offset_to_move_closer = (
        DEGREES_PER_ADJUSTMENT if clockwise else -DEGREES_PER_ADJUSTMENT
    )
    rotate_offset_to_move_farther = (
        -DEGREES_PER_ADJUSTMENT if clockwise else DEGREES_PER_ADJUSTMENT
    )

    scan_buffer: list[DAQ] = []

    while True:
        # scan_buffer = get_full_scan_buffer()
        # If the lidar has filled the buffer with all 360 degrees
        # Try to get a "frame" from the camera
        frame: Union[Frame, None] = camera.get_frame()
        if frame:
            # If the camera has provided a frame
            # Try to detect an object with the object detection
            target_object: Union[Detection, None] = get_target_object(
                frame, detector.get_detected_objects(frame), next_obstacle
            )
            if target_object:
                # If there is an object and the object matches the next obstacle being looked for
                # Return the "frame"
                return frame

        # Else initialize 'closest_lidar_scan' to the max distance
        closest_lidar_scan: DAQ = DAQ(math.inf, 0, 0)

        for scan in scan_buffer:
            if not closest_lidar_scan or scan.distance < closest_lidar_scan.distance:
                closest_lidar_scan = scan
        # Checks if the distance from the closest scan is farther than 'ORBIT_DISTANCE_METERS' allows
        if closest_lidar_scan.distance * LIDAR_VALUE_MULTIPLIER > ORBIT_DISTANCE_METERS:
            # Turn closer
            set_wheel_angle(current_wheel_angle + rotate_offset_to_move_closer)
        # Checks if the distance from the closest scan is closer than 'ORBIT_DISTANCE_METERS' allows
        elif (
            closest_lidar_scan.distance * LIDAR_VALUE_MULTIPLIER < ORBIT_DISTANCE_METERS
        ):
            # Turn farther
            set_wheel_angle(current_wheel_angle + rotate_offset_to_move_farther)
        carControls.setRot(current_wheel_angle)
        # Reset 'scan_buffer' for it to be refilled
        scan_buffer = []


# Reserved for driving between the red buckets
def drive_between():

    time.sleep(2)

    # # TODO adjust this to match realistic value
    # PASSED_UNDER_BRIDGE_DISTANCE_METERS: float = (
    #     1  # when both buckets are within this distance, returning from this function will be possible once the buckets are far enough away again
    # )

    # # TODO adjust this to match realistic value
    # MIN_DISTANCE_TO_RETURN: float = 2

    # passed_under_bridge: bool = False

    # scan_buffer: list[DAQ] = []
    # while True:
    #     # scan_buffer = get_full_scan_buffer()
    #     # If the lidar has filled the buffer with all 360 degrees
    #     # Else initialize 'closest_right_bucket_scan' to the max distance
    #     # Else initialize 'closest_left_bucket_scan' to the max distance
    #     closest_right_bucket_scan: DAQ = DAQ(math.inf, 0, 0)
    #     closest_left_bucket_scan: DAQ = DAQ(math.inf, 0, 0)

    #     for scan in scan_buffer:
    #         # right bucket
    #         # Checks right of the car with lidar (between 0 and 180 degrees)
    #         if scan.angle_degrees > 0 and scan.angle_degrees < 180:
    #             if scan.distance < closest_right_bucket_scan.distance:
    #                 closest_right_bucket_scan = scan
    #         # left bucket
    #         # Checks left of the car with lidar (between 180 and 360 degrees)
    #         elif scan.angle_degrees > 180 and scan.angle_degrees < 360:
    #             if scan.distance < closest_left_bucket_scan.distance:
    #                 closest_left_bucket_scan = scan
    #     if (
    #         passed_under_bridge
    #         and closest_left_bucket_scan.distance > MIN_DISTANCE_TO_RETURN
    #         and closest_right_bucket_scan.distance > MIN_DISTANCE_TO_RETURN
    #     ):
    #         # If the car has passed under the bridge and the distance from the left and right bucket is far enough return
    #         return

    #     if (
    #         not passed_under_bridge
    #         and closest_left_bucket_scan.distance < PASSED_UNDER_BRIDGE_DISTANCE_METERS
    #         and closest_right_bucket_scan.distance < PASSED_UNDER_BRIDGE_DISTANCE_METERS
    #     ):
    #         # If the car has not passed under the bridge but is close enough to the buckets to be passing through
    #         # Set 'passed_under_bridge' to true
    #         passed_under_bridge = True

    #     # keep close to the center between the buckets
    #     if closest_left_bucket_scan.distance > closest_right_bucket_scan.distance:
    #         set_wheel_angle(current_wheel_angle - 3)
    #     elif closest_left_bucket_scan.distance < closest_right_bucket_scan.distance:
    #         set_wheel_angle(current_wheel_angle + 3)
    #     carControls.setRot(current_wheel_angle)
    #     scan_buffer = []


def _navigate_to_location(location: tuple[float, float]):
    """Sets the rotation and speed to get the car closer to the target location, should be called inside a loop"""
    global last_target_location

    TURN_DAMPENING_FACTOR = 0.5

    last_target_location = location
    logger.debug(f"navigating to {location}")
    target_location_angle_from_center_degrees = math.degrees(
        math.atan2(
            location[0],
            location[1],
        )
    )
    logger.debug(
        f"target location angle from center: {target_location_angle_from_center_degrees}"
    )

    if (
        abs(target_location_angle_from_center_degrees)
        > ANGLE_CENTERING_TOLERANCE_DEGREES
    ):
        # set_wheel_angle(STRAIGHT_WHEEL_ANGLE_DEGREES - 10)
        set_wheel_angle(
            math.floor(
                STRAIGHT_WHEEL_ANGLE_DEGREES
                + (target_location_angle_from_center_degrees * TURN_DAMPENING_FACTOR)
            )
        )
    # elif target_location_angle_from_center_degrees > ANGLE_CENTERING_TOLERANCE_DEGREES:
    #     # set_wheel_angle(STRAIGHT_WHEEL_ANGLE_DEGREES + 10)
    #     set_wheel_angle(
    #         math.floor(
    #             STRAIGHT_WHEEL_ANGLE_DEGREES + target_location_angle_from_center_degrees
    #         )
    #     )
    else:
        set_wheel_angle(STRAIGHT_WHEEL_ANGLE_DEGREES)
    set_car_speed(current_speed + 5)
    logger.debug(f"setting rot to {current_wheel_angle} and speed to {current_speed}")
    carControls.setSpeed(current_speed)
    carControls.setRot(current_wheel_angle)


def navigate_to_yellow_or_blue_bucket(target_obstacle: TargetObstacle) -> None:
    logger.debug(f"starting navigation to {target_obstacle.type}")

    assert target_obstacle.type in [
        OBSTAClE_TYPES.BLUE,
        OBSTAClE_TYPES.YELLOW,
    ]

    OFFSET_FROM_TARGET_METERS: tuple[float, float] = (
        (-2.5, 0) if target_obstacle.type == OBSTAClE_TYPES.BLUE else (1.5, 0)
    )

    FAILED_DETECTION_TRESHOLD = 4  # After failing to find the target for this many frames, car will begin roaming

    CLOSEST_PREVIOUS_DISTANCE_BEFORE_ORBIT = 5

    closest_previous_distance: float = math.inf

    previous_target_location: tuple[float, float] = (0, 0)

    consecutive_failed_detections = 0
    while True:
        logger.debug("navigating...")
        frame: Union[Frame, None] = camera.get_frame()
        if frame:
            target_object = get_target_object(
                frame, detector.get_detected_objects(frame), target_obstacle
            )
            logger.debug(f"target object found: {target_object}")
            if target_object:
                consecutive_failed_detections = 0
                target_object_location = detector.get_cartesian_location_from_pixel(
                    frame, target_object.box.get_center()
                )
                logger.debug(f"target location: {target_object_location}")
                if target_object_location:
                    offset_target_location = (
                        (target_object_location[0] + OFFSET_FROM_TARGET_METERS[0]),
                        (target_object_location[1] + OFFSET_FROM_TARGET_METERS[1]),
                    )
                    target_distance = math.dist((0, 0), target_object_location)
                    if target_distance < closest_previous_distance:
                        closest_previous_distance = target_distance
                    _navigate_to_location(offset_target_location)
                    previous_target_location = offset_target_location
                    # if target_distance < BUCKET_START_ACTION_DISTANCE_METERS:
                    #     return
            else:
                consecutive_failed_detections += 1
                _navigate_to_location(previous_target_location)
                if consecutive_failed_detections > FAILED_DETECTION_TRESHOLD:
                    if (
                        closest_previous_distance
                        < CLOSEST_PREVIOUS_DISTANCE_BEFORE_ORBIT
                    ):
                        set_wheel_angle(STRAIGHT_WHEEL_ANGLE_DEGREES)
                        carControls.setRot(current_wheel_angle)
                        return
                    logger.warning(
                        f"Entering roam with closest previous distance {closest_previous_distance}"
                    )
                    roam(
                        target_obstacle,
                        spin_left=target_obstacle == OBSTAClE_TYPES.BLUE,
                    )
                    consecutive_failed_detections = 0


def roam(target_obstacle: TargetObstacle, spin_left: bool):
    """car will move around until it finds the target"""

    logger.warning(
        f"Roaming with {target_obstacle}, Spinning {'Left' if spin_left else 'Right'}"
    )

    set_wheel_angle(MIN_WHEEL_ANGLE_DEGREES if spin_left else MAX_WHEEL_ANGLE_DEGREES)
    carControls.setRot(current_wheel_angle)
    set_car_speed(MAX_CAR_SPEED)
    carControls.setSpeed(current_speed)

    while True:
        frame: Union[Frame, None] = camera.get_frame()
        if frame:
            target_object = get_target_object(
                frame, detector.get_detected_objects(frame), target_obstacle
            )
            if target_object:
                set_wheel_angle(STRAIGHT_WHEEL_ANGLE_DEGREES)
                # set_car_speed(0)

                carControls.setRot(current_wheel_angle)
                carControls.setSpeed(current_speed)

                logger.debug("Object found, roaming stopped")
                return


def navigate_to_red_bucket() -> None:

    OFFSET_FROM_TARGET_METERS: tuple[float, float] = (0, -2)

    FAILED_DETECTION_TRESHOLD = 5  # After failing to find two red buckets for this many frames, car will begin roaming

    DISTANCE_BEFORE_RETURNING = 3

    consecutive_failed_detections = 0
    while True:
        frame: Union[Frame, None] = camera.get_frame()
        if frame:
            detected_objects = detector.get_detected_objects(frame)
            red_buckets = [
                bucket
                for bucket in detected_objects
                if OBSTAClE_TYPES.RED.value.lower() in bucket.classification.lower()
            ]

            if len(red_buckets) > 2:
                new_red_buckets = []
                for _ in range(0, 2):
                    lowest_red_bucket = red_buckets[0]
                    for bucket in red_buckets:
                        if bucket.box.y > lowest_red_bucket.box.y:
                            lowest_red_bucket = bucket
                    new_red_buckets.append(lowest_red_bucket)
                    red_buckets.remove(lowest_red_bucket)
                red_buckets = new_red_buckets

            if len(red_buckets) == 2:
                consecutive_failed_detections = 0
                bucket_1_location = detector.get_cartesian_location_from_pixel(
                    frame, red_buckets[0].box.get_center()
                )
                bucket_2_location = detector.get_cartesian_location_from_pixel(
                    frame, red_buckets[1].box.get_center()
                )
                if bucket_1_location and bucket_2_location:
                    location_between_buckets: tuple[float, float] = (
                        (bucket_1_location[0] + bucket_2_location[0]) / 2,
                        (bucket_1_location[1] + bucket_2_location[1]) / 2,
                    )
                    target_location = (
                        (location_between_buckets[0] + OFFSET_FROM_TARGET_METERS[0]),
                        (location_between_buckets[1] + OFFSET_FROM_TARGET_METERS[1]),
                    )
                    _navigate_to_location(target_location)
                    if (
                        math.dist((0, 0), bucket_1_location) < DISTANCE_BEFORE_RETURNING
                        or math.dist((0, 0), bucket_2_location)
                        < DISTANCE_BEFORE_RETURNING
                    ):
                        return
            else:
                consecutive_failed_detections += 1
                if consecutive_failed_detections > FAILED_DETECTION_TRESHOLD:
                    roam(
                        TargetObstacle(OBSTAClE_TYPES.RED, MAX_TARGET_DISTANCE),
                        False,
                    )


def set_wheel_angle(new_angle_degrees: int):
    """Updates current_angle, does not set wheel rotation"""
    global current_wheel_angle
    current_wheel_angle = new_angle_degrees
    if current_wheel_angle > MAX_WHEEL_ANGLE_DEGREES:
        current_wheel_angle = MAX_WHEEL_ANGLE_DEGREES
    elif current_wheel_angle < MIN_WHEEL_ANGLE_DEGREES:
        current_wheel_angle = MIN_WHEEL_ANGLE_DEGREES
    logger.debug(f"Wheel angle set to {current_wheel_angle}")


def set_car_speed(new_speed: int):
    global current_speed
    current_speed = new_speed
    if current_speed > MAX_CAR_SPEED:
        current_speed = MAX_CAR_SPEED
    elif current_speed < MIN_CAR_SPEED:
        current_speed = MIN_CAR_SPEED

    if current_speed > 0 and current_speed < MIN_NONZERO_SPEED:
        current_speed = MIN_NONZERO_SPEED
    logger.debug(f"Wheel speed set to {current_speed}")
    # current_speed = 0


def get_target_object(
    frame: Frame,
    detected_objects: list[Detection],
    target_obstacle: TargetObstacle,
    min_target_distance: float = 0,
):
    logger.debug(f"Getting target: {target_obstacle}")

    debug_video.add_frame(frame, detected_objects)

    closest_target: Union[Detection, None] = None
    closest_target_distance: Union[float, None] = None

    y_coordinate_bounds = (100, 300)

    for detected_object in detected_objects:
        logger.debug(f"Detected targets: {detected_objects}")
        detected_object_center = detected_object.box.get_center()
        if (
            target_obstacle.type.value.lower() in detected_object.classification.lower()
            and detected_object_center[1] > y_coordinate_bounds[0]
            and detected_object_center[1] < y_coordinate_bounds[1]
        ):
            detected_object_location: Union[tuple[float, float], None] = (
                detector.get_cartesian_location_from_pixel(
                    frame, detected_object_center
                )
            )
            if detected_object_location:
                detected_object_distance: float = math.dist(
                    (0, 0), detected_object_location
                )
                if (
                    detected_object_distance > min_target_distance
                    and detected_object_distance < target_obstacle.max_distance
                ):
                    if not closest_target:
                        closest_target = detected_object
                        closest_target_distance = detected_object_distance
                    else:
                        assert closest_target_distance
                        if detected_object_distance < closest_target_distance:
                            closest_target = detected_object
                            closest_target_distance = detected_object_distance
    logger.debug(
        f"Returning closest target: {closest_target} at location {(detector.get_cartesian_location_from_pixel(frame, closest_target.box.get_center())) if closest_target else None} "
    )
    return closest_target


def navigate_to_ramp():

    OFFSET_FROM_TARGET_METERS: tuple[float, float] = (0, -3)

    FAILED_DETECTION_TRESHOLD = 5  # After failing to find two red buckets for this many frames, car will begin roaming

    DISTANCE_BEFORE_RETURN: float = 5

    consecutive_failed_detections = 0

    while True:
        frame: Union[Frame, None] = camera.get_frame()
        if frame:
            target_object = get_target_object(
                frame,
                detector.get_detected_objects(frame),
                TargetObstacle(OBSTAClE_TYPES.RAMP, MAX_TARGET_DISTANCE),
            )

            if target_object:
                consecutive_failed_detections = 0
                target_object_location = detector.get_cartesian_location_from_pixel(
                    frame, target_object.box.get_center()
                )
                if target_object_location:
                    target_location = (
                        (target_object_location[0] + OFFSET_FROM_TARGET_METERS[0]),
                        (target_object_location[1] + OFFSET_FROM_TARGET_METERS[1]),
                    )
                    _navigate_to_location(target_location)
                    if target_location[1] < DISTANCE_BEFORE_RETURN:
                        return
            else:
                consecutive_failed_detections += 1
                if consecutive_failed_detections > FAILED_DETECTION_TRESHOLD:
                    roam(
                        TargetObstacle(OBSTAClE_TYPES.RAMP, MAX_TARGET_DISTANCE),
                        False,
                    )


def run_ramp():

    # scan_buffer: list[DAQ] = []
    # while True:
    #     # scan_buffer = get_full_scan_buffer()

    #     closest_left_side_ramp_scan: DAQ = DAQ(math.inf, 0, 0)
    #     closest_right_side_ramp_scan: DAQ = DAQ(math.inf, 0, 0)
    #     for scan in scan_buffer:
    #         if scan.angle_degrees < 0 and scan.angle_degrees > 300:
    #             if scan.distance < closest_left_side_ramp_scan.distance:
    #                 closest_left_side_ramp_scan = scan
    #         elif scan.angle_degrees > 0 and scan.angle_degrees < 60:
    #             if scan.distance > closest_right_side_ramp_scan.distance:
    #                 closest_right_side_ramp_scan = scan
    #     if closest_left_side_ramp_scan.distance < closest_right_side_ramp_scan.distance:
    #         set_wheel_angle(current_wheel_angle - 3)
    #     elif (
    #         closest_left_side_ramp_scan.distance > closest_right_side_ramp_scan.distance
    #     ):
    #         set_wheel_angle(current_wheel_angle + 3)

    #     carControls.setRot(current_wheel_angle)

    #     if (
    #         closest_left_side_ramp_scan.distance < RAMP_TRIGGER_DISTANCE
    #         or closest_right_side_ramp_scan.distance < RAMP_TRIGGER_DISTANCE
    #     ):
    #         carControls.setRot(STRAIGHT_WHEEL_ANGLE_DEGREES)
    #         time.sleep(SLEEP_TIME_WHEN_HITTING_RAMP)
    #         return

    #     scan_buffer = []

    previous_speed = current_speed
    set_car_speed(40)
    carControls.setSpeed(current_speed)
    set_wheel_angle(90)
    carControls.setRot(current_wheel_angle)
    start_time = time.time()
    while time.time() - start_time < 2:
        carControls.setRot(current_wheel_angle)
        time.sleep(0.05)
    set_car_speed(previous_speed)
    carControls.setSpeed(current_speed)
    set_wheel_angle(90)
    carControls.setRot(current_wheel_angle)


def circumvent_ramp() -> None:
    logger.debug(f"starting navigation to {OBSTAClE_TYPES.RAMP}")

    OFFSET_FROM_TARGET_METERS: tuple[float, float] = (-4, 0)

    FAILED_DETECTION_TRESHOLD = 4  # After failing to find the target for this many frames, car will begin roaming

    CLOSEST_PREVIOUS_DISTANCE_BEFORE_ORBIT = 5

    closest_previous_distance: float = math.inf

    previous_target_location: tuple[float, float] = (0, 0)

    consecutive_failed_detections = 0
    while True:
        logger.debug("navigating...")
        frame: Union[Frame, None] = camera.get_frame()
        if frame:
            target_object = get_target_object(
                frame,
                detector.get_detected_objects(frame),
                TargetObstacle(OBSTAClE_TYPES.RAMP, MAX_TARGET_DISTANCE),
            )
            logger.debug(f"target object found: {target_object}")
            if target_object:
                consecutive_failed_detections = 0
                target_object_location = detector.get_cartesian_location_from_pixel(
                    frame, target_object.box.get_center()
                )
                logger.debug(f"target location: {target_object_location}")
                if target_object_location:
                    offset_target_location = (
                        (target_object_location[0] + OFFSET_FROM_TARGET_METERS[0]),
                        (target_object_location[1] + OFFSET_FROM_TARGET_METERS[1]),
                    )
                    target_distance = math.dist((0, 0), target_object_location)
                    if target_distance < closest_previous_distance:
                        closest_previous_distance = target_distance
                    _navigate_to_location(offset_target_location)
                    previous_target_location = offset_target_location
                    # if target_distance < BUCKET_START_ACTION_DISTANCE_METERS:
                    #     return
            else:
                consecutive_failed_detections += 1
                _navigate_to_location(previous_target_location)
                if consecutive_failed_detections > FAILED_DETECTION_TRESHOLD:
                    if (
                        closest_previous_distance
                        < CLOSEST_PREVIOUS_DISTANCE_BEFORE_ORBIT
                    ):
                        set_wheel_angle(STRAIGHT_WHEEL_ANGLE_DEGREES)
                        carControls.setRot(current_wheel_angle)
                        return
                    logger.warning(
                        f"Entering roam with closest previous distance {closest_previous_distance}"
                    )
                    roam(
                        TargetObstacle(OBSTAClE_TYPES.RAMP, MAX_TARGET_DISTANCE),
                        spin_left=False,
                    )
                    consecutive_failed_detections = 0


while len(carControls.serial_port.read()) == 0:
    continue
print("Data from serial received")

try:
    if DEBUG:
        while True:
            # set_wheel_angle(45)
            # carControls.setRot(current_wheel_angle)
            # set_car_speed(40)
            # carControls.setSpeed(current_speed)
            frame = camera.get_frame()
            if frame:
                # objects = detector.get_detected_objects(frame)
                debug_video.add_frame(frame, [])

    # try to ramp up speed slowly to reduce chances of lidar instability
    while current_speed != MAX_CAR_SPEED:
        set_car_speed(current_speed + 10)
        carControls.setSpeed(current_speed)
        time.sleep(0.25)

    for index, obstacle in enumerate(TARGET_OBSTACLES):
        if obstacle.type == OBSTAClE_TYPES.BLUE:
            last_function = f"navigate_to_yellow_or_blue_bucket({obstacle.type.name})"
            logger.debug(f"Entering {last_function}")
            navigate_to_yellow_or_blue_bucket(obstacle)
            last_function = f"orbit({TARGET_OBSTACLES[index + 1].type.name}, {True})"
            logger.debug(f"Entering {last_function}")
            orbit(TARGET_OBSTACLES[index + 1], clockwise=True)
            # attempt to cancel out angular momentum
            set_wheel_angle(60)
            carControls.setRot(current_wheel_angle)
            time.sleep(0.15)
            set_wheel_angle(90)
            carControls.setRot(current_wheel_angle)
        if obstacle.type == OBSTAClE_TYPES.YELLOW:
            last_function = f"navigate_to_yellow_or_blue_bucket({obstacle.type.name})"
            logger.debug(f"Entering {last_function}")
            navigate_to_yellow_or_blue_bucket(obstacle)
            last_function = f"orbit({TARGET_OBSTACLES[index + 1].type.name}, {False})"
            logger.debug(f"Entering {last_function}")
            orbit(TARGET_OBSTACLES[index + 1], clockwise=False)
            # attempt to cancel out angular momentum
            set_wheel_angle(120)
            carControls.setRot(current_wheel_angle)
            time.sleep(0.15)
            set_wheel_angle(90)
            carControls.setRot(current_wheel_angle)
            # ------------------------- DEBUG  ONLY -------------------------
            # carControls.setSpeed(0)
            # exit()
        if obstacle.type == OBSTAClE_TYPES.RED:
            last_function = f"navigate_to_red_bucket()"
            logger.debug(f"Entering {last_function}")
            navigate_to_red_bucket()
            last_function = f"drive_between()"
            logger.debug(f"Entering {last_function}")
            drive_between()
            # ------------------------- DEBUG  ONLY -------------------------
            carControls.setSpeed(0)
            exit()
        if HIT_RAMP and obstacle.type == OBSTAClE_TYPES.RAMP:
            last_function = f"navigate_to_ramp()"
            logger.debug(f"Entering {last_function}")
            navigate_to_ramp()
            last_function = f"run_ramp()"
            logger.debug(f"Entering {last_function}")
            run_ramp()
        elif obstacle.type == OBSTAClE_TYPES.RAMP:
            circumvent_ramp()
            orbit(TARGET_OBSTACLES[index + 1], clockwise=False, orbiting_ramp=True)
            set_wheel_angle(80)
            carControls.setRot(current_wheel_angle)
            time.sleep(0.15)
            set_wheel_angle(90)
            carControls.setRot(current_wheel_angle)

        if obstacle.type == OBSTAClE_TYPES.DUMMY:
            raise Exception(
                f"Bucket Type {OBSTAClE_TYPES.DUMMY.name} should never be reached"
            )
except (
    Exception,
    KeyboardInterrupt,
):  # KeyboardInterrupt must be specified to hand ctrl+c
    logger.error("Error: ", exc_info=True)  # exc_info=True will log the stack trace
finally:
    carControls.setSpeed(0)
    carControls.setRot(STRAIGHT_WHEEL_ANGLE_DEGREES)
    debug_video.close()
