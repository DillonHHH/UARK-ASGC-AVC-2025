LIDAR_SCAN_FOV_DEGREES = 120
NUMBER_OF_LANDMARKS = 40


# ALL MEASUREMENTS ARE FROM THE CENTER OF BUCKETS, NOT THE EDGES
# TODO scale this at longer distances to allow for greater difference between lidar and camera distances
MAX_LIDAR_CAMERA_DISTANCE_THRESHOLD_METERS = 0.75  # max difference between a lidar and camera scan to be considered a good measurment

MIN_DISTANCE_BETWEEN_BUCKETS_METERS = 1

MAX_BUCKET_MATCHING_DISTANCE_METERS = (
    0.5  # this needs to always be smaller than MIN_DISTANCE_BETWEEN_BUCKETS_METERS
)

MAX_LIDAR_SCAN_BUCKET_MATCHING_DISTANCE_METERS = 0.5

MIN_PIXEL_MATCH_DISTANCE_FROM_EDGES = 10  # minimum number of pixels a lidar scan must be from the box edges of a detected object to match with it

MIN_SCAN_BUFFER_SIZE = 200  # minimum number of scans before doing any processing, avoids skewing the EKF with too few data points

BUCKET_DIAMETER_METERS = 0.3048  # about 12 inches

# MIN_CONSECUTIVE_LANDMARK_RECOGNITIONS = 3  # number of times a landmark must be seen consecutively before getting add to the pointcloud
MIN_OBSERVATION_VALUE_TO_ADD_LANDMARK = 5
LANDMARK_OBSERVATION_RATIO = 1.5  # a landmark's observation value will be incremented by this much every frame it is seen and decremented by this value's inverse if it is not seen


MIN_POINTS_FOR_ARC = 5  # Require at least 5 points to attempt circle fitting
ARC_HYSTERESIS = 1.2  # Increase grouping threshold slightly to avoid breaks


# main
SERVER_PORT = 7000
MAP_SIZE = 10
CELL_SIZE = 1
# ORIGIN = Vector(MAP_SIZE / 2, MAP_SIZE / 2)  # centered by default
ACCELEROMETER_TRUST_FACTOR = 0.70
UPDATES_PER_SECOND = 120
NEW_SCAN_TRUST_FACTOR = 0.8


# Arc Recognition
MAX_DISTANCE_BETWEEN_POINTS_IN_ARC = 0.5
MAX_CIRCLE_RADIUS = (
    1.4  # buckets have a diameter of 12 inches at the top and 10.5 at the bottom
)
MIN_DISTANCE_BETWEEN_CIRCLES = 1
MIN_CIRCLE_RADIUS = 0.01

MAX_POINT_MATCH_TO_BUCKET_PERIMETER = 0.2  # A point which is less than this distance to the nearest circle's perimeter will be matched to that circle

NEW_BUCKET_SCAN_WEIGHT = (
    0.1  # How much to trust a new scan for a bucket vs the old scans
)

CAMERA_FOV_DEGREES: int = 69
MIN_PIXEL_DISTANCE_FROM_EDGE_TO_GET_DEPTH: int = (
    32  # Minimum number of pixels a pixel must be from the edge of the camera to get the depth. Useful since edges of camera are likely to return innacurate depths
)
