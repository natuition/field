"""Configuration file."""


CONFIG_VERSION = "0.3.6"  # still have some obsolete keys


# ======================================================================================================================
# NAVIGATION ROUTING SETTINGS
# ======================================================================================================================
KP = 0.2  # wheels turning degrees multiplier
KI = 0.02
MANEUVERS_FREQUENCY = 2  # seconds
# max distance in mm-s of robot's deviation from planned moving vector
# if dev. is bigger than this - robot will turn to it's planned moving vector
COURSE_SIDE_DEVIATION_MAX = 50  # max allowed robot's deviation from course im mm-s (threshold)
# distance to stop in mm-s between robot and path ending point
# (its a good idea to keep this value greater than allowed course deviation)
COURSE_DESTINATION_DIFF = 70
COURSE_ADJ_SMC_VAL = 7  # nav wheels turn value when trying to get back to the course (SHOULD BE POSITIVE VALUE!)
WINDOW = 10
# AB moving vector used as A----A1--B, where A1 is point when robot starts moving to next point.
# this determines A1-B distance
MANEUVER_START_DISTANCE = 3000
USE_SPEED_LIMIT = True  # when distance to target point is less than specified in the config
DECREASE_SPEED_TRESHOLD = 19000  # millimeters
SUM_ANGLES_HISTORY_MAX = 1000  # max value and min -value of sum(angles_history), should be positive here, in config
# distance between sides of spiral robot movements, expected to be equal to working area width, may be any positive val
SPIRAL_SIDES_INTERVAL = 457


# ======================================================================================================================
# PATHS SETTINGS
# ======================================================================================================================
INPUT_GPS_FIELD_FILE = "field.txt"
OUTPUT_GPS_HISTORY_FILE = "gps_history.txt"


# ======================================================================================================================
# VESC SETTINGS
# ======================================================================================================================
VESC_PORT = "/dev/ttyACM0"
VESC_BAUDRATE = 115200
VESC_RPM = -2500
VESC_MOVING_TIME = float("inf")
VESC_ALIVE_FREQ = 0.5  # freq of sending "keep working" signal to engines when moving
VESC_CHECK_FREQ = 0.01  # freq of checking need to stop


# ======================================================================================================================
# GPS SETTINGS
# ======================================================================================================================
GPS_PORT = "/dev/ttyTHS1"
GPS_BAUDRATE = 19200
GPS_POSITIONS_TO_KEEP = 1000


# ======================================================================================================================
# SMOOTHIE SETTINGS
# ======================================================================================================================
SMOOTHIE_HOST = "169.254.232.224"

# EXTRACTION
X_MIN = 0
X_MAX = 457
Y_MIN = 0
Y_MAX = 225
Z_MIN = 0
Z_MAX = 100

XY_F_MIN = 1
XY_F_MAX = 1000
Z_F_MIN = 1
Z_F_MAX = 1000

EXTRACTION_Z = 100

# CALIBRATION
USE_X_AXIS_CALIBRATION = False
USE_Y_AXIS_CALIBRATION = False
USE_Z_AXIS_CALIBRATION = False
USE_A_AXIS_CALIBRATION = False
USE_B_AXIS_CALIBRATION = False
USE_C_AXIS_CALIBRATION = False

X_AXIS_CALIBRATION_TO_MAX = False
Y_AXIS_CALIBRATION_TO_MAX = False
Z_AXIS_CALIBRATION_TO_MAX = True
A_AXIS_CALIBRATION_TO_MAX = None
B_AXIS_CALIBRATION_TO_MAX = None
C_AXIS_CALIBRATION_TO_MAX = None

CALIBRATION_DISTANCE = 1000
AFTER_CALIBRATION_AXIS_OFFSET = 0

# NAVIGATION
A_MIN = -38
A_MAX = 38
B_MIN = float("inf")
B_MAX = float("inf")
C_MIN = float("inf")
C_MAX = float("inf")

A_F_MIN = 1
A_F_MAX = 1000
B_F_MIN = 1
B_F_MAX = 1000
C_F_MIN = 1
C_F_MAX = 1000

A_ONE_DEGREE_IN_SMOOTHIE = 2  # A axis
A_DEGREES_PER_SECOND = 1  # A axis
B_ONE_MM_IN_SMOOTHIE = 0.338  # B axis # 0.404 for new wheels # obsolete
NAV_TURN_WHEELS_CENTER = 0


# ======================================================================================================================
# YOLO DETECTION SETTINGS
# ======================================================================================================================
CONFIDENCE_THRESHOLD = 0.25    # Confidence threshold
NMS_THRESHOLD = 0.4      # Non-maximum suppression threshold
INPUT_SIZE = (416, 416)
OUTPUT_IMG_DIR = "output/"
INPUT_IMG_DIR = "input/"
INPUT_IMG_FILE = "10.jpg"
YOLO_CONFIG_FILE = "yolo/yolov3_plantain_inference.cfg"
YOLO_WEIGHTS_FILE = "yolo/yolov3_plantain_final.weights"
YOLO_CLASSES_FILE = "yolo/classes.names"


# ======================================================================================================================
# CAMERA SETTINGS
# ======================================================================================================================
USE_PI_CAMERA = True
CAMERA_W = 3280
CAMERA_H = 2464
CROP_W_FROM = 0
CROP_W_TO = 2288
CROP_H_FROM = 498
CROP_H_TO = 2880
CAMERA_FRAMERATE = 5
CAMERA_FLIP_METHOD = 0
CORK_CENTER_X = 1144
CORK_CENTER_Y = 1590
ONE_MM_IN_PX = 6


# ======================================================================================================================
# APP SETTINGS
# ======================================================================================================================
SAVE_DEBUG_IMAGES = True


# ======================================================================================================================
# OBSOLETTE SETTINGS
# ======================================================================================================================
WEB_SERVER_HOST = "127.0.0.1"
WEB_SERVER_PORT = 8080

STREAM_SERVER_HOST = "192.168.8.100"
STREAM_SERVER_PORT_HTTP = 8082
STREAM_SERVER_PORT_WS = 8084


# ======================================================================================================================
# OLD PATHS
# ======================================================================================================================
QUERY_IMAGE_PATH = "input\\prise8.jpg"
OUTPUT_IMAGE_DIR = "output\\"
OUTPUT_IMAGE_NAME = "prise2 - AOI"
OUTPUT_IMAGE_EXTENSION = ".jpg"
