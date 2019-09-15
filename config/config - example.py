import cv2 as cv

CONFIG_VERSION = "0.1"

# ======================================================================================================================
# SMOOTHIE SETTINGS
# ======================================================================================================================
SMOOTHIE_HOST = "169.254.232.224"

# EXTRACTION
X_MIN = 0
X_MAX = 198
Y_MIN = 0
Y_MAX = 79
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
Y_AXIS_CALIBRATION_TO_MAX = True
Z_AXIS_CALIBRATION_TO_MAX = True
A_AXIS_CALIBRATION_TO_MAX = None
B_AXIS_CALIBRATION_TO_MAX = None
C_AXIS_CALIBRATION_TO_MAX = None

CALIBRATION_DISTANCE = 1000
AFTER_CALIBRATION_AXIS_OFFSET = 15

# NAVIGATION
A_MIN = -20
A_MAX = 20
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

ONE_MM_IN_SMOOTHIE = 0.338  # for B axis! # 0.404 for new wheels
NAV_TURN_WHEELS_CENTER = 0


# ======================================================================================================================
# COMPASS
# ======================================================================================================================
COMPASS_DEVICE_ADDRESS = 0x1e   # HMC5883L magnetometer device address


# ======================================================================================================================
# PATHS
# ======================================================================================================================
HIST_DATABASE_PATH = "database\\database.npy"
PATTERNS_DATASET_DIR = "database\\images"
QUERY_IMAGE_PATH = "input\\prise8.jpg"
OUTPUT_IMAGE_DIR = "output\\"
OUTPUT_IMAGE_NAME = "prise2 - AOI"
OUTPUT_IMAGE_EXTENSION = ".jpg"


# ======================================================================================================================
# IMAGE PROCESSING
# ======================================================================================================================
# AOI - area of interest
AOI_TOP_BORDER = 595
AOI_BOTTOM_BORDER = 1200
AOI_LEFT_BORDER = 854
AOI_RIGHT_BORDER = 1734

FRAGMENT_W = 120
FRAGMENT_H = 120
FRAGMENT_X_OFFSET = 60
FRAGMENT_Y_OFFSET = 60

HIST_CHANNELS = [0, 1, 2]
HIST_SIZE = (8, 12, 3)
HIST_RANGE = [0, 180, 0, 256, 0, 256]
HIST_COMP_METHOD = cv.HISTCMP_CHISQR

# for some hist comp algs lesser distance means more similar images, and vice versa
LESSER_DIST_MORE_SIMILAR = True

# ======================================================================================================================
# APP CONFIG
# ======================================================================================================================
# Modes:
# "database" - loads image and adds fragments from AOI to patterns and hist DB. Image sourse depends on use_camera
# setting
# "searching" - loads image and searches most unlike grass fragment and tries to extract this plant. Image source
# depends on use_camera setting
APP_MODE = "searching"
USE_PI_CAMERA = True
CAMERA_W = 2592
CAMERA_H = 1944
CAMERA_FRAMERATE = 32
CORK_CENTER_X = 1292
CORK_CENTER_Y = 1172
ONE_MM_IN_PX = 4

WEB_SERVER_HOST = "127.0.0.1"
WEB_SERVER_PORT = 8080

STREAM_SERVER_HOST = "192.168.8.100"
STREAM_SERVER_PORT_HTTP = 8082
STREAM_SERVER_PORT_WS = 8084
