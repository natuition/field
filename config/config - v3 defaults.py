"""Configuration file."""


CONFIG_VERSION = "0.3.14"  # still have some obsolete keys


# ======================================================================================================================
# ROBOT HARDWARE PROPERTIES
# ======================================================================================================================
CORK_TO_CAMERA_DISTANCE_X = 0  # # distance between camera and cork on the robot, X axis, relative, mm
CORK_TO_CAMERA_DISTANCE_Y = 57  # distance between camera and cork on the robot, Y axis, relative, mm


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
FIELD_REDUCE_SIZE = 200  # cut field's each side for this value, mms
PREV_CUR_POINT_MIN_DIST = 100  # pass by cur points dist between them and prev point is lesser than this, mms


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
VESC_RPM_FAST = -5600
VESC_RPM_SLOW = -2800
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
SMOOTHIE_HOST = "169.254.232.224"  # smoothie's ip address for telnet or port for usb serial connector
SMOOTHIE_BACKEND = 2  # 1 = telnet, 2 = serial

# EXTRACTION
X_MIN = 0
X_MAX = 186.34951456310679611650485436893
Y_MIN = 0
Y_MAX = 91.747572815533980582524271844661
Z_MIN = 0
Z_MAX = 100

XY_F_MIN = 1
XY_F_MAX = 1000  # TODO: obsolete, need to update
Z_F_MIN = 1
Z_F_MAX = 1000  # TODO: obsolete, need to update

XY_COEFFICIENT_TO_MM = 0.40776699029126213592233009708738
Z_COEFFICIENT_TO_MM = 1  # not used yet

EXTRACTION_Z = 100  # TODO: obsolete and not used, need to update

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

CALIBRATION_DISTANCE = 1000  # should be always positive, sign will be auto-defined using *_AXIS_CALIBRATION_TO_MAX flag key
AFTER_CALIBRATION_AXIS_OFFSET = 0

# NAVIGATION
A_MIN = -38
A_MAX = 38
B_MIN = -float("inf")
B_MAX = float("inf")
C_MIN = -float("inf")
C_MAX = float("inf")

A_F_MIN = 1
A_F_MAX = 1000
B_F_MIN = 1
B_F_MAX = 1000
C_F_MIN = 1
C_F_MAX = 1000

A_ONE_DEGREE_IN_SMOOTHIE = 2  # A axis
A_DEGREES_PER_SECOND = 1  # A axis
NAV_TURN_WHEELS_CENTER = 0


# ======================================================================================================================
# YOLO PERIPHERY NETWORK SETTINGS
# ======================================================================================================================
PERIPHERY_CONFIDENCE_THRESHOLD = 0.25    # Confidence threshold
PERIPHERY_NMS_THRESHOLD = 0.4      # Non-maximum suppression threshold
PERIPHERY_INPUT_SIZE = (416, 416)
PERIPHERY_CONFIG_FILE = "yolo/periphery_config.cfg"
PERIPHERY_WEIGHTS_FILE = "yolo/periphery_weights.weights"
PERIPHERY_CLASSES_FILE = "yolo/periphery_classes.names"
PERIPHERY_DNN_BACKEND = 5  # cv.dnn: DNN_BACKEND_CUDA = 5; DNN_BACKEND_OPENCV = 3
PERIPHERY_DNN_TARGET = 7  # cv.dnn: DNN_TARGET_CUDA = 6; DNN_TARGET_CUDA_FP16 = 7; DNN_TARGET_CPU = 0


# ======================================================================================================================
# YOLO PRECISE NETWORK SETTINGS
# ======================================================================================================================
PRECISE_CONFIDENCE_THRESHOLD = 0.25    # Confidence threshold
PRECISE_NMS_THRESHOLD = 0.4      # Non-maximum suppression threshold
PRECISE_INPUT_SIZE = (608, 608)
PRECISE_CONFIG_FILE = "yolo/precise_config.cfg"
PRECISE_WEIGHTS_FILE = "yolo/precise_weights.weights"
PRECISE_CLASSES_FILE = "yolo/precise_classes.names"
PRECISE_DNN_BACKEND = 5  # cv.dnn: DNN_BACKEND_CUDA = 5; DNN_BACKEND_OPENCV = 3
PRECISE_DNN_TARGET = 6  # cv.dnn: DNN_TARGET_CUDA = 6; DNN_TARGET_CUDA_FP16 = 7; DNN_TARGET_CPU = 0


# ======================================================================================================================
# CAMERA SETTINGS
# ======================================================================================================================
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
ISP_DIGITAL_GAIN_RANGE_FROM = 8
ISP_DIGITAL_GAIN_RANGE_TO = 8
GAIN_RANGE_FROM = 4
GAIN_RANGE_TO = 4
EXPOSURE_TIME_RANGE_FROM = 55000
EXPOSURE_TIME_RANGE_TO = 55000
AE_LOCK = True
CV_ROTATE_CODE = 2
VIEW_ZONE_POLY_POINTS = [[387, 618], [439, 510], [556, 433], [670, 375], [808, 319], [982, 285], [1143, 279], [1293, 294], [1501, 339], [1635, 395], [1766, 473], [1816, 550], [1867, 637], [1881, 675], [1919, 795], [1942, 926], [1959, 1066], [1964, 1217], [1957, 1321], [1949, 1393], [1874, 1425], [1802, 1457], [1692, 1498], [1555, 1537], [1410, 1567], [1219, 1589], [1081, 1590], [944, 1590], [804, 1575], [679, 1552], [569, 1525], [423, 1475], [330, 1431], [277, 1399], [273, 1289], [279, 1131], [297, 976], [343, 780]]
WORKING_ZONE_POLY_POINTS = [[387, 618], [504, 553], [602, 506], [708, 469], [842, 434], [1021, 407], [1228, 410], [1435, 443], [1587, 492], [1726, 558], [1867, 637], [1881, 675], [1919, 795], [1942, 926], [1954, 1055], [1953, 1176], [1551, 1187], [1145, 1190], [724, 1190], [454, 1188], [286, 1188], [283, 1082], [296, 979], [318, 874], [351, 753]]
IMAGE_CONTROL_POINTS_MAP = [[1148, 1042, 0, 20, 0], [996, 1042, -20, 20, 1], [851, 1046, -40, 20, 2], [725, 1054, -60, 20, 3], [617, 1061, -80, 20, 4], [529, 1069, -100, 20, 5], [459, 1077, -120, 20, 6], [400, 1085, -140, 20, 7], [357, 1091, -160, 20, 8], [321, 1097, -180, 20, 9], [1146, 897, 0, 40, 10], [999, 899, -20, 40, 11], [859, 905, -40, 40, 12], [734, 919, -60, 40, 13], [629, 934, -80, 40, 14], [540, 950, -100, 40, 15], [470, 964, -120, 40, 16], [409, 980, -140, 40, 17], [363, 994, -160, 40, 18], [329, 1005, -180, 40, 19], [1146, 767, 0, 60, 20], [1006, 770, -20, 60, 21], [872, 778, -40, 60, 22], [750, 796, -60, 60, 23], [647, 817, -80, 60, 24], [558, 839, -100, 60, 25], [487, 861, -120, 60, 26], [426, 880, -140, 60, 27], [379, 901, -160, 60, 28], [343, 919, -180, 60, 29], [1146, 658, 0, 80, 30], [1014, 661, -20, 80, 31], [887, 671, -40, 80, 32], [770, 691, -60, 80, 33], [669, 714, -80, 80, 34], [583, 739, -100, 80, 35], [511, 765, -120, 80, 36], [448, 788, -140, 80, 37], [397, 815, -160, 80, 38], [361, 837, -180, 80, 39], [1145, 567, 0, 100, 40], [1022, 571, -20, 100, 41], [904, 581, -40, 100, 42], [791, 601, -60, 100, 43], [694, 626, -80, 100, 44], [608, 653, -100, 100, 45], [538, 681, -120, 100, 46], [473, 707, -140, 100, 47], [421, 734, -160, 100, 48], [381, 760, -180, 100, 49], [1144, 496, 0, 120, 50], [1030, 499, -20, 120, 51], [919, 509, -40, 120, 52], [814, 528, -60, 120, 53], [720, 551, -80, 120, 54], [637, 577, -100, 120, 55], [566, 607, -120, 120, 56], [500, 634, -140, 120, 57], [441, 667, -160, 120, 58], [403, 693, -180, 120, 59], [1144, 439, 0, 140, 60], [1038, 441, -20, 140, 61], [935, 449, -40, 140, 62], [835, 468, -60, 140, 63], [746, 489, -80, 140, 64], [664, 517, -100, 140, 65], [593, 546, -120, 140, 66], [530, 572, -140, 140, 67], [472, 604, -160, 140, 68], [421, 632, -180, 140, 69], [1298, 1044, 20, 20, 70], [1432, 1048, 40, 20, 71], [1550, 1054, 60, 20, 72], [1647, 1061, 80, 20, 73], [1726, 1078, 100, 20, 74], [1793, 1075, 120, 20, 75], [1847, 1082, 140, 20, 76], [1886, 1087, 160, 20, 77], [1921, 1091, 180, 20, 78], [1292, 901, 20, 40, 79], [1423, 909, 40, 40, 80], [1540, 925, 60, 40, 81], [1636, 929, 80, 40, 82], [1717, 955, 100, 40, 83], [1786, 969, 120, 40, 84], [1839, 831, 140, 40, 85], [1879, 955, 160, 40, 86], [1914, 1005, 180, 40, 87], [1284, 775, 20, 60, 88], [1411, 788, 40, 60, 89], [1525, 806, 60, 60, 90], [1620, 828, 80, 60, 91], [1700, 849, 100, 60, 92], [1770, 868, 120, 60, 93], [1826, 886, 140, 60, 94], [1868, 907, 160, 60, 95], [1903, 924, 180, 60, 96], [1275, 677, 20, 80, 97], [1397, 682, 40, 80, 98], [1506, 703, 60, 80, 99], [1600, 727, 80, 80, 100], [1681, 752, 100, 80, 101], [1750, 776, 120, 80, 102], [1808, 799, 140, 80, 103], [1851, 824, 160, 80, 104], [1887, 845, 180, 80, 105], [1265, 788, 20, 100, 106], [1380, 593, 40, 100, 107], [1486, 615, 60, 100, 108], [1577, 640, 80, 100, 109], [1655, 668, 100, 100, 110], [1727, 694, 120, 100, 111], [1788, 719, 140, 100, 112], [1830, 747, 160, 100, 113], [1868, 773, 180, 100, 114], [1287, 505, 20, 120, 115], [1365, 520, 40, 120, 116], [1453, 541, 60, 120, 117], [1552, 567, 80, 120, 118], [1630, 595, 100, 120, 119], [1700, 621, 120, 120, 120], [1761, 647, 140, 120, 121], [1809, 679, 160, 120, 122], [1843, 705, 180, 120, 123], [1249, 445, 20, 140, 124], [1350, 460, 40, 140, 125], [1444, 480, 60, 140, 126], [1528, 506, 80, 140, 127], [1605, 532, 100, 140, 128], [1674, 557, 120, 140, 129], [1735, 584, 140, 140, 130], [1784, 713, 160, 140, 131], [1821, 635, 180, 140, 132]]
UNDISTORTED_ZONE_RADIUS = 240


# ======================================================================================================================
# APP SETTINGS
# ======================================================================================================================
SAVE_DEBUG_IMAGES = True
DEBUG_IMAGES_PATH = "debug_images/"
RECEIVE_FIELD_FROM_RTK = True
SLOW_MODE_MIN_TIME = 3  # seconds


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
