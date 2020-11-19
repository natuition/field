

import cv2 as cv

CONFIG_VERSION = "0.2.6"

# ======================================================================================================================
# SMOOTHIE SETTINGS
# ======================================================================================================================
SMOOTHIE_BACKEND = 2  # 1 = telnet, 2 = serial
SMOOTHIE_HOST = "/dev/ttyACM0" # "" 192.168.9.101"
SMOOTHIE_BAUDRATE = 115200

# EXTRACTION
X_MIN = 0
X_MAX = 440
Y_MIN = 0
Y_MAX = 225
Z_MAX = 100
Z_MIN = -float("inf")

XY_F_MIN = 1
XY_F_MAX = 4000
Z_F_MIN = 1
Z_F_MAX = 2000

EXTRACTION_Z = 85 #75
Z_F_EXTRACTION_UP = 1350 #1300
Z_F_EXTRACTION_DOWN = 1900 #2000

# CALIBRATION
USE_X_AXIS_CALIBRATION = True
USE_Y_AXIS_CALIBRATION = True
USE_Z_AXIS_CALIBRATION = True
USE_A_AXIS_CALIBRATION = False
USE_B_AXIS_CALIBRATION = False
USE_C_AXIS_CALIBRATION = False

X_AXIS_CALIBRATION_TO_MAX = False
Y_AXIS_CALIBRATION_TO_MAX = False
Z_AXIS_CALIBRATION_TO_MAX = False
A_AXIS_CALIBRATION_TO_MAX = None
B_AXIS_CALIBRATION_TO_MAX = None
C_AXIS_CALIBRATION_TO_MAX = None

CALIBRATION_DISTANCE = 1000
AFTER_CALIBRATION_AXIS_OFFSET = 0

# NAVIGATION
A_MIN = -38
A_MAX = 38
B_MIN = -float("inf")
B_MAX = float("inf")
C_MIN = -float("inf")
C_MAX = float("inf")

A_F_MIN = 1
A_F_MAX = 4000
B_F_MIN = 1
B_F_MAX = 4000
C_F_MIN = 1
C_F_MAX = 1000

ONE_MM_IN_SMOOTHIE = 0.338  # for B axis! # 0.404 for new wheels
NAV_TURN_WHEELS_CENTER = 0

# ======================================================================================================================
# COMPASS
# ======================================================================================================================
COMPASS_DEVICE_ADDRESS = 0x1e   # HMC5883L magnetometer device address
COMPASS_DECLINATION = -0.00669  # define declination angle of location where measurement going to be done


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
# YOLO DETECTION CONFIG
# ======================================================================================================================
CONFIDENCE_THRESHOLD = 0.25    # Confidence threshold
NMS_THRESHOLD = 0.4      # Non-maximum suppression threshold
INPUT_SIZE = (608, 608)
#INPUT_SIZE = (416, 416)
OUTPUT_IMG_DIR = "output/"
INPUT_IMG_DIR = "input/"
INPUT_IMG_FILE = "10.jpg"
#YOLO_CONFIG_FILE = "yolo/yolov3_natuition_inference.cfg"
#YOLO_WEIGHTS_FILE = "yolo/yolov3_natuition_best.weights"
#YOLO_CLASSES_FILE = "yolo/classes.names"

#YOLO_CONFIG_FILE = "yolo/yolov3_plantain_inference.cfg"
#YOLO_WEIGHTS_FILE = "yolo/yolov3_plantain_final.weights"
#YOLO_CLASSES_FILE = "yolo/classes.names"

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
CAMERA_W = 3280
CAMERA_H = 2464
APPLY_IMAGE_CROPPING = True
CROP_W_FROM = 530
CROP_W_TO = 2730
CROP_H_FROM = 0
CROP_H_TO = 2000
CAMERA_FRAMERATE = 5
CV_APPLY_ROTATION = False
CAMERA_FLIP_METHOD = 0
SCENE_CENTER_X = 1117  # 1647 for uncropped
SCENE_CENTER_Y = 1167  # 1167 for uncropped
ONE_MM_IN_PX = 6
WEB_SERVER_HOST = "127.0.0.1"
WEB_SERVER_PORT = 8080

STREAM_SERVER_HOST = "192.168.8.100"
STREAM_SERVER_PORT_HTTP = 8082
STREAM_SERVER_PORT_WS = 8084
# ======================================================================================================================
# NAVIGATION ROUTING SETTINGS
# ======================================================================================================================
# max distance in mm-s of robot's deviation from planned moving vector
# if dev. is bigger than this - robot will turn to it's planned moving vector
COURSE_SIDE_DEVIATION_MAX = 50  # max allowed robot's deviation from course im mm-s (threshold)
# distance to stop in mm-s between robot and path ending point
# (its a good idea to keep this value greater than allowed course deviation)
COURSE_DESTINATION_DIFF = 3000
COURSE_ADJ_SMC_VAL = 10  # nav wheels turn value when trying to get back to the course (SHOULD BE POSITIVE VALUE!)

# ======================================================================================================================
# EXTRACTION SETTINGS
# ======================================================================================================================
EXTRACTION_DEFAULT_METHOD = "single_center_drop" #"five_drops_near_center"   or "single_center_drop"
ADDITIONAL_EXTRACTIONS_DISTANCE_X = 20  # mm
ADDITIONAL_EXTRACTIONS_DISTANCE_Y = 20  # mm

# ======================================================================================================================
# PATHS SETTINGS
# ======================================================================================================================
INPUT_GPS_FIELD_FILE = "field.txt"
OUTPUT_GPS_HISTORY_FILE = "gps_history.txt"


# ======================================================================================================================
# VESC SETTINGS
# ======================================================================================================================
VESC_PORT = "/dev/ttyACM1"
VESC_BAUDRATE = 115200
VESC_RPM = -8000
VESC_MOVING_TIME = float("inf")  # value in seconds or float("inf")
VESC_ALIVE_FREQ = 0.05  # freq of sending "keep working" signal to engines when moving
VESC_CHECK_FREQ = 0.001  # freq of checking need to stop


# ======================================================================================================================
# GPS SETTINGS
# ======================================================================================================================
GPS_PORT = "/dev/ttyTHS1"
GPS_BAUDRATE = 19200
GPS_POSITIONS_TO_KEEP = 1000

A_ONE_DEGREE_IN_SMOOTHIE = 2

KP= 0.15  # wheels turning degrees multiplier
MANEUVERS_FREQUENCY = 1  # seconds
A_DEGREES_PER_SECOND = 5  # A axis
KI = 0.0075
WINDOW = float("inf")
# AB moving vector used as A----A1--B, where A1 is point when robot starts moving to next point.
# this determines A1-B distance
MANEUVER_START_DISTANCE = 4000
USE_SPEED_LIMIT = True  # when distance to target point is less than specified in the config
DECREASE_SPEED_TRESHOLD = 5000  # millimeters
SUM_ANGLES_HISTORY_MAX = 1000  # max value and min -value of sum(angles_history), should be positive here, in config
SPIRAL_SIDES_INTERVAL = 333

#=========================================================================================
# POLYGON WORKSONZ SETTINGS
#=========================================================================================
CORK_TO_CAMERA_DISTANCE_X = 0
CORK_TO_CAMERA_DISTANCE_Y = 30

WORKING_ZONE_POLY_POINTS = [[1963, 1180], [1345, 1168], [1024, 1165], [697, 1164], [239, 1166], [238, 1079], [244, 1008], [255, 925], [273, 835], [288, 783], [311, 712], [332, 653], [359, 597], [389, 540], [471, 489], [538, 451], [643, 407], [694, 386], [772, 363], [853, 344], [945, 329], [1042, 321], [1131, 323], [1204, 325], [1287, 337], [1376, 352], [1456, 374], [1530, 401], [1595, 428], [1658, 459], [1709, 489], [1758, 519], [1805, 551], [1836, 575], [1860, 621], [1877, 652], [1900, 711], [1922, 789], [1938, 857], [1951, 921], [1961, 995], [1965, 1077], [1965, 1120]]
IMAGE_CONTROL_POINTS_MAP = [[1117, 1094, 0, 10, 1], [989, 1094, -17, 10, 2], [851, 1097, -37, 10, 3], [734, 1097, -57, 10, 4], [617, 1101, -77, 10, 5], [527, 1104, -97, 10, 6], [453, 1110, -117, 10, 7], [389, 1114, -137, 10, 8], [340, 1118, -157, 10, 9], [298, 1122, -177, 10, 10], [272, 1123, -197, 10, 11], [250, 1127, -217, 10, 12], [1118, 950, 0, 30, 13], [991, 952, -17, 30, 14], [860, 953, -37, 30, 15], [735, 962, -57, 30, 16], [621, 975, -77, 30, 17], [530, 987, -97, 30, 18], [456, 998, -117, 30, 19], [395, 1010, -137, 30, 20], [344, 1022, -157, 30, 21], [304, 1031, -177, 30, 22], [273, 1040, -197, 30, 23], [250, 1048, -217, 30, 24], [1121, 816, 0, 50, 25], [996, 817, -17, 50, 26], [867, 822, -37, 50, 27], [738, 836, -57, 50, 28], [630, 854, -77, 50, 29], [544, 871, -97, 50, 30], [467, 892, -117, 50, 31], [406, 910, -137, 50, 32], [355, 929, -157, 50, 33], [311, 945, -177, 50, 34], [279, 957, -197, 50, 35], [255, 968, -217, 50, 36], [1119, 700, 0, 70, 37], [1001, 701, -17, 70, 38], [873, 708, -37, 70, 39], [752, 724, -57, 70, 40], [649, 745, -77, 70, 41], [559, 768, -97, 70, 42], [485, 793, -117, 70, 43], [424, 815, -137, 70, 44], [367, 841, -157, 70, 45], [321, 864, -177, 70, 46], [291, 878, -197, 70, 47], [267, 894, -217, 70, 48], [1121, 600, 0, 90, 49], [1011, 602, -17, 90, 50], [887, 611, -37, 90, 51], [774, 626, -57, 90, 52], [672, 649, -77, 90, 53], [583, 676, -97, 90, 54], [507, 705, -117, 90, 55], [445, 730, -137, 90, 56], [382, 761, -157, 90, 57], [331, 785, -177, 90, 58], [308, 794, -197, 90, 59], [288, 806, -217, 90, 60], [1121, 519, 0, 110, 61], [1016, 520, -17, 110, 62], [900, 528, -37, 110, 63], [791, 544, -57, 110, 64], [695, 568, -77, 110, 65], [609, 596, -97, 110, 66], [534, 625, -117, 110, 67], [468, 657, -137, 110, 68], [408, 684, -157, 110, 69], [355, 711, -177, 110, 70], [326, 729, -197, 110, 71], [308, 738, -217, 110, 72], [1121, 451, 0, 130, 73], [1024, 451, -17, 130, 74], [917, 459, -37, 130, 75], [811, 477, -57, 130, 76], [719, 498, -77, 130, 77], [632, 524, -97, 130, 78], [556, 556, -117, 130, 79], [490, 586, -137, 130, 80], [431, 615, -157, 130, 81], [385, 647, -177, 130, 82], [347, 670, -197, 130, 83], [326, 685, -217, 130, 84], [1121, 399, 0, 150, 85], [1029, 399, -17, 150, 86], [927, 411, -37, 150, 87], [829, 425, -57, 150, 88], [738, 446, -77, 150, 89], [657, 476, -97, 150, 90], [575, 507, -117, 150, 91], [512, 537, -137, 150, 92], [451, 568, -157, 150, 93], [402, 600, -177, 150, 94], [368, 623, -197, 150, 95], [346, 639, -217, 150, 96], [1120, 359, 0, 170, 97], [1035, 360, -17, 170, 98], [939, 370, -37, 170, 99], [847, 384, -57, 170, 100], [758, 406, -77, 170, 101], [674, 435, -97, 170, 102], [593, 468, -117, 170, 103], [528, 499, -137, 170, 104], [468, 533, -157, 170, 105], [421, 561, -177, 170, 106], [388, 584, -197, 170, 107], [364, 602, -217, 170, 108], [1120, 326, 0, 190, 109], [1042, 328, -17, 190, 110], [950, 335, -37, 190, 111], [859, 349, -57, 190, 112], [782, 366, -77, 190, 113], [693, 395, -97, 190, 114], [611, 429, -117, 190, 115], [546, 458, -137, 190, 116], [488, 491, -157, 190, 117], [439, 521, -177, 190, 118], [403, 545, -197, 190, 119], [382, 561, -217, 190, 120], [1243, 1096, 17, 10, 121], [1379, 1100, 37, 10, 122], [1499, 1104, 57, 10, 123], [1601, 1111, 77, 10, 124], [1688, 1115, 97, 10, 125], [1755, 1120, 117, 10, 126], [1808, 1125, 137, 10, 127], [1854, 1129, 157, 10, 128], [1895, 1136, 177, 10, 129], [1931, 1140, 197, 10, 130], [1955, 1142, 217, 10, 131], [1244, 955, 17, 30, 132], [1379, 962, 37, 30, 133], [1500, 975, 57, 30, 134], [1603, 987, 77, 30, 135], [1688, 1000, 97, 30, 136], [1757, 1014, 117, 30, 137], [1809, 1027, 137, 30, 138], [1853, 1036, 157, 30, 139], [1894, 1049, 177, 30, 140], [1933, 1058, 197, 30, 141], [1954, 1066, 217, 30, 142], [1242, 822, 17, 50, 143], [1374, 835, 37, 50, 144], [1490, 851, 57, 50, 145], [1592, 870, 77, 50, 146], [1678, 891, 97, 50, 147], [1746, 910, 117, 50, 148], [1803, 930, 137, 50, 149], [1840, 943, 157, 50, 150], [1886, 962, 177, 50, 151], [1930, 984, 197, 50, 152], [1950, 992, 217, 50, 153], [1236, 706, 17, 70, 154], [1364, 719, 37, 70, 155], [1478, 740, 57, 70, 156], [1577, 764, 77, 70, 157], [1662, 788, 97, 70, 158], [1730, 813, 117, 70, 159], [1787, 837, 137, 70, 160], [1824, 855, 157, 70, 161], [1879, 884, 177, 70, 162], [1921, 908, 197, 70, 163], [1942, 921, 217, 70, 164], [1230, 606, 17, 90, 165], [1352, 622, 37, 90, 166], [1459, 645, 57, 90, 167], [1558, 672, 77, 90, 168], [1642, 700, 97, 90, 169], [1711, 727, 117, 90, 170], [1767, 752, 137, 90, 171], [1807, 775, 157, 90, 172], [1862, 806, 177, 90, 173], [1909, 837, 197, 90, 174], [1929, 850, 217, 90, 175], [1224, 525, 17, 110, 176], [1339, 541, 37, 110, 177], [1442, 564, 57, 110, 178], [1536, 590, 77, 110, 179], [1618, 619, 97, 110, 180], [1687, 646, 117, 110, 181], [1748, 678, 137, 110, 182], [1787, 703, 157, 110, 183], [1843, 732, 177, 110, 184], [1891, 769, 197, 110, 185], [1913, 784, 217, 110, 186], [1219, 458, 17, 130, 187], [1323, 475, 37, 130, 188], [1423, 495, 57, 130, 189], [1514, 519, 77, 130, 190], [1593, 546, 97, 130, 191], [1662, 578, 117, 130, 192], [1723, 611, 137, 130, 193], [1765, 634, 157, 130, 194], [1816, 667, 177, 130, 195], [1865, 697, 197, 130, 196], [1891, 711, 217, 130, 197], [1211, 404, 17, 150, 198], [1311, 419, 37, 150, 199], [1404, 437, 57, 150, 200], [1492, 461, 77, 150, 201], [1570, 488, 97, 150, 202], [1635, 514, 117, 150, 203], [1699, 551, 137, 150, 204], [1745, 575, 157, 150, 205], [1792, 603, 177, 150, 206], [1839, 636, 197, 150, 207], [1869, 653, 217, 150, 208], [1206, 362, 17, 170, 209], [1299, 374, 37, 170, 210], [1387, 391, 57, 170, 211], [1472, 416, 77, 170, 212], [1544, 443, 97, 170, 213], [1613, 473, 117, 170, 214], [1674, 500, 137, 170, 215], [1723, 526, 157, 170, 216], [1773, 557, 177, 170, 217], [1817, 586, 197, 170, 218], [1845, 603, 217, 170, 219], [1202, 331, 17, 190, 220], [1288, 343, 37, 190, 221], [1377, 358, 57, 190, 222], [1455, 380, 77, 190, 223], [1530, 407, 97, 190, 224], [1596, 436, 117, 190, 225], [1658, 465, 137, 190, 226], [1707, 493, 157, 190, 227], [1757, 524, 177, 190, 228], [1803, 557, 197, 190, 229], [1833, 579, 217, 190, 230]]
UNDISTORTED_ZONE_RADIUS = 240
#=========================================================================================
# CAMERA SETTINGS
#=========================================================================================
ISP_DIGITAL_GAIN_RANGE_FROM = 4
ISP_DIGITAL_GAIN_RANGE_TO = 4
GAIN_RANGE_FROM = 4
GAIN_RANGE_TO = 4
EXPOSURE_TIME_RANGE_FROM = 660000 #660000
EXPOSURE_TIME_RANGE_TO = 660000 #660000
AE_LOCK = True
CV_ROTATE_CODE = 2

#=========================================================================================
# YOLO SETTINGS
#=========================================================================================
# YOLO PERIPHERY NETWORK SETTINGS
PERIPHERY_CONFIDENCE_THRESHOLD = 0.1    # Confidence threshold
PERIPHERY_NMS_THRESHOLD = 0.4      # Non-maximum suppression threshold
PERIPHERY_INPUT_SIZE = (416, 416)
PERIPHERY_CONFIG_FILE = "yolo/Y0012.cfg" #"yolo/tinyperif.cfg"
PERIPHERY_WEIGHTS_FILE = "yolo/Y0012.weights" # "yolo/tinyperif.weights"
PERIPHERY_CLASSES_FILE = "yolo/tinyperif.names"

# YOLO PRECISE NETWORK SETTINGS
PRECISE_CONFIDENCE_THRESHOLD = 0.1    # Confidence threshold
PRECISE_NMS_THRESHOLD = 0.4      # Non-maximum suppression threshold
PRECISE_INPUT_SIZE = (832, 832) # (608, 608)
PRECISE_CONFIG_FILE = "yolo/Y0012.cfg" # "yolo/precise_config.cfg"
PRECISE_WEIGHTS_FILE = "yolo/Y0012.weights" # "yolo/precise_weights.weights"
PRECISE_CLASSES_FILE = "yolo/tinyperif.names" # "yolo/precise_classes.names"

#photos for debug
SAVE_DEBUG_IMAGES = True
DEBUG_IMAGES_PATH = "debug_images/"

FIELD_REDUCE_SIZE = 200  # cut field's each side for this value, mms
RECEIVE_FIELD_FROM_RTK = False
SLOW_MODE_MIN_TIME = 3  # seconds
PREV_CUR_POINT_MIN_DIST = 100  # pass by cur points dist between them and prev point is lesser than this, mms
VESC_RPM_FAST = -2500
VESC_RPM_SLOW = -2500
#X_MAX = 186.34951456310679611650485436893
#Y_MAX = 91.747572815533980582524271844661
XY_COEFFICIENT_TO_MM = 1
Z_COEFFICIENT_TO_MM = 1
VIEW_ZONE_POLY_POINTS = [[387, 618], [439, 510], [556, 433], [670, 375], [808, 319], [982, 285], [1143, 279], [1293, 294], [1501, 339], [1635, 395], [1766, 473], [1816, 550], [1867, 637], [1881, 675], [1919, 795], [1942, 926], [1959, 1066], [1964, 1217], [1957, 1321], [1949, 1393], [1874, 1425], [1802, 1457], [1692, 1498], [1555, 1537], [1410, 1567], [1219, 1589], [1081, 1590], [944, 1590], [804, 1575], [679, 1552], [569, 1525], [423, 1475], [330, 1431], [277, 1399], [273, 1289], [279, 1131], [297, 976], [343, 780]]

PERIPHERY_DNN_BACKEND = 5  # cv.dnn: DNN_BACKEND_CUDA = 5; DNN_BACKEND_OPENCV = 3
PERIPHERY_DNN_TARGET = 6  # cv.dnn: DNN_TARGET_CUDA = 6; DNN_TARGET_CUDA_FP16 = 7; DNN_TARGET_CPU = 0
PRECISE_DNN_BACKEND = 5  # cv.dnn: DNN_BACKEND_CUDA = 5; DNN_BACKEND_OPENCV = 3
PRECISE_DNN_TARGET = 6  # cv.dnn: DNN_TARGET_CUDA = 6; DNN_TARGET_CUDA_FP16 = 7; DNN_TARGET_CPU = 0

FILTER_MAX_DIST = 5000  # maximum allowable distance between consecutive points (in millimeters)
FILTER_MIN_DIST = 600  # minimum allowable distance between consecutive points (in millimeters)

EXTRACTION_TUNING_MAX_COUNT = 3
APPLY_THREAD_BUFF_CLEANING = True
BUFF_CLEANING_DELAY = 0  # seconds of waiting before frame reading; should be positive or zero; set to 0 if thread cleaning is used
AVOID_CORK_VIEW_OBSCURING = True  # is True: adds offsets to control points to make a plant to be at the top half of the undistorted zone
DISTANCE_FROM_UNDIST_BORDER = 100  # pixels; the corkscrew will move so that the plant is at this distance from the upper border of undistorted zone if AVOID_CORK_VIEW_OBSCURING is True
USE_EMERGENCY_FIELD_GENERATION = False  # allows to generate field by moving forward for a given duration
EMERGENCY_FIELD_SIZE = 35000  # mms; side of the area that will be created if emergency field creation is enabled
EMERGENCY_MOVING_TIME = 10  # seconds of moving forward for vector getting

CONTINUE_PREVIOUS_PATH = False
PREVIOUS_PATH_POINTS_FILE = "path_points.dat"
PREVIOUS_PATH_INDEX_FILE = "path_index.txt"

DELAY_BEFORE_2ND_SCAN = 0.3

VESC_RPM_UI = -8000
ROBOT_SN = "SN005"
SEEK_DELTA_DISTANCE = 25  # mm; if weed is lost after tuning/getting closer - we do 3 shifts for that value (down, left, right) and trying to find it
EXTRACTIONS_FULL_CYCLES = 1  # count of full extraction loops called after periphery NN detection (should be >= 1)
UI_LANGUAGE = "nl"