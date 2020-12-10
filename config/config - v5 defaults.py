"""Configuration file."""


CONFIG_VERSION = "0.4.3"  # still have some obsolete keys


# ======================================================================================================================
# ROBOT HARDWARE PROPERTIES
# ======================================================================================================================
CORK_TO_CAMERA_DISTANCE_X = 0  # # distance between camera and cork on the robot, X axis, relative, mm
CORK_TO_CAMERA_DISTANCE_Y = 30  # distance between camera and cork on the robot, Y axis, relative, mm


# ======================================================================================================================
# NAVIGATION ROUTING SETTINGS
# ======================================================================================================================
KP = 0.21  # wheels turning degrees multiplier
KI = 0.0094
MANEUVERS_FREQUENCY = 1  # seconds
# max distance in mm-s of robot's deviation from planned moving vector
# if dev. is bigger than this - robot will turn to it's planned moving vector
COURSE_SIDE_DEVIATION_MAX = 50  # max allowed robot's deviation from course im mm-s (threshold)
# distance to stop in mm-s between robot and path ending point
# (its a good idea to keep this value greater than allowed course deviation)
COURSE_DESTINATION_DIFF = 3000
COURSE_ADJ_SMC_VAL = 10  # nav wheels turn value when trying to get back to the course (SHOULD BE POSITIVE VALUE!)
WINDOW = float("inf")
# AB moving vector used as A----A1--B, where A1 is point when robot starts moving to next point.
# this determines A1-B distance
MANEUVER_START_DISTANCE = 6500
USE_SPEED_LIMIT = True  # when distance to target point is less than specified in the config
DECREASE_SPEED_TRESHOLD = 5000  # millimeters
SUM_ANGLES_HISTORY_MAX = 1000  # max value and min -value of sum(angles_history), should be positive here, in config
# distance between sides of spiral robot movements, expected to be equal to working area width, may be any positive val
SPIRAL_SIDES_INTERVAL = 333
FIELD_REDUCE_SIZE = 200  # cut field's each side for this value, mms
PREV_CUR_POINT_MIN_DIST = 100  # pass by cur points dist between them and prev point is lesser than this, mms
FILTER_MAX_DIST = 5000  # maximum allowable distance between consecutive points (in millimeters)
FILTER_MIN_DIST = 600  # minimum allowable distance between consecutive points (in millimeters)
USE_EMERGENCY_FIELD_GENERATION = False  # allows to generate field by moving forward for a given duration
EMERGENCY_FIELD_SIZE = 15000  # mms; side of the area that will be created if emergency field creation is enabled
EMERGENCY_MOVING_TIME = 10  # seconds of moving forward for vector getting
CONTINUE_PREVIOUS_PATH = True
PREVIOUS_PATH_POINTS_FILE = "path_points.dat"
PREVIOUS_PATH_INDEX_FILE = "path_index.txt"
FAR_TARGET_THRESHOLD = 25000 # the output of the KP KI is boost if the target is far than this threshold
FAR_TARGET_GAIN = 1.15  # the output of the KP KI is boost by this muliplier if the target is far than the threshold
CLOSE_TARGET_THRESHOLD = 10000 # the output of the KP KI is managed if the target is less than this threshold
SMALL_RAW_ANGLE_SQUARE_THRESHOLD = 100  # the output of the KP KI is calm by a muliplier if the raw angle is less than the threshold
SMALL_RAW_ANGLE_SQUARE_GAIN = 0.9# the output of the KP KI is calm by this muliplier if the raw angle is less than the threshold
BIG_RAW_ANGLE_SQUARE_THRESHOLD = 625  # the output of the KP KI is boost by a muliplier if the raw angle is more than the threshold
BIG_RAW_ANGLE_SQUARE_GAIN = 1.35# the output of the KP KI is boost by this muliplier if the raw angle is more than the threshold



# ======================================================================================================================
# EXTRACTION SETTINGS
# ======================================================================================================================
EXTRACTION_DEFAULT_METHOD = "single_center_drop"  # or "five_drops_near_center"
ADDITIONAL_EXTRACTIONS_DISTANCE_X = 20  # mm
ADDITIONAL_EXTRACTIONS_DISTANCE_Y = 20  # mm
AVOID_CORK_VIEW_OBSCURING = True  # is True: adds offsets to control points to make a plant to be at the top half of the undistorted zone
DISTANCE_FROM_UNDIST_BORDER = 100  # pixels; the corkscrew will move so that the plant is at this distance from the upper border of undistorted zone if AVOID_CORK_VIEW_OBSCURING is True
EXTRACTIONS_FULL_CYCLES = 2  # count of full extraction loops called after periphery NN detection (should be >= 1)
SEEK_DELTA_DISTANCE = 25  # mm; if weed is lost after tuning/getting closer - we do 3 shifts for that value (down, left, right) and trying to find it


# ======================================================================================================================
# PATHS SETTINGS
# ======================================================================================================================
INPUT_GPS_FIELD_FILE = "field.txt"
OUTPUT_GPS_HISTORY_FILE = "gps_history.txt"
DARKNET_LIB_DIR_PATH = "/home/violette/field/darknet/"

# ======================================================================================================================
# VESC SETTINGS
# ======================================================================================================================
VESC_PORT = "/dev/ttyACM0"
VESC_BAUDRATE = 115200
VESC_RPM_UI = -8000
VESC_RPM_FAST = -2500
VESC_RPM_SLOW = -2500
VESC_MOVING_TIME = float("inf")
VESC_ALIVE_FREQ = 0.5  # freq of sending "keep working" signal to engines when moving
VESC_CHECK_FREQ = 0.001  # freq of checking need to stop
STEP_FORWARD_TIME = 1  # step after extraction loops are done
STEP_FORWARD_RPM = -2500  # # step after extraction loops are done


# ======================================================================================================================
# GPS SETTINGS
# ======================================================================================================================
GPS_PORT = "/dev/ttyTHS1"
GPS_BAUDRATE = 19200
GPS_POSITIONS_TO_KEEP = 1000


# ======================================================================================================================
# SMOOTHIE SETTINGS
# ======================================================================================================================
SMOOTHIE_HOST = "/dev/ttyACM1"  # smoothie's ip address for telnet or port for usb serial connector
SMOOTHIE_BAUDRATE = 115200
SMOOTHIE_BACKEND = 2  # 1 = telnet, 2 = serial

# EXTRACTION
X_MIN = 0
X_MAX = 440
Y_MIN = 0
Y_MAX = 248
Z_MIN = -float("inf")
Z_MAX = 100

XY_F_MIN = 1
XY_F_MAX = 8000 
Z_F_MIN = 1
Z_F_MAX = 2000 

XY_COEFFICIENT_TO_MM = 1
Z_COEFFICIENT_TO_MM = 1  # not used yet

EXTRACTION_Z = 45  # value that passed to smoothie when doing plant extration
Z_F_EXTRACTION_UP = 1350
Z_F_EXTRACTION_DOWN = 1900
ALLOW_PRECISE_RESCAN = True

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
A_F_MAX = 4000
B_F_MIN = 1
B_F_MAX = 4000
C_F_MIN = 1
C_F_MAX = 1000

A_ONE_DEGREE_IN_SMOOTHIE = 2  # A axis
A_DEGREES_PER_SECOND = 5  # A axis
NAV_TURN_WHEELS_CENTER = 0


# ======================================================================================================================
# YOLO PERIPHERY NETWORK SETTINGS
# ======================================================================================================================
PERIPHERY_CONFIDENCE_THRESHOLD = 0.999    # Confidence threshold
PERIPHERY_HIER_THRESHOLD = 0.5  # works only in darknet wrapper
PERIPHERY_NMS_THRESHOLD = 0.4      # Non-maximum suppression threshold
PERIPHERY_INPUT_SIZE = (416, 416)
PERIPHERY_CONFIG_FILE = "yolo/Y0014.cfg"
PERIPHERY_WEIGHTS_FILE = "yolo/Y0014.weights"
PERIPHERY_CLASSES_FILE = "yolo/Y0014.names"
PERIPHERY_DNN_BACKEND = 5  # cv.dnn: DNN_BACKEND_CUDA = 5; DNN_BACKEND_OPENCV = 3
PERIPHERY_DNN_TARGET = 6  # cv.dnn: DNN_TARGET_CUDA = 6; DNN_TARGET_CUDA_FP16 = 7; DNN_TARGET_CPU = 0
PERIPHERY_WRAPPER = 1  # 1 = darknet, 2 = opencv from darknet
PERIPHERY_DATA_FILE ="yolo/Y0014.data"

# ======================================================================================================================
# YOLO PRECISE NETWORK SETTINGS
# ======================================================================================================================
PRECISE_CONFIDENCE_THRESHOLD = 0.1    # Confidence threshold
PRECISE_HIER_THRESHOLD = 0.5  # works only in darknet wrapper
PRECISE_NMS_THRESHOLD = 0.4      # Non-maximum suppression threshold
PRECISE_INPUT_SIZE = (832, 832)
PRECISE_CONFIG_FILE = "yolo/Y0014.cfg"
PRECISE_WEIGHTS_FILE = "yolo/Y0014.weights"
PRECISE_CLASSES_FILE = "yolo/Y0014.names"
PRECISE_DATA_FILE ="yolo/Y0014.data"
PRECISE_DNN_BACKEND = 5  # cv.dnn: DNN_BACKEND_CUDA = 5; DNN_BACKEND_OPENCV = 3
PRECISE_DNN_TARGET = 6  # cv.dnn: DNN_TARGET_CUDA = 6; DNN_TARGET_CUDA_FP16 = 7; DNN_TARGET_CPU = 0
PRECISE_WRAPPER = 1  # 1 = darknet, 2 = opencv from darknet


# ======================================================================================================================
# CAMERA SETTINGS
# ======================================================================================================================
CAMERA_W = 3280
CAMERA_H = 2464
APPLY_IMAGE_CROPPING = True
CROP_W_FROM = 771
CROP_W_TO = 2981
CROP_H_FROM = 325
CROP_H_TO = 1966
CAMERA_FRAMERATE = 5
CAMERA_FLIP_METHOD = 0
SCENE_CENTER_X = 1017  # 1788 for uncropped
SCENE_CENTER_Y = 1043  # 1368 for uncropped
ONE_MM_IN_PX = 6
ISP_DIGITAL_GAIN_RANGE_FROM = 1
ISP_DIGITAL_GAIN_RANGE_TO = 1
GAIN_RANGE_FROM = 1
GAIN_RANGE_TO = 1
EXPOSURE_TIME_RANGE_FROM = 1000000
EXPOSURE_TIME_RANGE_TO = 1000000
AE_LOCK = True
CV_APPLY_ROTATION = False
CV_ROTATE_CODE = 2
APPLY_THREAD_BUFF_CLEANING = True
BUFF_CLEANING_DELAY = 0  # seconds of waiting before frame reading; should be positive or zero; set to 0 if thread cleaning is used
VIEW_ZONE_POLY_POINTS = [[387, 618], [439, 510], [556, 433], [670, 375], [808, 319], [982, 285], [1143, 279], [1293, 294], [1501, 339], [1635, 395], [1766, 473], [1816, 550], [1867, 637], [1881, 675], [1919, 795], [1942, 926], [1959, 1066], [1964, 1217], [1957, 1321], [1949, 1393], [1874, 1425], [1802, 1457], [1692, 1498], [1555, 1537], [1410, 1567], [1219, 1589], [1081, 1590], [944, 1590], [804, 1575], [679, 1552], [569, 1525], [423, 1475], [330, 1431], [277, 1399], [273, 1289], [279, 1131], [297, 976], [343, 780]]
WORKING_ZONE_POLY_POINTS = [[265, 1155], [264, 1020], [282, 846], [333, 652], [389, 512], [508, 456], [652, 405], [828, 366], [1026, 351], [1177, 359], [1285, 376], [1422, 407], [1536, 447], [1664, 503], [1708, 607], [1747, 731], [1771, 849], [1788, 1035], [1785, 1166], [1515, 1197], [1276, 1214], [1172, 1218], [854, 1216], [627, 1202], [424, 1182], [265, 1155]]
IMAGE_CONTROL_POINTS_MAP = [[1016, 1044, 0, 0, 1], [911, 1042, -20, 0, 2], [807, 1042, -40, 0, 3], [713, 1042, -60, 0, 4], [627, 1042, -80, 0, 5], [550, 1042, -100, 0, 6], [483, 1043, -120, 0, 7], [429, 1042, -140, 0, 8], [379, 1042, -160, 0, 9], [340, 1043, -180, 0, 10], [303, 1044, -200, 0, 11], [1017, 936, 0, 20, 12], [912, 938, -20, 20, 13], [811, 939, -40, 20, 14], [714, 941, -60, 20, 15], [629, 947, -80, 20, 16], [553, 951, -100, 20, 17], [488, 955, -120, 20, 18], [429, 961, -140, 20, 19], [382, 964, -160, 20, 20], [341, 969, -180, 20, 21], [305, 972, -200, 20, 22], [1018, 836, 0, 40, 23], [915, 835, -20, 40, 24], [816, 839, -40, 40, 25], [720, 844, -60, 40, 26], [637, 852, -80, 40, 27], [560, 859, -100, 40, 28], [495, 870, -120, 40, 29], [440, 877, -140, 40, 30], [389, 885, -160, 40, 31], [347, 894, -180, 40, 32], [312, 903, -200, 40, 33], [1020, 742, 0, 60, 34], [919, 743, -20, 60, 35], [823, 749, -40, 60, 36], [732, 756, -60, 60, 37], [649, 764, -80, 60, 38], [574, 777, -100, 60, 39], [508, 788, -120, 60, 40], [452, 799, -140, 60, 41], [403, 812, -160, 60, 42], [359, 824, -180, 60, 43], [321, 836, -200, 60, 44], [1021, 658, 0, 80, 45], [925, 660, -20, 80, 46], [831, 665, -40, 80, 47], [744, 674, -60, 80, 48], [663, 687, -80, 80, 49], [590, 700, -100, 80, 50], [525, 715, -120, 80, 51], [468, 728, -140, 80, 52], [420, 742, -160, 80, 53], [372, 757, -180, 80, 54], [333, 771, -200, 80, 55], [1022, 586, 0, 100, 56], [931, 586, -20, 100, 57], [841, 594, -40, 100, 58], [757, 603, -60, 100, 59], [678, 615, -80, 100, 60], [606, 630, -100, 100, 61], [543, 646, -120, 100, 62], [486, 660, -140, 100, 63], [434, 678, -160, 100, 64], [389, 693, -180, 100, 65], [346, 713, -200, 100, 66], [1024, 519, 0, 120, 67], [937, 522, -20, 120, 68], [851, 528, -40, 120, 69], [770, 538, -60, 120, 70], [692, 550, -80, 120, 71], [624, 564, -100, 120, 72], [561, 581, -120, 120, 73], [504, 600, -140, 120, 74], [454, 617, -160, 120, 75], [403, 639, -180, 120, 76], [358, 660, -200, 120, 77], [1025, 463, 0, 140, 78], [941, 464, -20, 140, 79], [861, 471, -40, 140, 80], [783, 481, -60, 140, 81], [709, 494, -80, 140, 82], [641, 508, -100, 140, 83], [579, 525, -120, 140, 84], [522, 545, -140, 140, 85], [472, 560, -160, 140, 86], [429, 576, -180, 140, 87], [383, 593, -200, 140, 88], [1027, 411, 0, 160, 89], [946, 416, -20, 160, 90], [869, 421, -40, 160, 91], [794, 433, -60, 160, 92], [725, 445, -80, 160, 93], [657, 459, -100, 160, 94], [598, 476, -120, 160, 95], [542, 494, -140, 160, 96], [493, 511, -160, 160, 97], [444, 532, -180, 160, 98], [399, 553, -200, 160, 99], [1027, 372, 0, 180, 100], [952, 372, -20, 180, 101], [880, 381, -40, 180, 102], [808, 390, -60, 180, 103], [740, 404, -80, 180, 104], [676, 418, -100, 180, 105], [619, 434, -120, 180, 106], [562, 452, -140, 180, 107], [507, 477, -160, 180, 108], [458, 496, -180, 180, 109], [411, 519, -200, 180, 110], [1123, 1044, 20, 0, 111], [1226, 1044, 40, 0, 112], [1321, 1045, 60, 0, 113], [1410, 1047, 80, 0, 114], [1490, 1048, 100, 0, 115], [1557, 1051, 120, 0, 116], [1621, 1052, 140, 0, 117], [1679, 1053, 160, 0, 118], [1726, 1055, 180, 0, 119], [1762, 1058, 200, 0, 120], [1124, 938, 20, 20, 121], [1227, 942, 40, 20, 122], [1321, 946, 60, 20, 123], [1408, 951, 80, 20, 124], [1486, 957, 100, 20, 125], [1555, 963, 120, 20, 126], [1618, 974, 140, 20, 127], [1676, 980, 160, 20, 128], [1719, 987, 180, 20, 129], [1752, 992, 200, 20, 130], [1122, 836, 20, 40, 131], [1222, 843, 40, 40, 132], [1319, 850, 60, 40, 133], [1401, 858, 80, 40, 134], [1481, 866, 100, 40, 135], [1549, 876, 120, 40, 136], [1608, 885, 140, 40, 137], [1661, 892, 160, 40, 138], [1704, 900, 180, 40, 139], [1741, 909, 200, 40, 140], [1122, 743, 20, 60, 141], [1219, 748, 40, 60, 142], [1312, 757, 60, 60, 143], [1395, 769, 80, 60, 144], [1470, 781, 100, 60, 145], [1539, 794, 120, 60, 146], [1598, 804, 140, 60, 147], [1648, 814, 160, 60, 148], [1696, 830, 180, 60, 149], [1735, 840, 200, 60, 150], [1118, 659, 20, 80, 151], [1213, 664, 40, 80, 152], [1305, 673, 60, 80, 153], [1385, 688, 80, 80, 154], [1460, 703, 100, 80, 155], [1525, 717, 120, 80, 156], [1585, 734, 140, 80, 157], [1636, 746, 160, 80, 158], [1684, 761, 180, 80, 159], [1719, 772, 200, 80, 160], [1117, 585, 20, 100, 161], [1205, 591, 40, 100, 162], [1293, 600, 60, 100, 163], [1373, 611, 80, 100, 164], [1444, 628, 100, 100, 165], [1510, 645, 120, 100, 166], [1571, 664, 140, 100, 167], [1623, 682, 160, 100, 168], [1664, 695, 180, 100, 169], [1704, 712, 200, 100, 170], [1111, 519, 20, 120, 171], [1198, 526, 40, 120, 172], [1281, 537, 60, 120, 173], [1359, 549, 80, 120, 174], [1431, 564, 100, 120, 175], [1493, 581, 120, 120, 176], [1557, 601, 140, 120, 177], [1606, 618, 160, 120, 178], [1647, 632, 180, 120, 179], [1681, 644, 200, 120, 180], [1108, 464, 20, 140, 181], [1192, 472, 40, 140, 182], [1269, 483, 60, 140, 183], [1344, 495, 80, 140, 184], [1412, 509, 100, 140, 185], [1477, 525, 120, 140, 186], [1532, 543, 140, 140, 187], [1587, 562, 160, 140, 188], [1629, 578, 180, 140, 189], [1665, 593, 200, 140, 190], [1104, 416, 20, 160, 191], [1183, 422, 40, 160, 192], [1258, 433, 60, 160, 193], [1328, 445, 80, 160, 194], [1396, 460, 100, 160, 195], [1458, 476, 120, 160, 196], [1517, 495, 140, 160, 197], [1565, 510, 160, 160, 198], [1609, 523, 180, 160, 199], [1642, 534, 200, 160, 200], [1103, 374, 20, 180, 201], [1176, 381, 40, 180, 202], [1247, 389, 60, 180, 203], [1315, 400, 80, 180, 204], [1383, 419, 100, 180, 205], [1442, 433, 120, 180, 206], [1497, 452, 140, 180, 207], [1548, 470, 160, 180, 208], [1591, 487, 180, 180, 209], [1631, 505, 200, 180, 210]]
UNDISTORTED_ZONE_RADIUS = 240


# ======================================================================================================================
# APP SETTINGS
# ======================================================================================================================
SAVE_DEBUG_IMAGES = False
DEBUG_IMAGES_PATH = "debug_images/"
ALLOW_GATHERING = True
FILES_TO_KEEP_COUNT = 600
RECEIVE_FIELD_FROM_RTK = False
SLOW_MODE_MIN_TIME = 3  # seconds
EXTRACTION_TUNING_MAX_COUNT = 3
DELAY_BEFORE_2ND_SCAN = 0.3  # delay in seconds after robot stop and before second scan (M=1)
DB_NAME = "dbname"
DB_USER = "username"
DB_HOST = "x.x.x.x"
DB_PWD = "password"
ROBOT_SN = "SN006"
UI_LANGUAGE = "fr"
AUDIT_MODE = False
AUDIT_OUTPUT_FILE = "audit.txt"
LOG_ROOT_DIR = "logs/"
STATISTICS_OUTPUT_FILE = "statistics.txt"
DATA_GATHERING_DIR = "gathered_data/"
CORK_CALIBRATION_MIN_TIME = 3600
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
