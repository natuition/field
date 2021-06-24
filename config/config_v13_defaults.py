"""Configuration file."""


CONFIG_VERSION = "0.4.4"  # still have some obsolete keys


# ======================================================================================================================
# ROBOT HARDWARE PROPERTIES
# ======================================================================================================================
CORK_TO_CAMERA_DISTANCE_X = 0  # # distance between camera and cork on the robot, X axis, relative, mm
CORK_TO_CAMERA_DISTANCE_Y = 30  # distance between camera and cork on the robot, Y axis, relative, mm


# ======================================================================================================================
# NAVIGATION ROUTING SETTINGS
# ======================================================================================================================
#KP = 0.2    # for -10000 : *0.55   wheels turning degrees multiplier
#KI = 0.0092 # fpr rpm-1000 : *0.91
KP = {-2500:0.2, 0:0.2, -10000:0.2*0.55}
KI = {-2500:0.0092, 0:0.0092, -10000:0.0092*0.91}

MANEUVERS_FREQUENCY = 1  # seconds
# max distance in mm-s of robot's deviation from planned moving vector
# if dev. is bigger than this - robot will turn to it's planned moving vector
COURSE_SIDE_DEVIATION_MAX = 50  # max allowed robot's deviation from course im mm-s (threshold)
# distance to stop in mm-s between robot and path ending point
# (its a good idea to keep this value greater than allowed course deviation)
COURSE_DESTINATION_DIFF = 3000
COURSE_ADJ_SMC_VAL = 10  # nav wheels turn value when trying to get back to the course (SHOULD BE POSITIVE VALUE!)
WINDOW = float("inf")   # anyway, the integral is used for only some hundreds of time
# AB moving vector used as A----A1--B, where A1 is point when robot starts moving to next point.
# this determines A1-B distance
AUDIT_MODE = False
MANEUVER_START_DISTANCE = {False:4000, True:4000} 
USE_SPEED_LIMIT = True  # when distance to target point is less than specified in the config
DECREASE_SPEED_TRESHOLD = 5000  # millimeters
SUM_ANGLES_HISTORY_MAX = 1000  # max value and min -value of sum(angles_history), should be positive here, in config
# distance between sides of spiral robot movements, expected to be equal to working area width, may be any positive val
AUDIT_MODE = False
SPIRAL_SIDES_INTERVAL = {False:333, True:2000} 
FIELD_REDUCE_SIZE = 200  # cut field's each side for this value, mms
PREV_CUR_POINT_MIN_DIST = 100  # pass by cur points dist between them and prev point is lesser than this, mms
FILTER_MAX_DIST = 5000  # maximum allowable distance between consecutive points (in millimeters)
FILTER_MIN_DIST = 600  # minimum allowable distance between consecutive points (in millimeters)
USE_EMERGENCY_FIELD_GENERATION = False  # allows to generate field by moving forward for a given duration
EMERGENCY_FIELD_SIZE = 45000  # mms; side of the area that will be created if emergency field creation is enabled
EMERGENCY_MOVING_TIME = 10  # seconds of moving forward for vector getting
CONTINUE_PREVIOUS_PATH = False
PREVIOUS_PATH_POINTS_FILE = "path_points.dat"
PREVIOUS_PATH_INDEX_FILE = "path_index.txt"
FAR_TARGET_THRESHOLD = {-2500:25000 ,0:25000, -10000:25000} # the output of the KP KI is boost if the target is far than this threshold
FAR_TARGET_GAIN = {-2500:1.15 ,0:1.15, -10000:1.15}   # the output of the KP KI is boost by this muliplier if the target is far than the threshold
CLOSE_TARGET_THRESHOLD = {-2500:5000 ,0:5000, -10000:5000} # the output of the KP KI is managed if the target is less than this threshold
SMALL_RAW_ANGLE_SQUARE_THRESHOLD = {-2500:100 ,0:100, -10000:100}  # the output of the KP KI is calm by a muliplier if the raw angle is less than the threshold
SMALL_RAW_ANGLE_SQUARE_GAIN = {-2500:0.9 ,0:0.9, -10000:1}    # the output of the KP KI is calm by this muliplier if the raw angle is less than the threshold
BIG_RAW_ANGLE_SQUARE_THRESHOLD = {-2500:625 ,0:625, -10000:625}  # the output of the KP KI is boost by a muliplier if the raw angle is more than the threshold
BIG_RAW_ANGLE_SQUARE_GAIN = {-2500:1.35 ,0:1.35, -10000:1.35}   # the output of the KP KI is boost by this muliplier if the raw angle is more than the threshold
#OPEN_LOOP_TF_AMPLITUDES =  [5,       5,           5*12/14, 5*12/17,  5*12/20,    5*12/23] # 2500 Sinus angle range when the servo system is in open loop
OPEN_LOOP_TF_AMPLITUDES =  [5*9/9,  5*9/13,      5*9/24,  5*9/28,   5*9/34,      5*9/40] # 10000 Sinus angle range when the servo system is in open loop
OPEN_LOOP_TF_MAX_SAMPLES = [5,        7,          10,      15,      20,          40] #Sinus  sampling range when the servo system is in open loop
OPEN_LOOP_TF_FREQUENCIES = [1/5,     1/7,         1/10,    1/15,    1/20,        1/40] # Sinus frequency range when the servo system is in open loop
OPEN_LOOP_FLAT = 9  # waiting time between two stimuli
OPEN_LOOP_TF_MODE = True # mode enable a sinus angle drive on the robot 
STRAIGHT_DIRECTION_INTEGRAL_DURATION = 10
ORIGIN_AVERAGE_SAMPLES = 5
GPS_CLOCK_JITTER = 0.050
PURSUIT_LIMIT = 3000  # length given in mm :  at -2500rpm the deviation get correct after 3 meter path

# ======================================================================================================================
# EXTRACTION SETTINGS
# ======================================================================================================================
EXTRACTION_DEFAULT_METHOD = "single_center_drop"  # or "five_drops_near_center"
ADDITIONAL_EXTRACTIONS_DISTANCE_X = 25  # mm
ADDITIONAL_EXTRACTIONS_DISTANCE_Y = 30  # mm
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
VESC_RPM_UI = -11500
VESC_RPM_SLOW = -2500
VESC_RPM_FAST = -10000
VESC_RPM_AUDIT = -10000
VESC_MOVING_TIME = float("inf")
VESC_ALIVE_FREQ = 0.5  # freq of sending "keep working" signal to engines when moving
VESC_CHECK_FREQ = 0.001  # freq of checking need to stop
STEP_FORWARD_TIME = 1  # step after extraction loops are done
STEP_FORWARD_RPM = -2500  # # step after extraction loops are done #-2500 à remettre
FAST_TO_SLOW_RPM = 2500
FAST_TO_SLOW_TIME = 5


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
X_MAX = 430
Y_MIN = 0
Y_MAX = 220
Z_MIN = -float("inf")
Z_MAX = 100

XY_F_MIN = 1
XY_F_MAX = 5500 
Z_F_MIN = 1
Z_F_MAX = 2000 

XY_COEFFICIENT_TO_MM = 1
Z_COEFFICIENT_TO_MM = 1  # not used yet

EXTRACTION_Z = 65  # value that passed to smoothie when doing plant extration
Z_F_EXTRACTION_UP = 1350
Z_F_EXTRACTION_DOWN = 1950
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
A_MIN = -9 #-38/6
A_MAX = 9 #38/6
B_MIN = -float("inf")
B_MAX = float("inf")
C_MIN = -float("inf")
C_MAX = float("inf")

A_F_MIN = 1
A_F_MAX = 4000
A_F_UI = 1000
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
PERIPHERY_CONFIDENCE_THRESHOLD = 0.1 #0.08 y16 vertys  # Confidence threshold
PERIPHERY_HIER_THRESHOLD = 0.5  # works only in darknet wrapper
PERIPHERY_NMS_THRESHOLD = 0.4      # Non-maximum suppression threshold
PERIPHERY_INPUT_SIZE = (416, 416)
PERIPHERY_CONFIG_FILE = "yolo/Y0016_416.cfg"
PERIPHERY_WEIGHTS_FILE = "yolo/Y0016.weights"
PERIPHERY_CLASSES_FILE = "yolo/Y0016.names"
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
PRECISE_CONFIG_FILE = "yolo/Y0016_832.cfg"
PRECISE_WEIGHTS_FILE = "yolo/Y0016.weights"
PRECISE_CLASSES_FILE = "yolo/Y0016.names"
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
CROP_W_FROM = 661
CROP_W_TO = 2708
CROP_H_FROM = 257
CROP_H_TO = 1551
CAMERA_FRAMERATE = 8
CAMERA_FLIP_METHOD = 0
SCENE_CENTER_X = 979  # 1788 for uncropped
SCENE_CENTER_Y = 994  # 1368 for uncropped
ONE_MM_IN_PX = 5.2
ISP_DIGITAL_GAIN_RANGE_FROM = 4
ISP_DIGITAL_GAIN_RANGE_TO = 4
GAIN_RANGE_FROM = 4
GAIN_RANGE_TO = 4
EXPOSURE_TIME_RANGE_FROM = 660000
EXPOSURE_TIME_RANGE_TO = 660000
AE_LOCK = True
CV_APPLY_ROTATION = False
CV_ROTATE_CODE = 2
APPLY_THREAD_BUFF_CLEANING = True
BUFF_CLEANING_DELAY = 0  # seconds of waiting before frame reading; should be positive or zero; set to 0 if thread cleaning is used
VIEW_ZONE_POLY_POINTS = [[387, 618], [439, 510], [556, 433], [670, 375], [808, 319], [982, 285], [1143, 279], [1293, 294], [1501, 339], [1635, 395], [1766, 473], [1816, 550], [1867, 637], [1881, 675], [1919, 795], [1942, 926], [1959, 1066], [1964, 1217], [1957, 1321], [1949, 1393], [1874, 1425], [1802, 1457], [1692, 1498], [1555, 1537], [1410, 1567], [1219, 1589], [1081, 1590], [944, 1590], [804, 1575], [679, 1552], [569, 1525], [423, 1475], [330, 1431], [277, 1399], [273, 1289], [279, 1131], [297, 976], [343, 780]]
WORKING_ZONE_POLY_POINTS = [[232, 1105], [231, 942], [243, 781], [271, 648], [338, 455], [450, 399], [569, 355], [726, 314], [870, 295], [996, 289], [1128, 297], [1251, 316], [1383, 351], [1530, 406], [1621, 444], [1663, 552], [1690, 648], [1713, 762], [1727, 863], [1732, 944], [1731, 1086], [1588, 1104], [1418, 1117], [1230, 1130], [1097, 1133], [683, 1136], [397, 1118], [232, 1104]]
IMAGE_CONTROL_POINTS_MAP = [[981, 995, 0, 0, 1], [876, 996, -20, 0, 2], [778, 998, -40, 0, 3], [686, 998, -60, 0, 4], [601, 1000, -80, 0, 5], [524, 1001, -100, 0, 6], [460, 1002, -120, 0, 7], [400, 1006, -140, 0, 8], [348, 1009, -160, 0, 9], [305, 1007, -180, 0, 10], [270, 1008, -200, 0, 11], [980, 892, 0, 20, 12], [877, 893, -20, 20, 13], [776, 895, -40, 20, 14], [683, 900, -60, 20, 15], [599, 906, -80, 20, 16], [521, 912, -100, 20, 17], [456, 916, -120, 20, 18], [397, 923, -140, 20, 19], [348, 929, -160, 20, 20], [305, 932, -180, 20, 21], [269, 936, -200, 20, 22], [979, 791, 0, 40, 23], [876, 792, -20, 40, 24], [779, 798, -40, 40, 25], [685, 805, -60, 40, 26], [602, 812, -80, 40, 27], [524, 821, -100, 40, 28], [458, 832, -120, 40, 29], [401, 841, -140, 40, 30], [349, 851, -160, 40, 31], [308, 860, -180, 40, 32], [269, 867, -200, 40, 33], [978, 697, 0, 60, 34], [878, 700, -20, 60, 35], [783, 706, -40, 60, 36], [691, 713, -60, 60, 37], [607, 725, -80, 60, 38], [534, 737, -100, 60, 39], [467, 749, -120, 60, 40], [409, 763, -140, 60, 41], [359, 774, -160, 60, 42], [315, 787, -180, 60, 43], [279, 798, -200, 60, 44], [977, 612, 0, 80, 45], [881, 616, -20, 80, 46], [788, 622, -40, 80, 47], [700, 633, -60, 80, 48], [619, 644, -80, 80, 49], [544, 658, -100, 80, 50], [480, 674, -120, 80, 51], [422, 690, -140, 80, 52], [371, 703, -160, 80, 53], [326, 719, -180, 80, 54], [285, 734, -200, 80, 55], [976, 538, 0, 100, 56], [885, 541, -20, 100, 57], [793, 547, -40, 100, 58], [711, 558, -60, 100, 59], [630, 572, -80, 100, 60], [559, 586, -100, 100, 61], [494, 602, -120, 100, 62], [435, 620, -140, 100, 63], [384, 638, -160, 100, 64], [338, 657, -180, 100, 65], [301, 672, -200, 100, 66], [976, 472, 0, 120, 67], [887, 475, -20, 120, 68], [801, 481, -40, 120, 69], [720, 491, -60, 120, 70], [642, 505, -80, 120, 71], [573, 521, -100, 120, 72], [509, 537, -120, 120, 73], [450, 556, -140, 120, 74], [400, 576, -160, 120, 75], [355, 596, -180, 120, 76], [313, 614, -200, 120, 77], [976, 414, 0, 140, 78], [892, 415, -20, 140, 79], [810, 422, -40, 140, 80], [730, 432, -60, 140, 81], [657, 446, -80, 140, 82], [585, 462, -100, 140, 83], [525, 480, -120, 140, 84], [467, 500, -140, 140, 85], [416, 520, -160, 140, 86], [366, 541, -180, 140, 87], [329, 558, -200, 140, 88], [975, 362, 0, 160, 89], [894, 364, -20, 160, 90], [818, 371, -40, 160, 91], [742, 381, -60, 160, 92], [668, 396, -80, 160, 93], [601, 411, -100, 160, 94], [539, 432, -120, 160, 95], [485, 450, -140, 160, 96], [435, 469, -160, 160, 97], [386, 495, -180, 160, 98], [344, 512, -200, 160, 99], [974, 320, 0, 180, 100], [899, 321, -20, 180, 101], [824, 330, -40, 180, 102], [752, 338, -60, 180, 103], [683, 351, -80, 180, 104], [619, 367, -100, 180, 105], [557, 386, -120, 180, 106], [505, 405, -140, 180, 107], [455, 424, -160, 180, 108], [408, 444, -180, 180, 109], [367, 461, -200, 180, 110], [1083, 994, 20, 0, 111], [1180, 992, 40, 0, 112], [1275, 992, 60, 0, 113], [1357, 991, 80, 0, 114], [1434, 990, 100, 0, 115], [1500, 990, 120, 0, 116], [1559, 992, 140, 0, 117], [1609, 994, 160, 0, 118], [1652, 993, 180, 0, 119], [1691, 995, 200, 0, 120], [1082, 891, 20, 20, 121], [1181, 893, 40, 20, 122], [1273, 895, 60, 20, 123], [1358, 898, 80, 20, 124], [1434, 901, 100, 20, 125], [1500, 907, 120, 20, 126], [1559, 912, 140, 20, 127], [1608, 916, 160, 20, 128], [1651, 920, 180, 20, 129], [1691, 923, 200, 20, 130], [1078, 791, 20, 40, 131], [1178, 795, 40, 40, 132], [1268, 800, 60, 40, 133], [1352, 808, 80, 40, 134], [1427, 814, 100, 40, 135], [1495, 823, 120, 40, 136], [1552, 830, 140, 40, 137], [1603, 839, 160, 40, 138], [1646, 846, 180, 40, 139], [1686, 854, 200, 40, 140], [1077, 698, 20, 60, 141], [1173, 702, 40, 60, 142], [1264, 710, 60, 60, 143], [1344, 721, 80, 60, 144], [1419, 731, 100, 60, 145], [1483, 741, 120, 60, 146], [1543, 754, 140, 60, 147], [1593, 766, 160, 60, 148], [1639, 775, 180, 60, 149], [1677, 785, 200, 60, 150], [1072, 612, 20, 80, 151], [1168, 619, 40, 80, 152], [1254, 628, 60, 80, 153], [1335, 639, 80, 80, 154], [1407, 655, 100, 80, 155], [1473, 668, 120, 80, 156], [1530, 681, 140, 80, 157], [1580, 694, 160, 80, 158], [1627, 710, 180, 80, 159], [1666, 723, 200, 80, 160], [1068, 540, 20, 100, 161], [1159, 545, 40, 100, 162], [1244, 552, 60, 100, 163], [1322, 566, 80, 100, 164], [1393, 580, 100, 100, 165], [1457, 597, 120, 100, 166], [1516, 612, 140, 100, 167], [1568, 628, 160, 100, 168], [1612, 644, 180, 100, 169], [1652, 661, 200, 100, 170], [1064, 474, 20, 120, 171], [1149, 479, 40, 120, 172], [1231, 487, 60, 120, 173], [1308, 500, 80, 120, 174], [1380, 515, 100, 120, 175], [1444, 533, 120, 120, 176], [1502, 549, 140, 120, 177], [1552, 567, 160, 120, 178], [1597, 586, 180, 120, 179], [1634, 600, 200, 120, 180], [1059, 413, 20, 140, 181], [1141, 420, 40, 140, 182], [1220, 429, 60, 140, 183], [1295, 442, 80, 140, 184], [1364, 458, 100, 140, 185], [1426, 474, 120, 140, 186], [1482, 491, 140, 140, 187], [1533, 509, 160, 140, 188], [1579, 531, 180, 140, 189], [1617, 544, 200, 140, 190], [1055, 364, 20, 160, 191], [1133, 369, 40, 160, 192], [1208, 380, 60, 160, 193], [1280, 393, 80, 160, 194], [1346, 409, 100, 160, 195], [1407, 424, 120, 160, 196], [1466, 443, 140, 160, 197], [1522, 462, 160, 160, 198], [1563, 479, 180, 160, 199], [1603, 496, 200, 160, 200], [1050, 319, 20, 180, 201], [1126, 328, 40, 180, 202], [1199, 340, 60, 180, 203], [1266, 348, 80, 180, 204], [1329, 363, 100, 180, 205], [1393, 379, 120, 180, 206], [1448, 397, 140, 180, 207], [1507, 416, 160, 180, 208], [1548, 430, 180, 180, 209], [1584, 446, 200, 180, 210]]
UNDISTORTED_ZONE_RADIUS = 240


# ======================================================================================================================
# APP SETTINGS
# ======================================================================================================================
SAVE_DEBUG_IMAGES = False
DEBUG_IMAGES_PATH = "debug_images/"
ALLOW_GATHERING = False
FILES_TO_KEEP_COUNT = 600
RECEIVE_FIELD_FROM_RTK = False
EXTRACTION_TUNING_MAX_COUNT = 3
DELAY_BEFORE_2ND_SCAN = 0.3  # delay in seconds after robot stop and before second scan (M=1)
DB_NAME = "dbname"
DB_USER = "username"
DB_HOST = "x.x.x.x"
DB_PWD = "password"
ROBOT_SN = "SNXXX"
UI_LANGUAGE = "fr"
AUDIT_MODE = False
AUDIT_DIVIDER = 6
SLOW_FAST_MODE = False
SLOW_MODE_MIN_TIME = 3  # seconds
AUDIT_OUTPUT_FILE = "audit.txt"
LOG_ROOT_DIR = "logs/"
STATISTICS_OUTPUT_FILE = "statistics.txt"
DATA_GATHERING_DIR = "gathered_data/"
CORK_CALIBRATION_MIN_TIME = 360000
QUEUE_NAME_UI_MAIN = "/queue_ui_main"
ALLOW_DETECT_AND_EXTRACT_GROUP = False
VERBOSE = True

# ======================================================================================================================
# MATRIX SETTINGS
# ======================================================================================================================
#DETECTION
MATRIX_PLANT_ROOT = 1
MATRIX_PLANT_LEAF = 2 # All leaft will be number greater than this parameter (2 = one leaft, 3 = two leaft, ...)
GROUP_THRESHOLD = 3
#EXTRACTION
MATRIX_EXTRACTION = 1
MATRIX_EXTRACTION_PATTERN = 2 # All different pattern will be number greater than this parameter (2 = pattern one in ExtractionMethods, 3 = pattern two in ExtractionMethods, ...)
#ALL
MATRIX_ONE_MATRICE_CELL_IN_MM = 10
OFFSET_FOR_MATRIX_BORDER_IN_CELL = 5
OFFSET_FOR_MATRIX_PATTERN_IN_MM = 20
DEBUG_MATRIX_FILE = False

# ======================================================================================================================
# PREDICTION SETTINGS
# ======================================================================================================================
ZONE_THRESHOLD_DEGREE = {436:5,697:7,796:17,849:15,953:6}


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
