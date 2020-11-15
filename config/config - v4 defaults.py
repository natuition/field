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
FILTER_MAX_DIST = 5000  # maximum allowable distance between consecutive points (in millimeters)
FILTER_MIN_DIST = 600  # minimum allowable distance between consecutive points (in millimeters)
USE_EMERGENCY_FIELD_GENERATION = False  # allows to generate field by moving forward for a given duration
EMERGENCY_FIELD_SIZE = 15000  # mms; side of the area that will be created if emergency field creation is enabled
EMERGENCY_MOVING_TIME = 3  # seconds of moving forward for vector getting
CONTINUE_PREVIOUS_PATH = False
PREVIOUS_PATH_POINTS_FILE = "path_points.dat"
PREVIOUS_PATH_INDEX_FILE = "path_index.txt"


# ======================================================================================================================
# EXTRACTION SETTINGS
# ======================================================================================================================
EXTRACTION_DEFAULT_METHOD = "five_drops_near_center"  # or "single_center_drop"
ADDITIONAL_EXTRACTIONS_DISTANCE_X = 10  # mm
ADDITIONAL_EXTRACTIONS_DISTANCE_Y = 10  # mm
AVOID_CORK_VIEW_OBSCURING = True  # is True: adds offsets to control points to make a plant to be at the top half of the undistorted zone
DISTANCE_FROM_UNDIST_BORDER = 100  # pixels; the corkscrew will move so that the plant is at this distance from the upper border of undistorted zone if AVOID_CORK_VIEW_OBSCURING is True
EXTRACTIONS_FULL_CYCLES = 2  # count of full extraction loops called after periphery NN detection (should be >= 1)
SEEK_DELTA_DISTANCE = 25  # mm; if weed is lost after tuning/getting closer - we do 3 shifts for that value (down, left, right) and trying to find it


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
VESC_RPM_UI = -8000
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
SMOOTHIE_BAUDRATE = 115200
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

EXTRACTION_Z = -50  # value that passed to smoothie when doing plant extration
Z_F_EXTRACTION_UP = 1700
Z_F_EXTRACTION_DOWN = 1300

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
PERIPHERY_CONFIDENCE_THRESHOLD = 0.25  # Confidence threshold
PERIPHERY_HIER_THRESHOLD = 0.50
PERIPHERY_NMS_THRESHOLD = 0.4  # Non-maximum suppression threshold
PERIPHERY_INPUT_SIZE = (416, 416)
PERIPHERY_CONFIG_FILE = "yolo/periphery_config.cfg"
PERIPHERY_WEIGHTS_FILE = "yolo/periphery_weights.weights"
PERIPHERY_CLASSES_FILE = "yolo/periphery_classes.names"
PERIPHERY_DATA_FILE = "yolo/periphery_data.data"
PERIPHERY_DNN_BACKEND = 5  # cv.dnn: DNN_BACKEND_CUDA = 5; DNN_BACKEND_OPENCV = 3
PERIPHERY_DNN_TARGET = 7  # cv.dnn: DNN_TARGET_CUDA = 6; DNN_TARGET_CUDA_FP16 = 7; DNN_TARGET_CPU = 0
PERIPHERY_WRAPPER = 1  # 1 = darknet, 2 = opencv from darknet


# ======================================================================================================================
# YOLO PRECISE NETWORK SETTINGS
# ======================================================================================================================
PRECISE_CONFIDENCE_THRESHOLD = 0.25  # Confidence threshold
PRECISE_HIER_THRESHOLD = 0.50
PRECISE_NMS_THRESHOLD = 0.4  # Non-maximum suppression threshold
PRECISE_INPUT_SIZE = (608, 608)
PRECISE_CONFIG_FILE = "yolo/precise_config.cfg"
PRECISE_WEIGHTS_FILE = "yolo/precise_weights.weights"
PRECISE_CLASSES_FILE = "yolo/precise_classes.names"
PRECISE_DATA_FILE = "yolo/precise_data.data"
PRECISE_DNN_BACKEND = 5  # cv.dnn: DNN_BACKEND_CUDA = 5; DNN_BACKEND_OPENCV = 3
PRECISE_DNN_TARGET = 6  # cv.dnn: DNN_TARGET_CUDA = 6; DNN_TARGET_CUDA_FP16 = 7; DNN_TARGET_CPU = 0
PRECISE_WRAPPER = 1  # 1 = darknet, 2 = opencv from darknet


# ======================================================================================================================
# CAMERA SETTINGS
# ======================================================================================================================
CAMERA_W = 3280
CAMERA_H = 2464
APPLY_IMAGE_CROPPING = True
CROP_W_FROM = 400
CROP_W_TO = 2700
CROP_H_FROM = 200
CROP_H_TO = 2200
CAMERA_FRAMERATE = 10
CAMERA_FLIP_METHOD = 0
SCENE_CENTER_X = 1181
SCENE_CENTER_Y = 927
ONE_MM_IN_PX = 6
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
BUFF_CLEANING_DELAY = 1.5  # seconds of waiting before frame reading; should be positive or zero; set to 0 if thread cleaning is used
VIEW_ZONE_POLY_POINTS = [[387, 618], [439, 510], [556, 433], [670, 375], [808, 319], [982, 285], [1143, 279], [1293, 294], [1501, 339], [1635, 395], [1766, 473], [1816, 550], [1867, 637], [1881, 675], [1919, 795], [1942, 926], [1959, 1066], [1964, 1217], [1957, 1321], [1949, 1393], [1874, 1425], [1802, 1457], [1692, 1498], [1555, 1537], [1410, 1567], [1219, 1589], [1081, 1590], [944, 1590], [804, 1575], [679, 1552], [569, 1525], [423, 1475], [330, 1431], [277, 1399], [273, 1289], [279, 1131], [297, 976], [343, 780]]
WORKING_ZONE_POLY_POINTS = [[1929, 934], [1930, 883], [1929, 849], [1923, 765], [1906, 657], [1880, 551], [1859, 494], [1837, 440], [1816, 386], [1775, 363], [1677, 318], [1551, 272], [1484, 254], [1382, 234], [1315, 225], [1253, 217], [1197, 215], [1116, 215], [1018, 220], [914, 234], [820, 253], [738, 277], [671, 300], [593, 332], [505, 372], [476, 451], [452, 521], [425, 600], [416, 625], [399, 709], [397, 725], [393, 796], [391, 870], [391, 928], [707, 928], [1078, 927], [1481, 928]]
IMAGE_CONTROL_POINTS_MAP = [[1181, 927, 0, 0, 0], [1077, 927, -20, 0, 1], [974, 926, -40, 0, 2], [876, 927, -60, 0, 3], [787, 926, -80, 0, 4], [708, 926, -100, 0, 5], [638, 928, -120, 0, 6], [577, 928, -140, 0, 7], [522, 926, -160, 0, 8], [474, 927, -180, 0, 9], [436, 930, -200, 0, 10], [405, 931, -220, 0, 11], [1180, 822, 0, 20, 12], [1078, 823, -20, 20, 13], [975, 825, -40, 20, 14], [879, 827, -60, 20, 15], [791, 829, -80, 20, 16], [712, 835, -100, 20, 17], [640, 840, -120, 20, 18], [578, 843, -140, 20, 19], [526, 847, -160, 20, 20], [479, 851, -180, 20, 21], [437, 855, -200, 20, 22], [404, 858, -220, 20, 23], [1179, 722, 0, 40, 24], [1079, 724, -20, 40, 25], [978, 726, -40, 40, 26], [883, 731, -60, 40, 27], [795, 737, -80, 40, 28], [715, 744, -100, 40, 29], [645, 753, -120, 40, 30], [584, 761, -140, 40, 31], [528, 769, -160, 40, 32], [482, 776, -180, 40, 33], [439, 786, -200, 40, 34], [405, 793, -220, 40, 35], [1179, 628, 0, 60, 36], [1083, 628, -20, 60, 37], [983, 633, -40, 60, 38], [888, 638, -60, 60, 39], [802, 647, -80, 60, 40], [722, 658, -100, 60, 41], [653, 668, -120, 60, 42], [589, 679, -140, 60, 43], [533, 690, -160, 60, 44], [488, 700, -180, 60, 45], [439, 716, -200, 60, 46], [407, 726, -220, 60, 47], [1179, 541, 0, 80, 48], [1084, 543, -20, 80, 49], [988, 548, -40, 80, 50], [896, 554, -60, 80, 51], [811, 567, -80, 80, 52], [732, 576, -100, 80, 53], [663, 589, -120, 80, 54], [600, 602, -140, 80, 55], [547, 617, -160, 80, 56], [497, 629, -180, 80, 57], [460, 638, -200, 80, 58], [426, 647, -220, 80, 59], [1179, 463, 0, 100, 60], [1088, 464, -20, 100, 61], [995, 471, -40, 100, 62], [907, 478, -60, 100, 63], [822, 487, -80, 100, 64], [747, 503, -100, 100, 65], [679, 517, -120, 100, 66], [615, 535, -140, 100, 67], [562, 551, -160, 100, 68], [512, 570, -180, 100, 69], [474, 582, -200, 100, 70], [442, 593, -220, 100, 71], [1175, 398, 0, 120, 72], [1090, 400, -20, 120, 73], [1001, 404, -40, 120, 74], [917, 411, -60, 120, 75], [833, 422, -80, 120, 76], [762, 438, -100, 120, 77], [697, 452, -120, 120, 78], [637, 468, -140, 120, 79], [580, 486, -160, 120, 80], [529, 501, -180, 120, 81], [490, 515, -200, 120, 82], [461, 528, -220, 120, 83], [1173, 339, 0, 140, 84], [1091, 339, -20, 140, 85], [1008, 344, -40, 140, 86], [928, 350, -60, 140, 87], [845, 365, -80, 140, 88], [777, 377, -100, 140, 89], [714, 396, -120, 140, 90], [654, 413, -140, 140, 91], [598, 433, -160, 140, 92], [547, 449, -180, 140, 93], [507, 465, -200, 140, 94], [478, 478, -220, 140, 95], [1173, 291, 0, 160, 96], [1093, 290, -20, 160, 97], [1013, 294, -40, 160, 98], [937, 301, -60, 160, 99], [855, 314, -80, 160, 100], [792, 325, -100, 160, 101], [729, 345, -120, 160, 102], [676, 361, -140, 160, 103], [620, 382, -160, 160, 104], [563, 401, -180, 160, 105], [518, 417, -200, 160, 106], [494, 427, -220, 160, 107], [1173, 249, 0, 180, 108], [1095, 251, -20, 180, 109], [1018, 256, -40, 180, 110], [942, 264, -60, 180, 111], [860, 278, -80, 180, 112], [800, 288, -100, 180, 113], [744, 305, -120, 180, 114], [690, 321, -140, 180, 115], [630, 341, -160, 180, 116], [572, 366, -180, 180, 117], [529, 381, -200, 180, 118], [506, 390, -220, 180, 119], [1172, 225, 0, 200, 120], [1096, 226, -20, 200, 121], [1021, 228, -40, 200, 122], [947, 238, -60, 200, 123], [864, 253, -80, 200, 124], [804, 265, -100, 200, 125], [751, 281, -120, 200, 126], [697, 297, -140, 200, 127], [640, 318, -160, 200, 128], [580, 343, -180, 200, 129], [533, 365, -200, 200, 130], [511, 376, -220, 200, 131], [1286, 927, 20, 0, 132], [1385, 925, 40, 0, 133], [1480, 930, 60, 0, 134], [1562, 929, 80, 0, 135], [1640, 933, 100, 0, 136], [1705, 931, 120, 0, 137], [1763, 933, 140, 0, 138], [1812, 936, 160, 0, 139], [1846, 937, 180, 0, 140], [1883, 936, 200, 0, 141], [1909, 935, 220, 0, 142], [1286, 826, 20, 20, 143], [1382, 828, 40, 20, 144], [1476, 831, 60, 20, 145], [1564, 834, 80, 20, 146], [1636, 839, 100, 20, 147], [1701, 845, 120, 20, 148], [1760, 850, 140, 20, 149], [1809, 855, 160, 20, 150], [1842, 859, 180, 20, 151], [1879, 863, 200, 20, 152], [1906, 866, 220, 20, 153], [1284, 725, 20, 40, 154], [1380, 729, 40, 40, 155], [1473, 735, 60, 40, 156], [1555, 743, 80, 40, 157], [1632, 751, 100, 40, 158], [1696, 759, 120, 40, 159], [1752, 769, 140, 40, 160], [1805, 779, 160, 40, 161], [1836, 785, 180, 40, 162], [1873, 794, 200, 40, 163], [1905, 800, 220, 40, 164], [1281, 628, 20, 60, 165], [1374, 636, 40, 60, 166], [1466, 645, 60, 60, 167], [1549, 655, 80, 60, 168], [1620, 665, 100, 60, 169], [1686, 677, 120, 60, 170], [1745, 690, 140, 60, 171], [1795, 703, 160, 60, 172], [1831, 714, 180, 60, 173], [1865, 723, 200, 60, 174], [1894, 728, 220, 60, 175], [1277, 545, 20, 80, 176], [1368, 552, 40, 80, 177], [1458, 561, 60, 80, 178], [1537, 574, 80, 80, 179], [1609, 583, 100, 80, 180], [1674, 597, 120, 80, 181], [1736, 614, 140, 80, 182], [1785, 626, 160, 80, 183], [1824, 640, 180, 80, 184], [1854, 650, 200, 80, 185], [1885, 662, 220, 80, 186], [1274, 466, 20, 100, 187], [1360, 472, 40, 100, 188], [1448, 485, 60, 100, 189], [1522, 496, 80, 100, 190], [1594, 512, 100, 100, 191], [1659, 526, 120, 100, 192], [1721, 546, 140, 100, 193], [1774, 563, 160, 100, 194], [1813, 576, 180, 100, 195], [1846, 590, 200, 100, 196], [1871, 600, 220, 100, 197], [1267, 399, 20, 120, 198], [1353, 406, 40, 120, 199], [1433, 416, 60, 120, 200], [1508, 432, 80, 120, 201], [1573, 442, 100, 120, 202], [1639, 458, 120, 120, 203], [1704, 477, 140, 120, 204], [1761, 495, 160, 120, 205], [1801, 508, 180, 120, 206], [1837, 522, 200, 120, 207], [1855, 527, 220, 120, 208], [1262, 341, 20, 140, 209], [1344, 351, 40, 140, 210], [1427, 363, 60, 140, 211], [1496, 377, 80, 140, 212], [1557, 390, 100, 140, 213], [1619, 404, 120, 140, 214], [1688, 419, 140, 140, 215], [1747, 435, 160, 140, 216], [1789, 447, 180, 140, 217], [1820, 460, 200, 140, 218], [1838, 466, 220, 140, 219], [1258, 292, 20, 160, 220], [1335, 297, 40, 160, 221], [1413, 310, 60, 160, 222], [1481, 323, 80, 160, 223], [1546, 338, 100, 160, 224], [1604, 358, 120, 160, 225], [1674, 372, 140, 160, 226], [1734, 388, 160, 160, 227], [1773, 402, 180, 160, 228], [1803, 415, 200, 160, 229], [1817, 418, 220, 160, 230], [1255, 252, 20, 180, 231], [1331, 259, 40, 180, 232], [1401, 267, 60, 180, 233], [1471, 279, 80, 180, 234], [1540, 298, 100, 180, 235], [1591, 312, 120, 180, 236], [1658, 332, 140, 180, 237], [1728, 362, 160, 180, 238], [1766, 376, 180, 180, 239], [1794, 390, 200, 180, 240], [1809, 395, 220, 180, 241], [1251, 226, 20, 200, 242], [1326, 233, 40, 200, 243], [1396, 244, 60, 200, 244], [1464, 257, 80, 200, 245], [1534, 278, 100, 200, 246], [1586, 291, 120, 200, 247], [1653, 315, 140, 200, 248], [1721, 346, 160, 200, 249], [1764, 363, 180, 200, 250], [1791, 377, 200, 200, 251], [1806, 385, 220, 200, 252]]
UNDISTORTED_ZONE_RADIUS = 240


# ======================================================================================================================
# APP SETTINGS
# ======================================================================================================================
SAVE_DEBUG_IMAGES = True
DEBUG_IMAGES_PATH = "debug_images/"
RECEIVE_FIELD_FROM_RTK = True
SLOW_MODE_MIN_TIME = 3  # seconds
EXTRACTION_TUNING_MAX_COUNT = 3
DELAY_BEFORE_2ND_SCAN = 0.3  # delay in seconds after robot stop and before second scan (M=1)
DB_NAME = "dbname"
DB_USER = "username"
DB_HOST = "x.x.x.x"
DB_PWD = "password"
ROBOT_SN = "SNXXX"


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
