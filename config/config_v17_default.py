"""Configuration file."""


CONFIG_VERSION = "0.7.0" 


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
KP = {0.175: 0.2, 0.7: 0.11000000000000001, -0.175:0.10} 
KI = {0.175: 0.0092, 0.7: 0.008372000000000001, -0.175:0.0025} 

MANEUVERS_FREQUENCY = 1  # seconds
# max distance in mm-s of robot's deviation from planned moving vector
# if dev. is bigger than this - robot will turn to it's planned moving vector
COURSE_SIDE_DEVIATION_MAX = 50  # max allowed robot's deviation from course im mm-s (threshold)
# distance to stop in mm-s between robot and path ending point
# (its a good idea to keep this value greater than allowed course deviation)
WINDOW = float("inf")   # anyway, the integral is used for only some hundreds of time
# AB moving vector used as A----A1--B, where A1 is point when robot starts moving to next point.
# this determines A1-B distance
MANEUVER_START_DISTANCE = {False:4000, True:4000} 
USE_SPEED_LIMIT = True  # when distance to target point is less than specified in the config
DECREASE_SPEED_TRESHOLD = 5000  # millimeters
SUM_ANGLES_HISTORY_MAX = 1000  # max value and min -value of sum(angles_history), should be positive here, in config
# distance between sides of spiral robot movements, expected to be equal to working area width, may be any positive val
SPIRAL_SIDES_INTERVAL = {False: 333, True: 3000} 
FIELD_REDUCE_SIZE = 200  # cut field's each side for this value, mms
PREV_CUR_POINT_MIN_DIST = 100  # pass by cur points dist between them and prev point is lesser than this, mms
FILTER_MAX_DIST = 5000  # maximum allowable distance between consecutive points (in millimeters)
FILTER_MIN_DIST = 600  # minimum allowable distance between consecutive points (in millimeters)

#Field creation with main
USE_EMERGENCY_FIELD_GENERATION = False  # allows to generate field by moving forward for a given duration
EMERGENCY_FIELD_SIZE = 45000  # mms; side of the area that will be created if emergency field creation is enabled
EMERGENCY_MOVING_TIME = 10  # seconds of moving forward for vector getting

#Continue mode
CONTINUE_PREVIOUS_PATH = False
PREVIOUS_PATH_POINTS_FILE = "path_points.dat"
PREVIOUS_PATH_INDEX_FILE = "path_index.txt"

#Cyril covid
FAR_TARGET_THRESHOLD = {0.175: 25000, 0.7: 25000, -0.175: 25000} # the output of the KP KI is boost if the target is far than this threshold
FAR_TARGET_GAIN = {0.175: 1.15, 0.7: 1.15, -0.175: 1.15} # the output of the KP KI is boost by this muliplier if the target is far than the threshold
CLOSE_TARGET_THRESHOLD = {0.175: 5000, 0.7: 5000, -0.175: 5000} # the output of the KP KI is managed if the target is less than this threshold
SMALL_RAW_ANGLE_SQUARE_THRESHOLD = {0.175: 100, 0.7: 100, -0.175: 100} # the output of the KP KI is calm by a muliplier if the raw angle is less than the threshold
SMALL_RAW_ANGLE_SQUARE_GAIN = {0.175: 0.9, 0.7: 1, -0.175: 0.9} # the output of the KP KI is calm by this muliplier if the raw angle is less than the threshold
BIG_RAW_ANGLE_SQUARE_THRESHOLD = {0.175: 625, 0.7: 625, -0.175: 625} # the output of the KP KI is boost by a muliplier if the raw angle is more than the threshold
BIG_RAW_ANGLE_SQUARE_GAIN = {0.175: 1.35, 0.7: 1.35, -0.175: 1.35} # the output of the KP KI is boost by this muliplier if the raw angle is more than the threshold
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

WHEELS_STRAIGHT_CHANGE_DIRECTION_OF_TRAVEL = True

# ======================================================================================================================
# PATH SETTINGS
# ======================================================================================================================

#Only one of the following three parameters must be true
TRADITIONAL_PATH = True #Snail path
BEZIER_CORNER_PATH = False #Snail path and use of bezier curve for turns
FORWARD_BACKWARD_PATH = False #Path where the robot goes straight in extraction then reverses without extraction ....

#This params work only if TRADITIONAL_PATH or BEZIER_CORNER_PATH are true. 
ADD_FORWARD_BACKWARD_TO_END_PATH = True #Adds the path FORWARD_BACKWARD to complete the missing center.

#This params work only if BEZIER_CORNER_PATH are true.
NUMBER_OF_BEZIER_POINT = 10 #Allows to determine the number of points put in the bezier turn.

TWO_POINTS_FOR_CREATE_FIELD = False

# ======================================================================================================================
# NAVIGATION TEST MODE SETTINGS
# ======================================================================================================================

NAVIGATION_TEST_MODE = False
QUEUE_NAVIGATION_TEST_MODE = "/queue_test_nav"
POINT_A = [[46.1578135, -1.1341983], -0.175]
POINT_B = [[46.1574592, -1.1350758], -0.175]

# ======================================================================================================================
# EXTRACTION SETTINGS
# ======================================================================================================================
EXTRACTION_DEFAULT_METHOD = "single_center_drop"  # or "five_drops_near_center"
ADDITIONAL_EXTRACTIONS_DISTANCE_X = 20 # mm
ADDITIONAL_EXTRACTIONS_DISTANCE_Y = 20 # mm
AVOID_CORK_VIEW_OBSCURING = True  # is True: adds offsets to control points to make a plant to be at the top half of the undistorted zone
EXTRACTIONS_FULL_CYCLES = 2  # count of full extraction loops called after periphery NN detection (should be >= 1)
SEEK_DELTA_DISTANCE = 25  # mm; if weed is lost after tuning/getting closer - we do 3 shifts for that value (down, left, right) and trying to find it
MYOPIA_PATCH = True

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
STEP_FORWARD_TIME = 1 # step after extraction loops are done
STEP_FORWARD_RPM = -2000  # # step after extraction loops are done #-2500 à remettre
FAST_TO_SLOW_RPM = 2500
FAST_TO_SLOW_TIME = 5

VESC_RPM_SLOW_SI = 0.175
MULTIPLIER_MM_S_TO_RPM = -14285

# ======================================================================================================================
# GPS SETTINGS
# ======================================================================================================================
GPS_PORT = "/dev/ttyTHS1"
GPS_BAUDRATE = 19200 
GPS_POSITIONS_TO_KEEP = 1000
NO_GPS_TIMEOUT = 15
GPS_CHECK_IN_DEGRADED_MODE = 30 #Check if gps returned every GPS_CHECK_IN_DEGRADED_MODE navigation cycle.

# ======================================================================================================================
# SMOOTHIE SETTINGS
# ======================================================================================================================
SMOOTHIE_HOST = "/dev/ttyACM1" # smoothie's ip address for telnet or port for usb serial connector
SMOOTHIE_BAUDRATE = 115200
SMOOTHIE_BACKEND = 2  # 1 = telnet, 2 = serial

X_MIN = 0
X_MAX = 450 
X_F_MIN = 0
X_F_MAX = 20000
X_COEFFICIENT_TO_MM = 1

Y_MIN = 0
Y_MAX = 220 
Y_F_MIN = 0
Y_F_MAX = 6000
Y_COEFFICIENT_TO_MM = 1

# smoothie movement separation update
ALLOW_SEPARATE_XY_MOVEMENT = True
XY_SEP_MOV_MAX_RATIO_THRESHOLD = 1

Z_MIN = -float("inf")
Z_MAX = float("inf")
Z_F_MIN = 1
Z_F_MAX = 2000 
EXTRACTION_Z = 40 # drill version value
Z_F_EXTRACTION_UP = 1500 
Z_F_EXTRACTION_DOWN = 1850 
Z_COEFFICIENT_TO_MM = 1  # not used yet

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
CORK_CALIBRATION_MIN_TIME = 3600 

# NAVIGATION
A_MIN = -5 
A_MAX = 5 
B_MIN = -float("inf")
B_MAX = float("inf")
C_MIN = -float("inf")
C_MAX = float("inf")

A_F_MIN = 1
A_F_MAX = 4000 #default value 4000
A_COEFFICIENT_TO_MM = 1
A_F_UI = 1000 #default value 1000

B_F_MIN = 1
B_COEFFICIENT_TO_MM = 1
B_F_MAX = 4000

C_F_MIN = 1
C_F_MAX = 1000
C_COEFFICIENT_TO_MM = 1

A_ONE_DEGREE_IN_SMOOTHIE = 2  # A axis
A_DEGREES_PER_SECOND = 5  # A axis
NAV_TURN_WHEELS_CENTER = 0

# ======================================================================================================================
# DETECTION SETTINGS
# ======================================================================================================================

ALLOW_PRECISE_RESCAN = True
ALLOW_PRECISE_SINGLE_SCAN_BEFORE_PDZ = False
EXTRACTION_TUNING_MAX_COUNT = 3 # Number of try to get closer to a plant

# ======================================================================================================================
# YOLO PERIPHERY NETWORK SETTINGS
# ======================================================================================================================
PERIPHERY_CONFIDENCE_THRESHOLD = 0.1 # Confidence threshold
PERIPHERY_HIER_THRESHOLD = 0.5  # works only in darknet wrapper
PERIPHERY_NMS_THRESHOLD = 0.4      # Non-maximum suppression threshold
PERIPHERY_INPUT_SIZE = (416, 416)
PERIPHERY_CONFIG_FILE = "yolo/Y0016_416.cfg"
PERIPHERY_WEIGHTS_FILE = "yolo/Y0016.weights"
PERIPHERY_CLASSES_FILE = "yolo/Y0016.names"
PERIPHERY_DNN_BACKEND = 5  # cv.dnn: DNN_BACKEND_CUDA = 5; DNN_BACKEND_OPENCV = 3
PERIPHERY_DNN_TARGET = 6  # cv.dnn: DNN_TARGET_CUDA = 6; DNN_TARGET_CUDA_FP16 = 7; DNN_TARGET_CPU = 0
PERIPHERY_WRAPPER = 1  # 1 = darknet, 2 = opencv from darknet
PERIPHERY_DATA_FILE = "yolo/Y0016.data" 

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
PRECISE_DATA_FILE = "yolo/Y0016.data" 
PRECISE_DNN_BACKEND = 5  # cv.dnn: DNN_BACKEND_CUDA = 5; DNN_BACKEND_OPENCV = 3
PRECISE_DNN_TARGET = 6  # cv.dnn: DNN_TARGET_CUDA = 6; DNN_TARGET_CUDA_FP16 = 7; DNN_TARGET_CPU = 0
PRECISE_WRAPPER = 1  # 1 = darknet, 2 = opencv from darknet


# ======================================================================================================================
# CAMERA SETTINGS
# ======================================================================================================================
CAMERA_W = 3280
CAMERA_H = 2464
APPLY_IMAGE_CROPPING = True
CROP_W_FROM = 498 
CROP_W_TO = 2498 
CROP_H_FROM = 160 
CROP_H_TO = 1660 
CAMERA_FRAMERATE = 8
CAMERA_FLIP_METHOD = 0
SCENE_CENTER_X = 1000  # 1576 for uncropped
SCENE_CENTER_Y = 980  # 1104 for uncropped
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
#WORKING_ZONE_POLY_POINTS = [[1000, 1145], [230, 1080], [245, 755], [340, 400], [640, 315], [1000, 270], [1360, 315], [1660, 400], [1755, 755], [1770, 1080]]
UNDISTORTED_ZONE_RADIUS = 300
DELAY_BEFORE_2ND_SCAN = 0.3  # delay in seconds after robot stop and before second scan (M=1)
VIEW_ZONE_POLY_POINTS = [[387, 618], [439, 510], [556, 433], [670, 375], [808, 319], [982, 285], [1143, 279], [1293, 294], [1501, 339], [1635, 395], [1766, 473], [1816, 550], [1867, 637], [1881, 675], [1919, 795], [1942, 926], [1959, 1066], [1964, 1217], [1957, 1321], [1949, 1393], [1874, 1425], [1802, 1457], [1692, 1498], [1555, 1537], [1410, 1567], [1219, 1589], [1081, 1590], [944, 1590], [804, 1575], [679, 1552], [569, 1525], [423, 1475], [330, 1431], [277, 1399], [273, 1289], [279, 1131], [297, 976], [343, 780]]
#WORKING_ZONE_POLY_POINTS = [[1000, 1145], [230, 1080], [245, 755], [340, 400], [640, 315], [1000, 270], [1360, 315], [1660, 400], [1755, 755], [1770, 1080]] 
WORKING_ZONE_POLY_POINTS = [[1000, 1050], [30, 1080], [45, 755], [40, 300], [640, 115], [1000, 70], [1360, 115], [1860, 300], [1955, 755], [1970, 1080]]
# ======================================================================================================================
# APP SETTINGS
# ======================================================================================================================
SAVE_DEBUG_IMAGES = False
DEBUG_IMAGES_PATH = "debug_images/"
ALLOW_GATHERING = False

ROBOT_SN = "SN003" 
UI_LANGUAGE = "en"
SLIDER_CREATE_FIELD_MIN = 15
SLIDER_CREATE_FIELD_MAX = 150
SLIDER_CREATE_FIELD_DEFAULT_VALUE = 25
SLIDER_CREATE_FIELD_STEP = 1

FRAME_SHOW = True
SHARED_MEMORY_NAME_DETECTED_FRAME = "/detected_frame"

AUDIT_MODE = False
AUDIT_DIVIDER = 6
AUDIT_OUTPUT_FILE = "audit.txt"

SLOW_FAST_MODE = False
SLOW_MODE_MIN_TIME = 3  # seconds

VERBOSE = False
LOG_ROOT_DIR = "logs/"
STATISTICS_OUTPUT_FILE = "statistics.txt"
DATACOLLECTOR_SAVE_FILE = "datacollection_save.dat"
DATA_GATHERING_DIR = "gathered_data/"
LAST_ANGLE_WHEELS_FILE = "last_angle_wheels.txt"
FILES_TO_KEEP_COUNT = 600

QUEUE_NAME_UI_MAIN = "/queue_ui_main"
QUEUE_NAME_UI_NOTIFICATION = "/queue_ui_notification"

CONTINUOUS_INFORMATION_SENDING = True
ALIVE_SENDING_TIMEOUT = 1

# ======================================================================================================================
# DETECTION MANAGER SETTINGS
# ======================================================================================================================

CAMERA_POSITIONS = [(220, 0)] # smoothie global coordinates to take photos from for forming plants list. Format is (x, y)
PDZ_DISTANCES = [{"top": 1000, "bot": 1000, "left": 1000, "right": 1000}] # precice detection zone sizes. At each camera position scan only plants inside this zone is added to extraction list
#CAMERA_POSITIONS = [(150, 0), (300,0)] # smoothie global coordinates to take photos from for forming plants list. Format is (x, y)
#PDZ_DISTANCES = [{"top": 750, "bot": 750, "left": 1000, "right": 350}, {"top": 750, "bot": 750, "left": 450, "right": 1000}] # precice detection zone sizes. At each camera position scan only plants inside this zone is added to extraction list
# values are px count from scene center to. Format is {"top": int, "bot": int, "left": int, "right": int}


# ======================================================================================================================
# EXTRACTION MANAGER SETTINGS
# ======================================================================================================================

EXTRACTION_PATTERNS_OFFSET_MM = 20
EXTRACTION_MAP_CELL_SIZE_MM = 10
EXTRACTION_TRIES_PER_PLANT = 3 # defines how many times robot will try to extract plant or plants group in undist. zone
# should be >= 1; expected value that equal to extraction strategies count so each of them can be applied

AVOID_CORK_VIEW_OBSCURING_DIST_X = 10  # mm; offset size to "remove" corkscrew tube from camera-plant view
# mm; offset size to "remove" corkscrew tube from camera-plant view, should be negative to move cork down and free view
AVOID_CORK_VIEW_OBSCURING_DIST_Y = -10

ALLOW_DELTA_SEEKING = True  # will try to move over supposed plant position if plant wasn't detected during approaching
# False: seek over coordinates updated by cork obscuring patch; True: seek over original coordinates
DELTA_SEEKING_IGNORE_OBSCURING = False

DEBUG_MATRIX_FILE = False
FILTER_EXTRACTED_PLANTS = True
FILTER_EXT_PLANTS_TRIGGER_DIST = 25  # px; trigger distance for previous option key

# ======================================================================================================================
# PREDICTION SETTINGS
# ======================================================================================================================
ZONE_THRESHOLD_DEGREE = [(436,5),(697,7),(796,17),(849,15),(953,6)]

# ======================================================================================================================
# NTRIP CLIENT SETTINGS
# ======================================================================================================================
NTRIP = False 
NTRIP_RESTART_TIMEOUT = 60
NTRIP_USER = "centipede"
NTRIP_PASSWORD = "centipede"
NTRIP_CASTER = "caster.centipede.fr"
NTRIP_PORT = 2101
NTRIP_MOUNTPOINT = "LIENSS"

NTRIP_OUTPUT_PORT = "/dev/ttyACM1"
NTRIP_OUTPUT_BAUDRATE = 38400
