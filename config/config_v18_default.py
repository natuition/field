"""Configuration file."""


CONFIG_VERSION = "0.8.0" 


# ======================================================================================================================
# NAVIGATION ROUTING SETTINGS
# ======================================================================================================================
KP = {
    0.175: 0.2,
    -0.175: 0.19,

    0.7: 0.11000000000000001,
    -0.7: 0.0360000000000001,

    0.5: 0.293*0.277,
    -0.5: 0.293*0.14,

    0: 0.2
} # SI_speed: value of KP at this speed (warning - is important for si_speed)

KI = {
    0.175: 0.0092, 
    -0.175: 0.0092, 

    0.7: 0.008372000000000001, 
    -0.7: 0.000000001,
    
    0.5: 0.00125,
    -0.5: 0.00003,

    0: 0.092
} # SI_speed: value of KP at this speed (warning - is important for si_speed)

CENTROID_FACTOR_ORIENTED = 0.585 #multiply factor at the angle of the centroid calculate when the robot is going in the right direction
CENTROID_FACTOR_LOST = 0.2 #multiply factor at the angle of the centroid calculate when the robot isn't going in the right direction

CORNER_THRESHOLD = 1500 #threshold which represents the maximum deviation of the points in the turns for it to be recognized

MANEUVERS_FREQUENCY = 0.25  #seconds
# max distance in mm-s of robot's deviation from planned moving vector
# if dev. is bigger than this - robot will turn to it's planned moving vector
WINDOW = float("inf")   # anyway, the integral is used for only some hundreds of time
# AB moving vector used as A----A1--B, where A1 is point when robot starts moving to next point.
# this determines A1-B distance
MANEUVER_START_DISTANCE = {False:5000, True:4000} # this parameter depends on whether the audit mode is false or true
USE_SPEED_LIMIT = True  # when distance to target point is less than specified in the config
DECREASE_SPEED_TRESHOLD = 5000  # millimeters
SUM_ANGLES_HISTORY_MAX = 1000  # max value and min -value of sum(angles_history), should be positive here, in config
SPIRAL_SIDES_INTERVAL = {False: 450, True: 3000} # distance between sides of spiral robot movements, 
# expected to be equal to working area width, may be any positive val
# this parameter depends on whether the audit mode is false or true
FIELD_REDUCE_SIZE = 200  # cut field's each side for this value, mms
PREV_CUR_POINT_MIN_DIST = 10  # pass by cur points dist between them and prev point is lesser than this, mms

#Field creation with main
USE_EMERGENCY_FIELD_GENERATION = False  # allows to generate field by moving forward for a given duration
EMERGENCY_FIELD_SIZE = 45000  # mms; side of the area that will be created if emergency field creation is enabled
EMERGENCY_MOVING_TIME = 10  # seconds of moving forward for vector getting

#Continue mode
CONTINUE_PREVIOUS_PATH = False
PREVIOUS_PATH_POINTS_FILE = "path_points.dat"
PREVIOUS_PATH_INDEX_FILE = "path_index.txt"

#Cyril covid
ORIGIN_AVERAGE_SAMPLES = 8

WHEELS_STRAIGHT_CHANGE_DIRECTION_OF_TRAVEL = True

FUTURE_NUMBER_OF_POINTS = 3 #number of points that is given to the move to point function in addition to the one that is aimed


# ======================================================================================================================
# ROBOT PATH CREATION SETTINGS
# ======================================================================================================================
#Only one of the following three parameters must be true
TRADITIONAL_PATH = False #Snail path
BEZIER_PATH = True #Snail path and use of bezier curve for turns
FORWARD_BACKWARD_PATH = False #Path where the robot goes straight in extraction then reverses without extraction ....

#This params work only if TRADITIONAL_PATH or BEZIER_PATH are true. 
ADD_FORWARD_BACKWARD_TO_END_PATH = True #Adds the path FORWARD_BACKWARD to complete the missing center.
#This params work only if BEZIER_PATH are true. 
ADD_CORNER_TO_BEZIER_PATH = False #Add management of corner for bezier curve

#This params work only if BEZIER_CORNER_PATH are true.
NUMBER_OF_BEZIER_POINT = 11 #Allows to determine the number of points put in the bezier turn.

TWO_POINTS_FOR_CREATE_FIELD = False


# ======================================================================================================================
# NAVIGATION TEST MODE SETTINGS
# =====================================================================================================================
NAVIGATION_TEST_MODE = False # mode allowing the robot to do A->B, B->A
#The robot will aim for the furthest point, 
#when it reaches this point it will wait for a press on enter to go to the furthest point from it.
DISPLAY_INSTRUCTION_PATH = True #Allows to display the robot guide points on the ui.
DELTA_DISPLAY_INSTRUCTION_PATH = 15 #Number of guide points display on the ui.
POINT_A = [[46.1579425, -1.1344245], -0.5] #Point coordinate for test navigation mode, [[lat,long],speed]
# the speed represents the speed the robot will apply to reach this point.
POINT_B = [[46.1577957, -1.1347992], 0.5] #Point coordinate for test navigation mode, [[lat,long],speed]
# the speed represents the speed the robot will apply to reach this point.


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
STATISTICS_OUTPUT_FILE = "statistics.txt"
DATACOLLECTOR_SAVE_FILE = "datacollection_save.dat"
LAST_ANGLE_WHEELS_FILE = "last_angle_wheels.txt"
LEARN_GO_STRAIGHT_FILE = "learn_go_straight.txt"

LOG_ROOT_DIR = "logs/"
DATA_GATHERING_DIR = "gathered_data/"


# ======================================================================================================================
# VESC SETTINGS
# ======================================================================================================================
VESC_PORT = "/dev/ttyACM0" #Deprecated for most scripts that use a function to find it dynamically.
VESC_BAUDRATE = 115200

VESC_RPM_SLOW = -2500
VESC_MOVING_TIME = float("inf")
VESC_ALIVE_FREQ = 0.5  # freq of sending "keep working" signal to engines when moving
VESC_CHECK_FREQ = 0.001  # freq of checking need to stop
FAST_TO_SLOW_TIME = 5

# step after extraction loops are done
STEP_FORWARD_TIME = 1 
SI_SPEED_STEP_FORWARD = 0.5

SI_SPEED_UI = 1 #0.8 for 12v
SI_SPEED_FWD = 0.5
SI_SPEED_REV = -0.5
SI_SPEED_FAST = 0.7
MULTIPLIER_SI_SPEED_TO_RPM = -14285 #multiplier to go from speed to rpm vesc


# ======================================================================================================================
# GPS SETTINGS
# ======================================================================================================================
GPS_PORT = "/dev/ttyTHS1"
GPS_BAUDRATE = 115200 
GPS_POSITIONS_TO_KEEP = 1000
NO_GPS_TIMEOUT = 15
GPS_CHECK_IN_DEGRADED_MODE = 30 #Check if gps returned every GPS_CHECK_IN_DEGRADED_MODE navigation cycle.

SER2NET_CONNECT_GPS_PORT = False


# ======================================================================================================================
# SMOOTHIE SETTINGS
# ======================================================================================================================
SMOOTHIE_HOST = "/dev/ttyACM1" # smoothie's ip address for telnet or port for usb serial connector
# Deprecated for most scripts that use a function to find it dynamically.
SMOOTHIE_BAUDRATE = 115200
SMOOTHIE_BACKEND = 2  # 1 = telnet, 2 = serial

X_MIN = 0
X_MAX = 450 
X_F_MIN = 0
X_F_MAX = 20000
X_COEFFICIENT_TO_MM = 1

Y_MIN = 0
Y_MAX = 210 
Y_F_MIN = 0
Y_F_MAX = 20000
Y_COEFFICIENT_TO_MM = 1

# smoothie movement separation update
ALLOW_SEPARATE_XY_MOVEMENT = False
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
A_MIN = -10 
A_MAX = 10 
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
PERIPHERY_CONFIDENCE_THRESHOLD = 0.99
PERIPHERY_HIER_THRESHOLD = 0.5
PERIPHERY_NMS_THRESHOLD = 0.4
PERIPHERY_INPUT_SIZE = (416, 416)
PERIPHERY_CONFIG_FILE = "yolo/Y0016_416.cfg"
PERIPHERY_WEIGHTS_FILE = "yolo/Y0016.weights"
PERIPHERY_CLASSES_FILE = "yolo/Y0016.names"
PERIPHERY_DNN_BACKEND = 5
PERIPHERY_DNN_TARGET = 6
PERIPHERY_WRAPPER = 1
PERIPHERY_DATA_FILE = "yolo/Y0016.data"

PERIPHERY_MODEL_PATH = "yolo/Y0016.trt"


# ======================================================================================================================
# YOLO PRECISE NETWORK SETTINGS
# ======================================================================================================================
PRECISE_CONFIDENCE_THRESHOLD = 0.99
PRECISE_HIER_THRESHOLD = 0.5
PRECISE_NMS_THRESHOLD = 0.4
PRECISE_INPUT_SIZE = (832, 832)
PRECISE_CONFIG_FILE = "yolo/Y0016_832.cfg"
PRECISE_WEIGHTS_FILE = "yolo/Y0016.weights"
PRECISE_CLASSES_FILE = "yolo/Y0016.names"
PRECISE_DATA_FILE = "yolo/Y0016.data"
PRECISE_DNN_BACKEND = 5
PRECISE_DNN_TARGET = 6
PRECISE_WRAPPER = 1


# ======================================================================================================================
# CAMERA SETTINGS
# ======================================================================================================================
CAMERA_W = 3264
CAMERA_H = 1848
APPLY_IMAGE_CROPPING = True
CROP_W_FROM = 566
CROP_W_TO = 2566
CROP_H_FROM = 0
CROP_H_TO = 1500
CAMERA_FRAMERATE = 16
CAMERA_FLIP_METHOD = 0
SCENE_CENTER_X = 1000
SCENE_CENTER_Y = 980
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
UNDISTORTED_ZONE_RADIUS = 300
DELAY_BEFORE_2ND_SCAN = 0.3  # delay in seconds after robot stop and before second scan (M=1)
WORKING_ZONE_POLY_POINTS = [[1000, 1050], [30, 1080], [45, 755], [40, 300], [640, 115], [1000, 70], [1360, 115], [1860, 300], [1955, 755], [1970, 1080]]

CORK_TO_CAMERA_DISTANCE_X = 0  # # distance between camera and cork on the robot, X axis, relative, mm
CORK_TO_CAMERA_DISTANCE_Y = 30  # distance between camera and cork on the robot, Y axis, relative, mm


# ======================================================================================================================
# APP SETTINGS
# ======================================================================================================================
SAVE_DEBUG_IMAGES = False
DEBUG_IMAGES_PATH = "debug_images/"
ALLOW_GATHERING = False

ROBOT_SN = "SNXXX" 
UI_LANGUAGE = "en"
SLIDER_CREATE_FIELD_MIN = 15 #minimum of the slider allowing to configure the second segment of the terrain on the ui.
SLIDER_CREATE_FIELD_MAX = 150 #maximum of the slider allowing to configure the second segment of the terrain on the ui
SLIDER_CREATE_FIELD_DEFAULT_VALUE = 25 #default value of the slider allowing to configure the second segment of the terrain on the ui
SLIDER_CREATE_FIELD_STEP = 1 #step of the slider allowing to configure the second segment of the terrain on the ui

FRAME_SHOW = False
SHARED_MEMORY_NAME_DETECTED_FRAME = "/detected_frame"

AUDIT_MODE = False
AUDIT_DIVIDER = 6
AUDIT_OUTPUT_FILE = "audit.txt"

SLOW_FAST_MODE = False
SLOW_MODE_MIN_TIME = 3  # seconds
SLOW_FAST_MODE_HEAD_FACTOR = 1

VERBOSE = False
VERBOSE_EXTRACT = True
FILES_TO_KEEP_COUNT = 600

QUEUE_NAME_UI_MAIN = "/queue_ui_main"
QUEUE_NAME_UI_NOTIFICATION = "/queue_ui_notification"

CONTINUOUS_INFORMATION_SENDING = True
ALIVE_SENDING_TIMEOUT = 1

LEARN_GO_STRAIGHT = True
LEARN_GO_STRAIGHT_UI = False
MIN_PERPENDICULAR_GO_STRAIGHT = 100 # in mm
VALUES_LEARN_GO_STRAIGHT = 40

GPS_POINT_WAIT_TIME_MAX = 1000

ANTI_THEFT_ZONE_RADIUS = 5000


# ======================================================================================================================
# DETECTION MANAGER SETTINGS
# ======================================================================================================================
CAMERA_POSITIONS = [(220, 0)] # smoothie global coordinates to take photos from for forming plants list. Format is (x, y)
PDZ_DISTANCES = [{"top": 1000, "bot": 1000, "left": 1000, "right": 1000}] # precice detection zone sizes. At each camera position scan only plants inside this zone is added to extraction list
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

MAKE_MANEUVER_AFTER_FIELD_CREATE = True
MANEUVER_TIME_BACKWARD = 3 #6 for 12V, 3 for 24V
MANEUVER_TIME_FORWARD = 4 #7 for 12V, 4 for 24V

# ======================================================================================================================
# PREDICTION SETTINGS
# ======================================================================================================================
ZONE_THRESHOLD_DEGREE = [(436,5),(697,7),(796,17),(849,15),(953,6)]


# ======================================================================================================================
# NTRIP CLIENT SETTINGS
# ======================================================================================================================
NTRIP = True 
FIND_MOUNTPOINT = True #Allows you to find the station closest to MAX_DISTANCE_MOUNTPOINT maximum.
SEND_LOCATION_TO_NTRIP = False #Allows us to send our coordinates to the rtk caster

NTRIP_USER = "centipede"
NTRIP_PASSWORD = "centipede"
NTRIP_CASTER = "caster.centipede.fr"
NTRIP_PORT = 2101
NTRIP_MOUNTPOINT = "LIENSS" #Mountpoint of the caster, if SEND_LOCATION_TO_NTRIP=true the program will take the closest.

NTRIP_OUTPUT_PORT = "/dev/ttyACM1"
NTRIP_OUTPUT_BAUDRATE = 115200

NTRIP_RESTART_TIMEOUT = 60
MAX_DISTANCE_MOUNTPOINT = 1000 #Allows you to find the station closest to MAX_DISTANCE_MOUNTPOINT maximum if FIND_MOUNTPOINT=True.
RTK_ID_SEND = [1077,1087,1127,1230,1005] #Id of the rtk frames that will be sent to the gps

GPS_QUALITY_IGNORE = False #If this is activated, stops the robot when it no longer has quality 4. 
# It restarts the ntrip service and waits to find quality 4