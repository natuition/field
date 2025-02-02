"""Configuration file."""


CONFIG_VERSION = "2.1.1"


# ======================================================================================================================
# CONTENTS:

# NAVIGATION ROUTING SETTINGS
# ROBOT PATH (TRAJECTORY PLANNER) CREATION SETTINGS
# EXTRACTION SETTINGS
# VESC SETTINGS
# GPS SETTINGS
# SMOOTHIE SETTINGS
# DETECTION SETTINGS
# APP SETTINGS
# WEB INTERFACE SETTINGS
# EXTRACTION MANAGER SETTINGS
# NTRIP CLIENT SETTINGS
# SEEDER SETTINGS
# MILLING SETTINGS
# YOLO PERIPHERY NETWORK SETTINGS
# YOLO PRECISE NETWORK SETTINGS
# CAMERA SETTINGS
# PATHS SETTINGS
# PREDICTION SETTINGS
# NAVIGATION TEST MODE SETTINGS
# UNSORTED KEYS
# ======================================================================================================================


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
MANEUVER_START_DISTANCE = 5000  # this parameter depends on whether the audit mode is false or true
USE_SPEED_LIMIT = True  # when distance to target point is less than specified in the config
DECREASE_SPEED_TRESHOLD = 5000  # millimeters
SUM_ANGLES_HISTORY_MAX = 1000  # max value and min -value of sum(angles_history), should be positive here, in config
SPIRAL_SIDES_INTERVAL = 450  # distance between sides of spiral robot movements,
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

#FORCE STEP
STEP_FORWARD_TIME = 0.8 
SI_SPEED_STEP_FORWARD = 0.175

#SPEEDS
SI_SPEED_UI = 1 #0.8 for 12v
SI_SPEED_FWD = 0.175 
SI_SPEED_REV = -0.5
SI_SPEED_FAST = 0.5 
MULTIPLIER_SI_SPEED_TO_RPM = -14285 #multiplier to go from speed to rpm vesc

SLOW_FAST_MODE = True
SLOW_MODE_MIN_TIME = 10  # seconds
SLOW_FAST_MODE_HEAD_FACTOR = 0.5

# protection to prevent robot leaving a field it works on
# True: stop robot and set state to out if service if robot has left the field; False: disable leaving the field control
ALLOW_FIELD_LEAVING_PROTECTION = True
# mms; robot will stop if ALLOW_FIELD_LEAVING_PROTECTION=True and PERPENDICULAR from a field's line is bigger
# than this value (all ABCDA field lines are checked) and it's on left side of the line
LEAVING_PROTECTION_DISTANCE_MAX = 1000


# ======================================================================================================================
# ROBOT PATH (TRAJECTORY PLANNER) CREATION SETTINGS
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

MAKE_MANEUVER_AFTER_FIELD_CREATE = True
MANEUVER_TIME_BACKWARD = 3 #6 for 12V, 3 for 24V
MANEUVER_TIME_FORWARD = 4 #7 for 12V, 4 for 24V

# True: robot will try to approach his continue previous job target point smoothly by visiting some previous points
# (will look for a point with a good angle between point and current position); False: go straight to target point
USE_SMOOTH_APPROACHING_TO_FIELD = True
# smooth approach to target point when continuing previous job will be aborted if robot wants to visit more
# previous points than this value (default value for bezier path is NUMBER_OF_BEZIER_POINT * 4 + 10;
# default value for bezier with filled corners *not implemented yet*)
SMOOTH_APPROACHING_MAX_POINTS = NUMBER_OF_BEZIER_POINT * 4 + 10


# ======================================================================================================================
# EXTRACTION SETTINGS
# ======================================================================================================================
EXTRACTION_CONTROLLER = 1  # 1 is smoothie, 2 is vesc
# === If EXTRACTION_CONTROLLER = 2 ===
EXTRACTION_MODE = 1  # 0 is reserved, 1 is extractions, 2 is milling
EXTRACTION_CORK_DOWN_RPM = 20000
EXTRACTION_CORK_UP_RPM = -5500
EXTRACTION_CORK_DOWN_TIME = 1  # seconds; how much time cork should move down during plant extraction
EXTRACTION_CORK_STOPPER_REACHING_MAX_TIME = 3  # seconds
# this will have effect only if Z axis is controlled by vesc (this config EXTRACTION_CONTROLLER key)
# True: will try to drop and pick cork again if stopper hit was registered earlier than Z_AXIS_PICKUP_MIN_TIME;
# False: cork pickup will work as usual, relying on GPIO stopper and EXTRACTION_CORK_STOPPER_REACHING_MAX_TIME timeout;
ALLOW_VESC_CORK_PICKUP_MIN_TIME = True
# this will have effect only if ALLOW_VESC_CORK_PICKUP_MIN_TIME is set to True;
# seconds; if cork stopper hits earlier than this time - robot will try to drop and pick cork again
VESC_CORK_PICKUP_MIN_TIME = 1
# int; ALWAYS MUST BE >= 1 FOR CORK PROPER WORKING REGARDLESS OF OTHER SETTINGS INCLUDING config.EXTRACTION_CONTROLLER!
# values bigger than 1 will have effect only if ALLOW_VESC_CORK_PICKUP_MIN_TIME is set to True;
# defines how many times robot will try to drop and pick cork if there are troubles with stopper before error is raised
VESC_CORK_PICKUP_MAX_TRIES = 3
# === end of EXTRACTION_CONTROLLER = 2 ===

AVOID_CORK_VIEW_OBSCURING = True  # is True: adds offsets to control points to make a plant to be at the top half of the undistorted zone

EXTRACTIONS_FULL_CYCLES = 1  # count of full extraction loops called after periphery NN detection (should be >= 1)
EXTRACTION_TUNING_MAX_COUNT = 2 # Number of try to get closer to a plant

SEEK_DELTA_DISTANCE = 25  # mm; if weed is lost after tuning/getting closer - we do 3 shifts for that value (down, left, right) and trying to find it
MYOPIA_PATCH = True

# set to True to disable weeds extractions during movement to first point when continuing previous path (continue mode)
FIRST_POINT_NO_EXTRACTIONS = False

# True: set on pause extractions and wait for manual continue permission; False: usual extraction
SET_EXTRACTIONS_ON_DEBUG_PAUSE = False


# ======================================================================================================================
# VESC SETTINGS
# ======================================================================================================================
VESC_PORT = "/dev/ttyACM2" #Deprecated for most scripts that use a function to find it dynamically.
VESC_BAUDRATE = 115200

VESC_RPM_SLOW = -2500
VESC_MOVING_TIME = float("inf")
VESC_ALIVE_FREQ = 4  # freq in herz of sending "keep working" signal to engines when moving
VESC_CHECK_FREQ = 1000  # freq in herz of checking need to stop
FAST_TO_SLOW_TIME = 5
VESC_STOPPER_CHECK_FREQ = 1000 # freq in herz
VESC_TIMEOUT_READ = 0.05 # timeout in seconds of trying to read the serial

INCREMENTAL_ENGINE_KEY = [0] # 0 = PROPULSION_KEY
FREQUENCY_INCREMENTAL_RPM = 0.025  # freq of sending RPM to vesc for engine in RPM_INCREMENTAL_ENGINE_KEY list.
STEP_INCREMENTAL_RPM = 500 # RPM step max by tick defined by RPM_FREQUENCY
# int; bumper is considered pressed if voltage is getting lesser (not equal) than this value
VESC_BUMBER_TRIGGER_VOLTAGE = 1
# int; bumper is considered unpressed if voltage is getting bigger (not equal) than this value
VESC_BUMBER_UNTRIGGER_VOLTAGE = 1

# engine 1 (master vesc)
# enables propulsion vesc initialization and usage
VESC_ALLOW_PROPULSION = True # propulsion GPIO stopper PIN number, set to None to disable stopper usage or if no stopper is used for this vesc
VESC_PROPULSION_STOPPER_PIN = None # set to True if GPIO returns 1 if stopper was hit, otherwise set to False
VESC_PROPULSION_STOP_SIGNAL = False # set to True to allow this axis init calibration during vesc adapter instance creation
VESC_PROPULSION_CALIBRATE_AT_INIT = False # Not used in current version of robot
VESC_PROPULSION_CALIBRATION_RPM = 0 # Not used in current version of robot
VESC_PROPULSION_CALIBRATION_MAX_TIME = 2  # Not used in current version of robot
VESC_PROPULSION_AUTODETECT_CAN_ID = False # this can id will be used if VESC_EXTRACTION_AUTODETECT_CAN_ID is set to False
VESC_PROPULSION_CAN_ID = None  # parent vesc has can_id=None

# engine 2
# enables extraction vesc initialization and usage
VESC_ALLOW_EXTRACTION = False # extraction GPIO stopper PIN number, set to None to disable stopper usage or if no stopper is used for this vesc
VESC_EXTRACTION_STOPPER_PIN = 16  # VALUE MAY BE DIFFERENT FOR EACH ROBOT # set to True if GPIO returns 1 if stopper was hit, otherwise set to False
VESC_EXTRACTION_STOP_SIGNAL = False # set to True to allow this axis init calibration during vesc adapter instance creation
VESC_EXTRACTION_CALIBRATE_AT_INIT = False 
VESC_EXTRACTION_CALIBRATION_RPM = -2500 # rpm for init calibration 
VESC_EXTRACTION_CALIBRATION_MAX_TIME = 2 # seconds; max time needed to reach stopper, calibration will be stopped after this timeout 
VESC_EXTRACTION_CALIBRATION_Z5_FIX_RPM = 2500  
VESC_EXTRACTION_CALIBRATION_Z5_FIX_TIME = 0.3 # seconds; calibration small movement down time (calibration "Z-5" fix) 
VESC_EXTRACTION_AUTODETECT_CAN_ID = False # set to False to use vesc can id from this config, set to True to try detect vesc can id during initialization
VESC_EXTRACTION_CAN_ID = 2 # this can id will be used if VESC_EXTRACTION_AUTODETECT_CAN_ID is set to False

VESC_SMOOTH_ACCEL_RPM_STEP = 2500
VESC_SMOOTH_ACCEL_TIME_STEP = 0.1  
VESC_SMOOTH_DECEL_RPM_STEP = 2500
VESC_SMOOTH_DECEL_TIME_STEP = 0.1

# ======================================================================================================================
# GPS SETTINGS
# ======================================================================================================================
GPS_PORT = "/dev/ttyTHS1"
GPS_BAUDRATE = 115200 
GPS_POSITIONS_TO_KEEP = 1000
NO_GPS_TIMEOUT = 5 
GPS_CHECK_IN_DEGRADED_MODE = 30 #Check if gps returned every GPS_CHECK_IN_DEGRADED_MODE navigation cycle.

SER2NET_CONNECT_GPS_PORT = False

# max time with no fresh GPS points before robot stops
GPS_POINT_TIME_BEFORE_STOP = 2
# max time with no fresh GPS points before gps adapter reconnects to ublox
# (starts counting after robot was stopped, not after last point received)
GPS_POINT_TIME_BEFORE_RECONNECT = 5
# True: allow robot to restart ntrip service if received GPS point's quality is not '4'; False: ignore points quality
ALLOW_GPS_BAD_QUALITY_NTRIP_RESTART = True
# True: allow robot to stop if GPS point quality is not '4'; False: don't stop the robot
ALLOW_GPS_BAD_QUALITY_STOP = True
# True: stop robot if prev-cur position distance is bigger than PREV_CUR_POINT_MAX_DIST and wait for another
# cur_pos point with suitable distance, no more than specified in GPS_DIST_WAIT_TIME_MAX time;
# False: ignore prev-cur position distance
ALLOW_GPS_PREV_CUR_DIST_STOP = True
# if ALLOW_GPS_PREV_CUR_DIST_STOP = True; mms; max allowed distance between cur and prev position, will stop robot
# if distance has exceeded this value
PREV_CUR_POINT_MAX_DIST = 10000
# if ALLOW_GPS_PREV_CUR_DIST_STOP = True; seconds to wait for cur_pos point with prev-cur pos
# distance < PREV_CUR_POINT_MAX_DIST before stop reading new points and accept current cur_pos point
GPS_DIST_WAIT_TIME_MAX = 30


# ======================================================================================================================
# SMOOTHIE SETTINGS
# ======================================================================================================================
SMOOTHIE_HOST = "/dev/ttyACM0" # smoothie's ip address for telnet or port for usb serial connector
# Deprecated for most scripts that use a function to find it dynamically.
SMOOTHIE_BAUDRATE = 115200
SMOOTHIE_BACKEND = 2  # 1 = telnet, 2 = serial

X_MIN = 0
X_MAX = 450 
X_F_MIN = 0
X_F_MAX = 20000
X_COEFFICIENT_TO_MM = 1

Y_MIN = 0
Y_MAX = 216 
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
EXTRACTION_Z = 30 # drill version value
Z_F_EXTRACTION_UP = 1500 
Z_F_EXTRACTION_DOWN = 1950 
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
CALIBRATION_ORDER = ["Z", "Y", "X", "A", "B", "C"]

# DIRECTION WHEELS
A_MIN = -11
A_MAX = 11
#NOT USED
B_MIN = -float("inf")
B_MAX = float("inf")
C_MIN = -float("inf")
C_MAX = float("inf")
# DIRECTION WHEELS
A_F_MIN = 1
A_F_MAX = 4000 #default value 4000
A_COEFFICIENT_TO_MM = 1
A_F_UI = 1000 #default value 1000
#NOT USED
B_F_MIN = 1
B_COEFFICIENT_TO_MM = 1
B_F_MAX = 4000

C_F_MIN = 1
C_F_MAX = 1000
C_COEFFICIENT_TO_MM = 1

A_ONE_DEGREE_IN_SMOOTHIE = 2  # A axis
A_DEGREES_PER_SECOND = 5  # A axis
NAV_TURN_WHEELS_CENTER = 0

RETRY_CORK_UP_MIN = 1
RETRY_CORK_UP_MAX = 2
RETRY_CORK_UP_STEP = 1


# ======================================================================================================================
# DETECTION SETTINGS
# ======================================================================================================================
ALLOW_PRECISE_RESCAN = True
ALLOW_PRECISE_SINGLE_SCAN_BEFORE_PDZ = False

CAMERA_POSITIONS = [(X_MAX/2, 0)] # smoothie global coordinates to take photos from for forming plants list. Format is (x, y)
#CAMERA_POSITIONS = [(X_MAX/3, 0), (2*X_MAX/3, 0)] # smoothie global coordinates to take photos from for forming plants list. Format is (x, y)
# pdz distances are amounts of px from scene center
PDZ_DISTANCES = [{"top": 1000, "bot": 1000, "left": 1000, "right": 1000}] # precice detection zone sizes. At each camera position scan only plants inside this zone is added to extraction list
# values are px count from scene center to. Format is {"top": int, "bot": int, "left": int, "right": int}

# True: allows movement on X axis (from side to side) during periphery scans to increase camera view; False: usual scans
# NOTICE: enabling this will prevent usage of ALLOW_PRECISE_SINGLE_SCAN_BEFORE_PDZ due to incompatibility
ALLOW_X_MOVEMENT_DURING_SCANS = False
# list of camera positions for X axis movement during cruise periphery scans if they are enabled. Format is a list of
# separate camera X positions.
X_MOVEMENT_CAMERA_POSITIONS = [X_MAX/3, 2*X_MAX/3]
# list of camera positions forces values for X axis movement during cruise periphery scans is they are enabled. Format
# is a list of separate camera X positions force values.
X_MOVEMENT_CAMERA_X_F = [X_F_MAX, X_F_MAX]
# list of coordinates of areas on camera image. Plants out of current area is being ignored (to avoid triggering on
# plants which are out of robot's working zone). Each image area must have a same index in this list as camera position
# for which this area. Be aware that this area will be applied instantly during movement while cork won't reach his
# X target position instantly. This may lead to situations when robot can trigger on weed out of working area, or pass
# by a weed because cork didn't reach his target position, and area is build in a way to prevent trigger on weeds
# outside working zone when cork is already in his target position. Quick enough X movement may fix this possible issue.
# Format is a list of dict items [{"top": int, "bot": int, "left": int, "right": int}, {...}, ...] where each dict is a
# separate image zone having same index as it's camera position. Zone T B L R values are pixels from image scene center.
X_MOVEMENT_IMAGE_ZONES = [{"top": 1000, "bot": 1000, "left": 1000, "right": 1000}]
# count of models to load; supports 1 or 2 NN models for now
# 1 = only periphery NN is loaded and used for all scans types; 2 = load periphery and precise
NN_MODELS_COUNT = 2


# ======================================================================================================================
# APP SETTINGS
# ======================================================================================================================
SAVE_DEBUG_IMAGES = False
DEBUG_IMAGES_PATH = "debug_images/"
ALLOW_GATHERING = False
DATA_GATHERING_MAX_IMAGES = 400 #ONLY if ALLOW_GATHERING = True

AUDIT_MODE = False
AUDIT_DIVIDER = 6
AUDIT_OUTPUT_FILE = "audit.txt"

VERBOSE = False
VERBOSE_EXTRACT = True
FILES_TO_KEEP_COUNT = 600
LOG_SPEED_MODES = True
PRINT_SPEED_MODES = True
UI_VERBOSE_LOGGING = False

LEARN_GO_STRAIGHT = False
LEARN_GO_STRAIGHT_UI = False
MIN_PERPENDICULAR_GO_STRAIGHT = 100 # in mm
VALUES_LEARN_GO_STRAIGHT = 40

ANTI_THEFT_ZONE_RADIUS = 5000

ROBOT_SN = "SN000"

# posix_ipc.MessageQueue in main script setting
# int: override OS's default max amount messages in queue before new msg sending will be blocked and forced to wait;
# None: use OS default
QUEUE_MESSAGES_MAX = 200
# max seconds to wait for sending position in main loop. Currently positions are sent not more often than once per sec
# Position sending will be canceled and skipped if wait time is exceeded this value.
QUEUE_WAIT_TIME_MAX = 0.03
# True: enable queue messages sending performance tracking and writing to log at cur target point arrival;
# False: disable performance tracking and writing to log
QUEUE_TRACK_PERFORMANCE = False

MAX_LENGHT_POINT_HISTORY = 100
ROBOT_SYNTHESIS_HOST = "127.0.0.1"
ROBOT_SYNTHESIS_PORT = 2006
DATAGATHERING_HOST = "172.16.0.10"
DATAGATHERING_PORT = 8080

LIFE_LINE_PIN = 77 #77 for lifeline with nvidia board (board pin 38) | 78 for motherboard V2.4
# ======================================================================================================================
# WEB INTERFACE SETTINGS
# ======================================================================================================================
UI_LANGUAGE = "en"

SLIDER_CREATE_FIELD_MIN = 15 #minimum of the slider allowing to configure the second segment of the terrain on the ui.
SLIDER_CREATE_FIELD_MAX = 150 #maximum of the slider allowing to configure the second segment of the terrain on the ui
SLIDER_CREATE_FIELD_DEFAULT_VALUE = 25 #default value of the slider allowing to configure the second segment of the terrain on the ui
SLIDER_CREATE_FIELD_STEP = 1 #step of the slider allowing to configure the second segment of the terrain on the ui

FRAME_SHOW = False
SHARED_MEMORY_NAME_DETECTED_FRAME = "/detected_frame"

QUEUE_NAME_UI_MAIN = "/queue_ui_main"
QUEUE_NAME_UI_NOTIFICATION = "/queue_ui_notification"

CONTINUOUS_INFORMATION_SENDING = True
ALIVE_SENDING_TIMEOUT = 1

TIMEOUT_JOYSTICK_USER_ACTION = 1

# ======================================================================================================================
# EXTRACTION MANAGER SETTINGS
# ======================================================================================================================

EXTRACTION_PATTERNS_OFFSET_MM = 5 
EXTRACTION_MAP_CELL_SIZE_MM = 10
EXTRACTION_TRIES_PER_PLANT = 1 # defines how many times robot will try to extract plant or plants group in undist. zone
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
# path to a csv file with a control points. Each point has 7 elements, current format is:
# [X_px: int, Y_px: int, X_diff_px: int, Y_diff_px: int, Distance_px: float, X_mm: float, Y_mm: float]
CONTROL_POINTS_CSV_PATH = "control_points_v2.csv"


# ======================================================================================================================
# NTRIP CLIENT SETTINGS
# ======================================================================================================================
NTRIP = True 
FIND_MOUNTPOINT = True #FOR CENTIPEDE: Allows you to find the station closest to MAX_DISTANCE_MOUNTPOINT maximum.
SEND_LOCATION_TO_NTRIP = False #FOR MOVERTK: Allows us to send our coordinates to the rtk caster

NTRIP_USER = "centipede" 
NTRIP_PASSWORD = "centipede" 
NTRIP_CASTER = "caster.centipede.fr" 
NTRIP_PORT = 2101
NTRIP_MOUNTPOINT = "LIENNS" #Mountpoint of the caster, if SEND_LOCATION_TO_NTRIP=true the program will take the closest.

NTRIP_OUTPUT_PORT = "/dev/ttyACM1"
NTRIP_OUTPUT_BAUDRATE = 115200

NTRIP_RESTART_TIMEOUT = 10
MAX_DISTANCE_MOUNTPOINT = 1000 #Allows you to find the station closest to MAX_DISTANCE_MOUNTPOINT maximum if FIND_MOUNTPOINT=True.
RTK_ID_SEND = [(1005, 1006), (1124, 1127), (1084, 1087), (1074, 1077)] #Id of the rtk frames that will be sent to the gps
# 1005 : Stationary RTK reference station ARP
# 1006 : Stationary RTK reference station ARP with antenna height
# Origine RTK           MSM4        MSM7
# BeiDou (Chine)        1124        1127
# GPS (USA)             1074        1077
# GLONASS (Russie)      1084        1087
# Galileo (UE)          1094        1097
NTRIP_SLEEP_TIME = 10 # Time in seconds between two sessions of getting data (MSM and ARP)

CASTER_RESPONSE_DECODE= "ascii"  #"iso-8859-16" for swissgreen

# ======================================================================================================================
# SEEDER SETTINGS
# ======================================================================================================================
SEEDER_QUANTITY = 0  # how many times to plant seeds at hole after extraction; 0 means disabled
SEEDER_FILL_DELAY = 0.5  # delay between two M280 commands
SEEDER_PLANT_DELAY = 0.3  # seconds; delay time to allow seeder finish his job (was SEEDER_DELAY earlier)
SEEDER_EXT_OFFSET_X = 0  # mm; if not 0 - will move cork for this value on X axis before using seeder (extraction mode)
SEEDER_EXT_OFFSET_X_F = 20000  # offset force for X axis
SEEDER_EXT_OFFSET_Y = 25  # mm; if not 0 - will move cork for this value on Y axis before using seeder (extraction mode)
SEEDER_EXT_OFFSET_Y_F = 20000  # offset force for Y axis
SEEDER_CLOSE_COMMAND = 2 # Command send to close exit of robot's seeder (M280 S[SEEDER_CLOSE_COMMAND])
SEEDER_OPEN_COMMAND = 5.5 # Command send to open exit of robot's seeder (M280 S[SEEDER_OPEN_COMMAND])

# ======================================================================================================================
# MILLING SETTINGS
# ======================================================================================================================
MILLING_CORK_GROUND_REACHING_DELAY = 1  # seconds; delay to let the cork to reach the ground before moving cork aside
# additional security: if GPIO stopper fails cork will be stopped after this time passed
MILLING_CORK_STOPPER_REACHING_MAX_TIME = 1  # seconds
MILLING_Y_STEP = 10  # mms; define Y axis step for milling to "fill up" the plant box
# plant box area is being "filled" by movements with cork enabled, if last X line of cork movement is further
# from the edge of plant box by this value - additional line will be added to "fill" up the box
MILLING_LAST_LINE_THRESHOLD = 2  # mms
MILLING_X_F = 20000
MILLING_Y_F = 20000
MILLING_CORK_DOWN_RPM = 20000
MILLING_CORK_UP_RPM = -5500

# change box milling area, box X size is multiplied by this value
# i.e. 1 is original box size, 2 doubles box size, 0.5 reduces by half
MILLING_PLANT_BOX_X_SIZE_SCALE = 1
# change box milling area, box Y size is multiplied by this value
# i.e. 1 is original box size, 2 doubles box size, 0.5 reduces by half
MILLING_PLANT_BOX_Y_SIZE_SCALE = 1


# ======================================================================================================================
# YOLO PERIPHERY NETWORK SETTINGS
# ======================================================================================================================
PERIPHERY_CONFIDENCE_THRESHOLD = 0.1
PERIPHERY_INPUT_SIZE = (416, 416)
PERIPHERY_CLASSES_FILE = "yolo/Y0016.names"
PERIPHERY_MODEL_PATH = "yolo/Y0016_416_416.trt"

PERIPHERY_HIER_THRESHOLD = 0.5
PERIPHERY_NMS_THRESHOLD = 0.4

PERIPHERY_DNN_BACKEND = 5
PERIPHERY_DNN_TARGET = 6
PERIPHERY_WRAPPER = 1

PERIPHERY_CONFIG_FILE = "yolo/Y0016_416.cfg"
PERIPHERY_WEIGHTS_FILE = "yolo/Y0016.weights"
PERIPHERY_DATA_FILE = "yolo/Y0016.data"

# ======================================================================================================================
# YOLO PRECISE NETWORK SETTINGS
# ======================================================================================================================
PRECISE_CONFIDENCE_THRESHOLD = 0.1
PRECISE_INPUT_SIZE = (832, 832)
PRECISE_CLASSES_FILE = "yolo/Y0016.names"
PRECISE_MODEL_PATH = "yolo/Y0016_832_832.trt"  # for TRT wrapper

PRECISE_HIER_THRESHOLD = 0.5
PRECISE_NMS_THRESHOLD = 0.4

PRECISE_DNN_BACKEND = 5
PRECISE_DNN_TARGET = 6
PRECISE_WRAPPER = 1

PRECISE_CONFIG_FILE = "yolo/Y0016_832.cfg"
PRECISE_WEIGHTS_FILE = "yolo/Y0016.weights"
PRECISE_DATA_FILE = "yolo/Y0016.data"

# ======================================================================================================================
# CAMERA SETTINGS
# ======================================================================================================================
CAMERA_W = 1920
CAMERA_H = 1080
APPLY_IMAGE_CROPPING = False
CROP_W_FROM = 0 
CROP_W_TO = 1920 
CROP_H_FROM = 0 
CROP_H_TO = 1080
CAMERA_FRAMERATE = 16
CAMERA_FLIP_METHOD = 0
SCENE_CENTER_X = 1000
SCENE_CENTER_Y = 980
ONE_MM_IN_PX = 3.2
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
# working zone polygon's points relative to scene center (used for abs points calculation during calibrations)
WORKING_ZONE_POLY_POINTS_REL = [[621, 96], [622, -48], [609, -189], [585, -317], [558, -431], [389, -562], [-10, -564], [-407, -564], [-585, -445], [-611, -338], [-637, -208], [-646, -67], [-646, 89], [-21, 119]]
# max res is used during deployment calibration to detect scene center position
DEPLOYMENT_CAMERA_MAX_W = 3264
# max res is used during deployment calibration to detect scene center position
DEPLOYMENT_CAMERA_MAX_H = 2464
# min possible camera framerate at max possible camera resolution
DEPLOYMENT_CAMERA_MIN_FRAMERATE = 16
# defines how much px must be grabbed from the top side of scene center during cropping
DEPLOYMENT_CROP_GRAB_TOP_PX = 1260
# defines how much px must be grabbed from the bottom side of scene center during cropping
DEPLOYMENT_CROP_GRAB_BOT_PX = 240
# defines how much px must be grabbed from the left side of scene center during cropping
DEPLOYMENT_CROP_GRAB_LEFT_PX = 1000
# defines how much px must be grabbed from the right side of scene center during cropping
DEPLOYMENT_CROP_GRAB_RIGHT_PX = 1000

CORK_TO_CAMERA_DISTANCE_X = 0  # # distance between camera and cork on the robot, X axis, relative, mm
CORK_TO_CAMERA_DISTANCE_Y = 30  # distance between camera and cork on the robot, Y axis, relative, mm


# ======================================================================================================================
# PATHS SETTINGS
# ======================================================================================================================
INPUT_GPS_FIELD_FILE = "field.txt"
MINIMUM_SIZE_FIELD = 15 # Size minimum of a field (in meter)
CHECK_MINIMUM_SIZE_FIELD = True # Do check if a field is bigger than the minimum
OUTPUT_GPS_HISTORY_FILE = "gps_history.txt"
DARKNET_LIB_DIR_PATH = "/home/violette/field/darknet/"
STATISTICS_OUTPUT_FILE = "statistics.txt"
DATACOLLECTOR_SAVE_FILE = "datacollection_save.dat"
LAST_ANGLE_WHEELS_FILE = "last_angle_wheels.txt"
LEARN_GO_STRAIGHT_FILE = "learn_go_straight.txt"

LOG_ROOT_DIR = "logs/"
DATA_GATHERING_DIR = "gathered_data/"

# statistics database file name (just a name, not full path)
STATISTICS_DB_FILE_NAME = "statistics_db.sqlite3"


# ======================================================================================================================
# PREDICTION SETTINGS
# ======================================================================================================================
ZONE_THRESHOLD_DEGREE = [(436,5),(697,7),(796,17),(849,15),(953,6)]


# ======================================================================================================================
# NAVIGATION TEST MODE SETTINGS
# ======================================================================================================================
NAVIGATION_TEST_MODE = False # mode allowing the robot to do A->B, B->A
#The robot will aim for the furthest point, 
#when it reaches this point it will wait for a press on enter to go to the furthest point from it.
DISPLAY_INSTRUCTION_PATH = False #Allows to display the robot guide points on the ui.
DELTA_DISPLAY_INSTRUCTION_PATH = 15 #Number of guide points display on the ui.
POINT_A = [[46.1579425, -1.1344245], -0.5] #Point coordinate for test navigation mode, [[lat,long],speed]
# the speed represents the speed the robot will apply to reach this point.
POINT_B = [[46.1577957, -1.1347992], 0.5] #Point coordinate for test navigation mode, [[lat,long],speed]
# the speed represents the speed the robot will apply to reach this point.


# ======================================================================================================================
# UNSORTED KEYS
# ======================================================================================================================
# wheels mechanics hotfix update
# enables temp. hotfix: checking robot position and wheels turning to right WITHOUT adapter usage
ENABLE_ADDITIONAL_WHEELS_TURN = False
# this address of telnet smoothie is used when wheels turning hotfix is enabled (ENABLE_ADDITIONAL_WHEELS_TURN = True)
SMOOTHIE_TELNET_HOST = "192.168.9.101"
# mms; wheels will be turned if robot position perpendicular distance is bigger than this value and robot is on the
# left side of last movement line
ADDITIONAL_WHEELS_TURN_THRESHOLD = 1500
# smoothie wheels key
ADDITIONAL_WHEELS_KEY = "A"
# smoothie value to turn
ADDITIONAL_WHEELS_VALUE = -5
# smoothie force to turn
ADDITIONAL_WHEELS_FORCE = 2000

# gps point reading time predictor update
# True: enable gps points reading time predictions and allow inference (and currently speed control too) skip if
# inference will delay new gps point reading
# False: older way without prediction of gps points reading time; do inference once, then try to read point, ...
ALLOW_GPS_TIME_PREDICTIONS_LIMITING_INFERENCE = True
# seconds; this time value is used to determine how much time inference could take and if there's enough time for
# single inference, or need to start to skip inferences and read gps points (works only if gps points reading time
# predictions and inference limiting are allowed)
INFERENCE_MAX_TICK_TIME = 0.040
# seconds; defines how often GPS points are expected to come. This value is used to predict GPS reading time and
# limit inference if inference may delay new gps point reading (works only if gps points reading time
# # predictions and inference limiting are allowed)
GPS_POINT_WAIT_TIME_MAX = 0.25

# demo pause update
ALLOW_DEMO_PAUSES = False
DEMO_PAUSES_HOST = "127.0.0.1"
DEMO_PAUSES_PORT = 4546
