import cv2
import time
import threading
from flask import Response, Flask
from config import config
import detection
import os
import numpy as np

# Image frame sent to the Flask object
global video_frame
video_frame = None

# Use locks for thread-safe viewing of frames in multiple browsers
global thread_lock 
thread_lock = threading.Lock()

#Use for detect
global use_detector 
use_detector = True

#Use for draw working zone and undistorted zone.
global draw_zones 
draw_zones = True

#Use for draw image control point.
global draw_control_point 
draw_control_point = False

#Use for save detection
#Todo : no working today
global use_save_detector
use_save_detector = False

if use_detector:
    global detector
    detector = detection.YoloDarknetDetector(config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_CONFIG_FILE,
                                                             config.PERIPHERY_DATA_FILE, config.PERIPHERY_CONFIDENCE_THRESHOLD,
                                                             config.PERIPHERY_HIER_THRESHOLD, config.PERIPHERY_NMS_THRESHOLD)
    if use_save_detector:
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        global out
        out = cv2.VideoWriter('output.avi',fourcc, 20.0, (config.CROP_W_TO-config.CROP_W_FROM,config.CROP_H_TO-config.CROP_H_FROM))
        
# GStreamer Pipeline to access the Raspberry Pi camera
GSTREAMER_PIPELINE = 'nvarguscamerasrc exposuretimerange="660000 660000" gainrange="4 4" ispdigitalgainrange="4 4" ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=21/1 ! nvvidconv flip-method=0 ! video/x-raw, width=3280, height=2464, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink wait-on-eos=false max-buffers=1 drop=True'

GST_CONFIG = (
    "nvarguscamerasrc "
    "ispdigitalgainrange=\"%.2f %.2f\" "
    "gainrange=\"%.2f %.2f\" "
    "exposuretimerange=\"%d %d\" "
    "! "
    "video/x-raw(memory:NVMM), "
    "width=(int)%d, height=(int)%d, "
    "format=(string)NV12, framerate=(fraction)%d/1 ! "
    "nvvidconv flip-method=%d ! "
    "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=(string)BGR ! appsink"
    % (
        config.ISP_DIGITAL_GAIN_RANGE_FROM,
        config.ISP_DIGITAL_GAIN_RANGE_TO,
        config.GAIN_RANGE_FROM,
        config.GAIN_RANGE_TO,
        config.EXPOSURE_TIME_RANGE_FROM,
        config.EXPOSURE_TIME_RANGE_TO,
        config.CAMERA_W,
        config.CAMERA_H,
        config.CAMERA_FRAMERATE,
        config.CAMERA_FLIP_METHOD,
        config.CAMERA_W,
        config.CAMERA_H
    )
)

# Create the Flask object for the application
app = Flask(__name__)

def draw_zone_circle(image, circle_center_x, circle_center_y, circle_radius):
    """Draws received circle on image. Used for drawing undistorted zone edges on photo"""

    return cv2.circle(image, (circle_center_x, circle_center_y), circle_radius, (0, 0, 255), thickness=3)


def draw_zone_poly(image, np_poly_points):
    """Draws received polygon on image. Used for drawing working zone edges on photo"""

    return cv2.polylines(image, [np_poly_points], isClosed=True, color=(0, 0, 255), thickness=5)

def captureFrames():
    global video_frame, thread_lock

    # Video capturing from OpenCV
    video_capture = cv2.VideoCapture(GST_CONFIG, cv2.CAP_GSTREAMER)

    while True and video_capture.isOpened():
        return_key, frame = video_capture.read()
        if not return_key:
            break

        # Create a copy of the frame and store it in the global variable,
        # with thread safe access
        with thread_lock:
            video_frame = frame.copy()
        
        key = cv2.waitKey(30) & 0xff
        if key == 27:
            break

    video_capture.release()
    if use_save_detector:
        out.release()
        
def encodeFrame():
    global thread_lock
    while True:
        # Acquire thread_lock to access the global video_frame object
        with thread_lock:
            global video_frame, use_detector
            if video_frame is None:
                continue

            original_frame = video_frame[config.CROP_H_FROM:config.CROP_H_TO, config.CROP_W_FROM:config.CROP_W_TO]

            if use_detector:
                global detector
                plants_boxes = detector.detect(original_frame)

            if draw_zones:
                undistorted_zone_radius = config.UNDISTORTED_ZONE_RADIUS
                poly_zone_points_cv = np.array(config.WORKING_ZONE_POLY_POINTS, np.int32).reshape((-1, 1, 2))
                original_frame = draw_zone_circle(original_frame, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, undistorted_zone_radius)
                frame = draw_zone_poly(original_frame, poly_zone_points_cv)
            else:
                frame = original_frame

            if draw_control_point:
                points_map = config.IMAGE_CONTROL_POINTS_MAP
                for i in range(len(points_map)):
                    frame = draw_zone_circle(frame, points_map[i][0],points_map[i][1], 5)

            if use_detector:
                frameFinal = detection.draw_boxes(frame, plants_boxes)
            else:
                frameFinal = frame

            if use_save_detector:
                global out
                out.write(frameFinal)

            return_key, encoded_image = cv2.imencode(".jpg", frameFinal)
            if not return_key:
                continue

        # Output image as a byte array
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
            bytearray(encoded_image) + b'\r\n')

@app.route("/video")
def streamFrames():
    return Response(encodeFrame(), mimetype = "multipart/x-mixed-replace; boundary=frame")

# check to see if this is the main thread of execution
if __name__ == '__main__':
    print("Reset service cam !")
    os.system("sudo systemctl restart nvargus-daemon")
    # Create a thread and attach the method that captures the image frames, to it
    process_thread = threading.Thread(target=captureFrames)
    process_thread.daemon = True

    # Start the thread
    process_thread.start()

    # start the Flask Web Application
    # While it can be run on any feasible IP, IP = 0.0.0.0 renders the web app on
    # the host machine's localhost and is discoverable by other machines on the same network 
    app.run("0.0.0.0", port="8080", use_reloader=False)