import cv2
import threading
from flask import Response, Flask
from config import config
import detection
import os
from multiprocessing import Process
import sys

# Image frame sent to the Flask object
global video_frame
video_frame = None

# Use locks for thread-safe viewing of frames in multiple browsers
global thread_lock 
thread_lock = threading.Lock()

#Use for detect
global use_detector 
use_detector = True
        
ispdigitalgainrange_from = config.ISP_DIGITAL_GAIN_RANGE_FROM
ispdigitalgainrange_to = config.ISP_DIGITAL_GAIN_RANGE_TO
gainrange_from = config.GAIN_RANGE_FROM
gainrange_to = config.GAIN_RANGE_TO
exposuretimerange_from = config.EXPOSURE_TIME_RANGE_FROM
exposuretimerange_to = config.EXPOSURE_TIME_RANGE_TO
capture_width = config.CAMERA_W
capture_height = config.CAMERA_H
framerate = config.CAMERA_FRAMERATE
crop_h_from = config.CROP_H_FROM
crop_h_to = config.CROP_H_TO
crop_w_from = config.CROP_W_FROM
crop_w_to = config.CROP_W_TO
nvidia_flip_method = config.CAMERA_FLIP_METHOD
aelock = "aelock=true " if config.AE_LOCK else ""

if config.APPLY_IMAGE_CROPPING:
    GST_CONFIG = (
        "nvarguscamerasrc "
        "ispdigitalgainrange=\"%.2f %.2f\" "
        "gainrange=\"%.2f %.2f\" "
        "exposuretimerange=\"%d %d\" "
        "%s"
        "! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv top=%d bottom=%d left=%d right=%d flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            ispdigitalgainrange_from,
            ispdigitalgainrange_to,
            gainrange_from,
            gainrange_to,
            exposuretimerange_from,
            exposuretimerange_to,
            aelock,
            capture_width,
            capture_height,
            framerate,
            crop_h_from,
            crop_h_to,
            crop_w_from,
            crop_w_to,
            nvidia_flip_method,
            crop_w_to-crop_w_from,
            crop_h_to-crop_h_from
        )
    )
else:
    GST_CONFIG = (
        "nvarguscamerasrc "
        "ispdigitalgainrange=\"%.2f %.2f\" "
        "gainrange=\"%.2f %.2f\" "
        "exposuretimerange=\"%d %d\" "
        "%s"
        "! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            ispdigitalgainrange_from,
            ispdigitalgainrange_to,
            gainrange_from,
            gainrange_to,
            exposuretimerange_from,
            exposuretimerange_to,
            aelock,
            capture_width,
            capture_height,
            framerate,
            nvidia_flip_method,
            capture_width,
            capture_height
        )
    )

def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

# Create the Flask object for the application
app = Flask(__name__)

global thread_alive
thread_alive=True

def captureFrames():
    global video_frame, thread_lock, thread_alive

    # Video capturing from OpenCV
    video_capture = cv2.VideoCapture(GST_CONFIG, cv2.CAP_GSTREAMER)

    while video_capture.isOpened() and thread_alive:
        return_key, frame = video_capture.read()
        if not return_key:
            break

        # Create a copy of the frame and store it in the global variable,
        # with thread safe access
        with thread_lock:
            video_frame = frame
        
        key = cv2.waitKey(30) & 0xff
        if key == 27:
            break

    video_capture.release()
        
def encodeFrame():
    global thread_lock
    while True:
        # Acquire thread_lock to access the global video_frame object
        with thread_lock:
            global video_frame, use_detector
            if video_frame is None:
                continue

            frame = video_frame

        if use_detector:
            global detector
            plants_boxes = detector.detect(frame, True)
            frameFinal = detection.draw_boxes(frame, plants_boxes)
        else:
            frameFinal = frame

        frameFinal = rescale_frame(frameFinal, percent=50)
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

    use_detector_arg = True

    if len(sys.argv)>1:
        use_detector_arg = sys.argv[1]=="True"

    if use_detector and use_detector_arg:
        global detector
        #detector = detection.YoloDarknetDetector(config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_CONFIG_FILE,
        #                                                         config.PERIPHERY_DATA_FILE, config.PERIPHERY_CONFIDENCE_THRESHOLD,
        #                                                         config.PERIPHERY_HIER_THRESHOLD, config.PERIPHERY_NMS_THRESHOLD)
        detector = detection.YoloTRTDetector(config.PERIPHERY_MODEL_PATH, config.PERIPHERY_CONFIDENCE_THRESHOLD, config.PERIPHERY_NMS_THRESHOLD)

    process_thread = threading.Thread(target=captureFrames)
    process_thread.daemon = True
    process_thread.start()
    app.run("0.0.0.0",8080,False)

    