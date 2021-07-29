import cv2
import threading
from flask import Response, Flask
from config import config
import detection
import os
from multiprocessing import Process

# Image frame sent to the Flask object
global video_frame
video_frame = None

# Use locks for thread-safe viewing of frames in multiple browsers
global thread_lock 
thread_lock = threading.Lock()

#Use for detect
global use_detector 
use_detector = True

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

            frame = video_frame[config.CROP_H_FROM:config.CROP_H_TO, config.CROP_W_FROM:config.CROP_W_TO]

            if use_detector:
                global detector
                plants_boxes = detector.detect(frame, True)
                frameFinal = detection.draw_boxes(frame, plants_boxes)
                if use_save_detector:
                    global out
                    out.write(frameFinal)
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

    process_thread = threading.Thread(target=captureFrames)
    process_thread.daemon = True
    process_thread.start()
    app.run("0.0.0.0",8080,False)

    