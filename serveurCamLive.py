import cv2
import threading
from flask import Response, Flask, request, current_app
from config import config
import detection
import os
from multiprocessing import Process
import sys
from flask_cors import CORS

def generateGstConfig():
    aelock = "aelock=true " if config.AE_LOCK else ""

    gst_config_start = (
        "nvarguscamerasrc "
        "ispdigitalgainrange=\"%.2f %.2f\" "
        "gainrange=\"%.2f %.2f\" "
        "exposuretimerange=\"%d %d\" "
        "%s"
        "! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        
        % (
            config.ISP_DIGITAL_GAIN_RANGE_FROM,
            config.ISP_DIGITAL_GAIN_RANGE_TO,
            config.GAIN_RANGE_FROM,
            config.GAIN_RANGE_TO,
            config.EXPOSURE_TIME_RANGE_FROM,
            config.EXPOSURE_TIME_RANGE_TO,
            aelock,
            config.CAMERA_W,
            config.CAMERA_H,
            config.CAMERA_FRAMERATE,
            
        )
    )

    if config.APPLY_IMAGE_CROPPING:
        gst_config_end = (
                "nvvidconv top=%d bottom=%d left=%d right=%d flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
            % (
                config.CROP_H_FROM,
                config.CROP_H_TO,
                config.CROP_W_FROM,
                config.CROP_W_TO,
                config.CAMERA_FLIP_METHOD,
                config.CROP_W_TO-config.CROP_W_FROM,
                config.CROP_H_TO-config.CROP_H_FROM
            )
        )
    else:
        gst_config_end = (
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                config.CAMERA_FLIP_METHOD,
                config.CAMERA_W,
                config.CAMERA_H
            )
        )
    
    return gst_config_start+gst_config_end

def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

def captureFrames():
    with app.app_context():
        if current_app.thread_alive:
            video_capture = None
            try:
                video_capture = cv2.VideoCapture(generateGstConfig(), cv2.CAP_GSTREAMER)
            except KeyboardInterrupt:
                raise KeyboardInterrupt()
            except:
                pass
            if video_capture:
                while video_capture.isOpened() and current_app.thread_alive:
                    return_key, frame = video_capture.read()
                    if not return_key:
                        break

                    with current_app.thread_lock:
                        current_app.video_frame = frame
                    
                    key = cv2.waitKey(30) & 0xff
                    if key == 27:
                        break
                video_capture.release()
        
def encodeFrame():
    with app.app_context():
        while True:
            with current_app.thread_lock:

                if current_app.video_frame is None:
                    continue
                frame = current_app.video_frame

            if current_app.detector:
                plants_boxes = current_app.detector.detect(frame, True)
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


app = Flask(__name__)
CORS(app)

@app.route("/video")
def streamFrames():
    return Response(encodeFrame(), mimetype = "multipart/x-mixed-replace; boundary=frame")

if __name__ == '__main__':

    use_detector_arg = True
    if len(sys.argv)>1:
        use_detector_arg = not sys.argv[1]=="False"

    print("Reset service cam !")
    os.system("sudo systemctl restart nvargus-daemon")

    with app.app_context():
        current_app.thread_alive = True
        current_app.video_frame = None
        current_app.detector = None
        current_app.thread_lock = threading.Lock()

        if use_detector_arg:
            #current_app.detector = detection.YoloDarknetDetector(config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_CONFIG_FILE,
            #                                                         config.PERIPHERY_DATA_FILE, config.PERIPHERY_CONFIDENCE_THRESHOLD,
            #                                                         config.PERIPHERY_HIER_THRESHOLD, config.PERIPHERY_NMS_THRESHOLD)
            current_app.detector = detection.YoloTRTDetector(
                config.PERIPHERY_MODEL_PATH,
                config.PERIPHERY_CLASSES_FILE,
                config.PERIPHERY_CONFIDENCE_THRESHOLD,
                config.PERIPHERY_NMS_THRESHOLD,
                config.PERIPHERY_INPUT_SIZE)

    process_thread = threading.Thread(target=captureFrames)
    process_thread.daemon = True
    process_thread.start()
    app.run("0.0.0.0",8080,False)

    