import cv2
import threading
from flask import Response, Flask, request
from config import config
import detection
import os
from multiprocessing import Process
import sys
from flask_cors import CORS
import adapters
import logging
from engineio.payload import Payload
from werkzeug.serving import make_server

class ServerCamLive:

    def __init__(self, args):
        self.__app = Flask(__name__)
        self.__setting_flask()
        self.__init_flask_route()  # ROUTE FLASK
        self.__use_detector_arg = False
        if len(sys.argv)>1:
            self.__use_detector_arg = not args[1]=="False"
        self.__thread_alive = True
        self.__video_frame = None
        self.__detector = None
        self.__thread_lock = threading.Lock()
        self.__percent_rescale=50
        if self.__use_detector_arg:
            self.__detector = detection.YoloTRTDetector(
                    config.PERIPHERY_MODEL_PATH,
                    config.PERIPHERY_CLASSES_FILE,
                    config.PERIPHERY_CONFIDENCE_THRESHOLD,
                    config.PERIPHERY_NMS_THRESHOLD,
                    config.PERIPHERY_INPUT_SIZE)
        self.__camera = adapters.CameraAdapterDR_U3_50Y2C_C3_S(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                                config.CROP_H_TO, config.CV_ROTATE_CODE,
                                                config.ISP_DIGITAL_GAIN_RANGE_FROM, config.ISP_DIGITAL_GAIN_RANGE_TO,
                                                config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                                config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                                config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                                config.CAMERA_H, config.CAMERA_FRAMERATE, config.CAMERA_FLIP_METHOD)
        self.__process_thread = threading.Thread(target=self.captureFrames)
        self.__process_thread.daemon = True
        self.__process_thread.start()
        
    def run(self, host=None, port=None, debug=None, load_dotenv=True, **options):
        # Debug/Development
        # self.__app.run(host, port, debug, load_dotenv, **options)
        # Production
        self.__server = make_server(host, port, self.__app)
        self.__server.serve_forever()
        
    def exit(self):
        print(f"[{self.__class__.__name__}] -> Stoping class...")
        self.__thread_alive = False
        self.__process_thread.join()
        print(f"[{self.__class__.__name__}] -> Closing camera...")
        self.__camera.release()
        print(f"[{self.__class__.__name__}] -> Camera closed. ✅")
        print(f"[{self.__class__.__name__}] -> Closing server...")
        self.__server.shutdown()
        print(f"[{self.__class__.__name__}] -> Server closed. ✅")

    def __init_flask_route(self):
        self.__app.add_url_rule("/video", view_func=self.streamFrames)

    def __setting_flask(self):
        CORS(self.__app)
        self.__app.config['DEBUG'] = False
        self.__app.logger.disabled = True
        self.__log = logging.getLogger('werkzeug')
        self.__log.disabled = True
        Payload.max_decode_packets = 500

    def __rescale_frame(self, frame):
        width = int(frame.shape[1] * self.__percent_rescale/ 100)
        height = int(frame.shape[0] * self.__percent_rescale/ 100)
        dim = (width, height)
        return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

    def captureFrames(self):
        while self.__thread_alive:
            frame = self.__camera.get_image()

            with self.__thread_lock:
                self.__video_frame = frame
                
            cv2.waitKey(int(1000/config.CAMERA_FRAMERATE))
    
    def encodeFrame(self):
        print(f"[{self.__class__.__name__}] -> Encoding...")
        while True:
            with self.__thread_lock:
                if self.__video_frame is None:
                    continue
                frame = self.__video_frame
            if self.__detector:
                plants_boxes = self.__detector.detect(frame, True)
                frameFinal = detection.draw_boxes(frame, plants_boxes)
            else:
                frameFinal = frame
            frameFinal = self.__rescale_frame(frameFinal)
            return_key, encoded_image = cv2.imencode(".jpg", frameFinal)
            if not return_key:
                continue
            # Output image as a byte array
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
                bytearray(encoded_image) + b'\r\n')

    def streamFrames(self):
        return Response(self.encodeFrame(), mimetype = "multipart/x-mixed-replace; boundary=frame")


if __name__ == '__main__':
    serverCamLive = ServerCamLive(sys.argv)
    try:
        serverCamLive.run("0.0.0.0",8080,False, use_reloader=False)
    finally:
        serverCamLive.exit()
        
    