import cv2
from flask import Response, Flask, render_template

import sys
sys.path.append('../')
from config import config

import posix_ipc
import os
import numpy as np
from mmap import mmap

# Create the Flask object for the application
#app = Flask(__name__)
        
def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

def generate():
    sharedMemory = posix_ipc.SharedMemory(config.SHARED_MEMORY_NAME_DETECTED_FRAME)
    sharedMem = mmap(fileno=sharedMemory.fd, length=0)
    sharedMemory.close_fd()

    if config.APPLY_IMAGE_CROPPING:
        img_w = config.CROP_W_TO - config.CROP_W_FROM
        img_h = config.CROP_H_TO - config.CROP_H_FROM
    else:
        img_w = config.CAMERA_W
        img_h = config.CAMERA_H

    sharedArray = np.ndarray((img_h, img_w, 3), dtype=np.uint8, buffer=sharedMem)

    while True:

        frame = rescale_frame(sharedArray, percent=30)
        return_key, encoded_image = cv2.imencode(".jpg", frame)
        if not return_key:
            continue
        
        # Output image as a byte array
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')

def video_feed():   
	return Response(generate(), mimetype = "multipart/x-mixed-replace; boundary=frame")

def index():
	return render_template("index.html")

#if __name__ == '__main__':
#    app.run("0.0.0.0", port="888", use_reloader=False)