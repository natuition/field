import cv2
from flask import Response, Flask
from config import config

import posix_ipc
import os
import numpy as np
from mmap import mmap

# Create the Flask object for the application
app = Flask(__name__)
        
def encodeFrame():
    while True:

        video_frame = sharedMemory
        if video_frame is None:
            continue

        return_key, encoded_image = cv2.imencode(".jpg", video_frame)
        if not return_key:
            continue

        # Output image as a byte array
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')

@app.route("/")
def streamFrames():
    return Response(encodeFrame(), mimetype = "multipart/x-mixed-replace; boundary=frame")

if __name__ == '__main__':
    sharedMemory = posix_ipc.SharedMemory(config.SHARED_MEMORY_NAME_DETECTED_FRAME)
    sharedMem = mmap(fileno=sharedMemory.fd, length=9000000)
    c = np.ndarray((1500,2000,3), dtype=np.uint8, buffer=sharedMem)
    cv2.imwrite('img.jpg', c)
    #app.run("0.0.0.0", port="888", use_reloader=False)