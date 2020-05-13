"""Takes and saves images while moving forward. Movement forward can be interrupted by keyboard interrupt (Ctrl+C)"""

import datetime
import cv2 as cv
import adapters
from config import config
import detection
import time
import os
import queue
import threading

# Vesc settings
VESC_PORT = "/dev/ttyACM0"
VESC_BAUDRATE = 115200
# VESC_RPM = -5000
# VESC_MOV_TIME = 30  # seconds to move forward
VESC_ALIVE_FREQ = 0.5  # freq of sending "keep working" signal to engines when moving
VESC_CHECK_FREQ = 0.01  # freq of checking need to stop


# Paths
NO_PLANTS_OUTPUT_PATH = "no plants detected/"
WITH_PLANTS_OUTPUT_PATH = "with plants detected/"
DRAWN_BOXES_OUTPUT_PATH = "with boxes drawn/"
SAVE_WITH_BOXES = False  # if True: will save images with boxes drawn in the separate dir

# Camera settings
ISP_DIGITAL_GAIN_RANGE_FROM = 8
ISP_DIGITAL_GAIN_RANGE_TO = 8
GAIN_RANGE_FROM = 4
GAIN_RANGE_TO = 4
EXPOSURE_TIME_RANGE_FROM = 55000
EXPOSURE_TIME_RANGE_TO = 55000
AE_LOCK = True


class CameraAdapterIMX219_170:

    def __init__(self,
                 ispdigitalgainrange_from,
                 ispdigitalgainrange_to,
                 gainrange_from,
                 gainrange_to,
                 exposuretimerange_from,
                 exposuretimerange_to,
                 aelock = False,
                 capture_width=config.CAMERA_W,
                 capture_height=config.CAMERA_H,
                 display_width=config.CAMERA_W,
                 display_height=config.CAMERA_H,
                 framerate=config.CAMERA_FRAMERATE,
                 flip_method=config.CAMERA_FLIP_METHOD):

        gst_config_old = (
                "nvarguscamerasrc ! "
                "video/x-raw(memory:NVMM), "
                "width=(int)%d, height=(int)%d, "
                "format=(string)NV12, framerate=(fraction)%d/1 ! "
                "nvvidconv flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
                % (
                    capture_width,
                    capture_height,
                    framerate,
                    flip_method,
                    display_width,
                    display_height
                )
        )

        aelock = "aelock=true " if aelock else ""
        # ispdigitalgainrange="14.72 14.72" gainrange="14.72 14.72" exposuretimerange="55000 55000" aelock=true
        gst_config = (
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
                    flip_method,
                    display_width,
                    display_height
                )
        )
        self._cap = VideoCaptureNoBuffer(gst_config, cv.CAP_GSTREAMER)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()

    def release(self):
        self._cap.release()

    def get_image(self):
        if self._cap.isOpened():
            image = self._cap.read()
            # rotate for 90 degrees and crop black zones
            return cv.rotate(image, 2)[config.CROP_H_FROM:config.CROP_H_TO, config.CROP_W_FROM:config.CROP_W_TO]
        else:
            raise RuntimeError("Unable to open camera")


class VideoCaptureNoBuffer:
    """Minimalistic layer for cv2's VideoCapture with buffer cleaning thread ()"""

    def __init__(self, *args):
        self._cap = cv.VideoCapture(*args)
        self._queue = queue.Queue()
        self._thread = threading.Thread(target=self._reader)
        self._thread.daemon = True
        self._thread.start()

    def release(self):
        self._cap.release()

    def isOpened(self):
        return self._cap.isOpened()

    # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while True:
            ret, frame = self._cap.read()
            if not ret:
                break
            if not self._queue.empty():
                try:
                    self._queue.get_nowait()  # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            self._queue.put(frame)

    def read(self):
        return self._queue.get()


def create_directories(*args):
    """Creates directories, receives any args count, each arg is separate dir"""

    for path in args:
        if not os.path.exists(path):
            try:
                os.mkdir(path)
            except OSError:
                print("Creation of the directory %s failed" % path)
            else:
                print("Successfully created the directory %s " % path)
        else:
            print("Directory %s is already exists" % path)


def get_current_time():
    """Returns formatted string with current time (YYYY-MM-DD HH-MM-SS)"""

    date = str(datetime.datetime.now())
    return date[:date.rindex(".")].replace(":", "-")


def save_image(path_to_save, image, counter, session_label, sep=" "):
    """Assembles image file name and saves received image under this name to specified directory"""

    session_label = session_label + sep if session_label != "" else ""
    cv.imwrite(path_to_save + session_label + get_current_time() + sep + str(counter) + ".jpg", image)


def main():
    #meters_multiplier = ask_meters_multiplier()
    #distance, force = ask_speed_mode(meters_multiplier)
    rpm = float(input("Set RPM: "))
    moving_time = float(input("Set moving time (seconds): "))

    print("Initializing detector...")
    create_directories(WITH_PLANTS_OUTPUT_PATH, NO_PLANTS_OUTPUT_PATH, DRAWN_BOXES_OUTPUT_PATH)
    detector = detection.YoloOpenCVDetection()

    print("Initializing smoothie...")
    smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)

    # move camera to the low-center position
    print("Moving camera to position...")
    smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2, Y=config.Y_MIN)
    smoothie.wait_for_all_actions_done()

    print("Initializing camera...")
    with CameraAdapterIMX219_170(
            ISP_DIGITAL_GAIN_RANGE_FROM,
            ISP_DIGITAL_GAIN_RANGE_TO,
            GAIN_RANGE_FROM,
            GAIN_RANGE_TO,
            EXPOSURE_TIME_RANGE_FROM,
            EXPOSURE_TIME_RANGE_TO,
            AE_LOCK
    ) as camera:

        time.sleep(2)
        counter = 1
        try:
            with adapters.VescAdapter(rpm, moving_time, VESC_ALIVE_FREQ, VESC_CHECK_FREQ, VESC_PORT, VESC_BAUDRATE) as vesc_engine:
                # start moving forward
                vesc_engine.start_moving()

                # get and save images until keyboard interrupt
                print("Starting to take photos...")
                while vesc_engine.is_movement_allowed():
                    frame = camera.get_image()
                    plants = detector.detect(frame)

                    # sort and save photo
                    if len(plants) > 0:
                        save_image(WITH_PLANTS_OUTPUT_PATH, frame, counter, "With plants")
                        if SAVE_WITH_BOXES:
                            detection.draw_boxes(frame, plants)
                            save_image(DRAWN_BOXES_OUTPUT_PATH, frame, counter, "With boxes")
                    else:
                        save_image(NO_PLANTS_OUTPUT_PATH, frame, counter, "No plants")
                    counter += 1

                    if counter % 100 == 0:
                        print("Saved", counter, "photos")
        except KeyboardInterrupt:
            vesc_engine.stop_moving()
            print("Stopped by a keyboard interrupt (Ctrl + C)")
            print("Saved", counter, "photos")
            exit(0)
    print("Saved", counter, "photos")
    print("Done!")


if __name__ == '__main__':
    main()
