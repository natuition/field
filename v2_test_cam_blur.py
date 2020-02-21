"""Takes and saves images while moving forward. Movement forward cannot be interrupted"""

import datetime
import cv2 as cv
import adapters
from config import config
import detection
import time
import os

OUTPUT_PATH = "blur_test_output/"


def create_directories(*args):
    """Create two photos output directories: with detected plants, and without them"""

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


def ask_meters_multiplier():
    while True:
        try:
            meters_multiplier = float(input("Set distance meter multiplier (1 = 1m, 2 = 2m, ...): "))
            if meters_multiplier <= 0.0001:
                print("Multiplier should be > 0")
                continue
            return meters_multiplier
        except KeyboardInterrupt:
            exit()
        except Exception as e:
            print(e)


def ask_speed_mode(meters_multiplier):
    # ask for movement speed mode
    while True:
        speed_mode = input("Type l to set low speed, type h to set high speed: ")
        if speed_mode == "l":
            distance = -314 * meters_multiplier
            force = 1900
            return distance, force
        elif speed_mode == "h":
            distance = -17.4 * meters_multiplier
            force = 1100
            return distance, force
        else:
            print("Wrong speed mode, can be l or h")


def get_current_time():
    """Returns formatted string with current time (YYYY-MM-DD HH-MM-SS)"""

    date = str(datetime.datetime.now())
    return date[:date.rindex(".")].replace(":", "-")


def save_image(path_to_save, image, counter, session_label, sep=" "):
    """Assembles image file name and saves received image under this name to specified directory"""

    session_label = session_label + sep if session_label != "" else ""
    cv.imwrite(path_to_save + session_label + get_current_time() + sep + str(counter) + ".jpg", image)


def main():
    meters_multiplier = ask_meters_multiplier()
    distance, force = ask_speed_mode(meters_multiplier)
    print("Initializing detector...")
    create_directories(OUTPUT_PATH)
    detector = detection.YoloOpenCVDetection()
    print("Initializing smoothie...")
    smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)

    # move camera to the low-center position
    print("Moving camera to position...")
    smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2, Y=config.Y_MIN)
    smoothie.wait_for_all_actions_done()

    print("Initializing camera...")
    with adapters.CameraAdapterIMX219_170() as camera:
        time.sleep(3)

        response = smoothie.custom_move_for(force, B=distance)
        if response == smoothie.RESPONSE_OK:
            print("Moving forward for", meters_multiplier, "meters")
        else:
            print("Couldn't move forward, smoothie error occurred:", response)
            exit(1)

        # get and save images until keyboard interrupt
        counter = 1
        print("Starting to take photos...")
        while True:
            frame = camera.get_image()
            save_image(OUTPUT_PATH, frame, counter, "Clean")
            counter += 1
            plants = detector.detect(frame)
            detection.draw_boxes(frame, plants)
            save_image(OUTPUT_PATH, frame, counter, "With Boxes")
            counter += 1
            if counter % 100 == 0:
                print("Saved", counter, "photos")


if __name__ == '__main__':
    main()
