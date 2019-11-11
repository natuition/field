import adapters
import detection
import cv2 as cv
import datetime
import os
import time
from config import config
import glob


# circle working area
working_zone_radius = 850
# circle undistorted area
undistorted_zone_radius = 240


def draw_zones_circle(image, center_x, center_y, undist_zone_radius, work_zone_radius):
    cv.circle(image, (center_x, center_y), undist_zone_radius, (0, 0, 255), thickness=3)
    cv.circle(image, (center_x, center_y), work_zone_radius, (0, 0, 255), thickness=3)
    return image


def main():
    # check if input folder exists
    if not os.path.exists(config.INPUT_IMG_DIR):
        try:
            os.mkdir(config.INPUT_IMG_DIR)
        except OSError:
            print("Creation of the directory %s failed" % config.INPUT_IMG_DIR)
            exit()
        else:
            print("Successfully created the directory %s " % config.INPUT_IMG_DIR)

    # check if output folder exists
    if not os.path.exists(config.OUTPUT_IMG_DIR):
        try:
            os.mkdir(config.OUTPUT_IMG_DIR)
        except OSError:
            print("Creation of the directory %s failed" % config.OUTPUT_IMG_DIR)
            exit()
        else:
            print("Successfully created the directory %s " % config.OUTPUT_IMG_DIR)

    print("Loading YOLO...")
    det = detection.YoloOpenCVDetection()
    print("Loading camera...")
    camera = adapters.CameraAdapterIMX219_170()
    print("Camera warming up...")
    time.sleep(3)
    print("Loading complete.")

    counter = 1
    sep = " "

    # make photos
    while True:
        action = input("Type empty string (just hit enter key) to make photo, type anything to begin detection: ")
        if action != "":
            break

        image = camera.get_image()
        cv.imwrite(config.INPUT_IMG_DIR + str(datetime.datetime.now()) + sep + str(counter) + ".jpg", image)
        counter += 1

    # detect plants on that photos
    images = glob.glob(config.INPUT_IMG_DIR + "*.jpg")
    counter = 1
    for file_path in images:
        print("Processing " + str(counter) + " of " + str(len(images)) + " images")
        counter += 1

        img = cv.imread(file_path)
        img_y_c, img_x_c = int(img.shape[0] / 2), int(img.shape[1] / 2)

        boxes = det.detect(img)

        for i in range(len(boxes)):
            detection.draw_box(img, boxes[i])

        img = draw_zones_circle(img, img_x_c, img_y_c, undistorted_zone_radius, working_zone_radius)

        # put results in the input folder
        # cv.imwrite(file_path[:-4] + " - result.jpg", img)

        file_name = file_path.split("\\")[-1]
        cv.imwrite(config.OUTPUT_IMG_DIR + "Result " + file_name, img)

    print("Done!")


if __name__ == "__main__":
    main()
