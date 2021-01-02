"""
This script is for dataset objects pre-markup.
Detects plants on images in a given directory, and saves detected objects in txt (yolo format) in this directory
(txt names are corresponding to img names).
Creates empty txt if no objects were detected on image.
"""

import cv2 as cv
import glob
import os
import detection
from config import config

# settings
DATASET_INPUT_DIR = "input/"
IMAGES_EXTENSIONS = [".jpg"]  # example for multiple: [".jpg", ".jpeg", ".png"] and so on
PROGRESS_PRINT_RATE = 100  # print message each 100 images


def main():
    # check for input dir existence
    if not os.path.isdir(DATASET_INPUT_DIR):
        print(DATASET_INPUT_DIR + " is not a directory or doesn't exist.")
        exit(0)

    # try to get all images to process list and check is there found any images to process
    in_img_paths = []
    for extension in IMAGES_EXTENSIONS:
        in_img_paths += glob.glob(DATASET_INPUT_DIR + "*" + extension)
    if len(in_img_paths) == 0:
        print("No images with specified extensions found in " + DATASET_INPUT_DIR + " directory.")
        exit(0)
    print(len(in_img_paths), "images found.")

    # load detector
    print("Loading periphery detector...")
    if config.PERIPHERY_WRAPPER == 1:
        periphery_detector = detection.YoloDarknetDetector(config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_CONFIG_FILE,
                                                           config.PERIPHERY_DATA_FILE,
                                                           config.PERIPHERY_CONFIDENCE_THRESHOLD,
                                                           config.PERIPHERY_HIER_THRESHOLD,
                                                           config.PERIPHERY_NMS_THRESHOLD)
    elif config.PERIPHERY_WRAPPER == 2:
        periphery_detector = detection.YoloOpenCVDetection(config.PERIPHERY_CLASSES_FILE, config.PERIPHERY_CONFIG_FILE,
                                                           config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_INPUT_SIZE,
                                                           config.PERIPHERY_CONFIDENCE_THRESHOLD,
                                                           config.PERIPHERY_NMS_THRESHOLD, config.PERIPHERY_DNN_BACKEND,
                                                           config.PERIPHERY_DNN_TARGET)
    else:
        print("Wrong config.PERIPHERY_WRAPPER = " + str(config.PERIPHERY_WRAPPER) + " code. Exiting.")
        exit(0)

    # process images; in_ means input
    print("Current progress print frequency is 1 per", PROGRESS_PRINT_RATE, "image(s). Starting processing...")
    iter_number = 0
    for in_img_path in in_img_paths:
        in_img = cv.imread(in_img_path)
        plants_boxes = periphery_detector.detect(in_img)
        in_txt_path = in_img_path[:in_img_path.rfind(".")] + ".txt"

        with open(in_txt_path, "w") as txt_file:
            plant_box: detection.DetectedPlantBox
            for plant_box in plants_boxes:
                txt_file.write(plant_box.get_as_yolo(return_as_text=True) + "\n")

        # display progress
        iter_number += 1
        if iter_number % PROGRESS_PRINT_RATE == 0:
            print("Processed", iter_number, "of", len(in_img_paths), "images...")

    print("Processed", iter_number, "of", len(in_img_paths), "images.\nDone.")


if __name__ == '__main__':
    main()
