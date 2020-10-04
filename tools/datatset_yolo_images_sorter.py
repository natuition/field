"""
Loads images in given directory, and uses yolo for detection, replaces images to "false" dir if any plants are detected on the image
"""

from config import config
import cv2 as cv
import utility
import glob
import os
import detection

INPUT_IMAGES_DIR = "../input/"
OUTPUT_FALSE_IMAGES_DIR = INPUT_IMAGES_DIR + "false/"


def main():
    if not os.path.exists(INPUT_IMAGES_DIR):
        print(INPUT_IMAGES_DIR, "is not exist.")
        exit(0)
    utility.create_directories(OUTPUT_FALSE_IMAGES_DIR)

    slash = utility.get_path_slash()
    counter = 0
    empty_files = 0

    print("Loading periphery detector...")
    periphery_detector = detection.YoloOpenCVDetection("../" + config.PERIPHERY_CLASSES_FILE, "../" + config.PERIPHERY_CONFIG_FILE,
                                                       "../" + config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_INPUT_SIZE,
                                                       config.PERIPHERY_CONFIDENCE_THRESHOLD,
                                                       config.PERIPHERY_NMS_THRESHOLD, config.PERIPHERY_DNN_BACKEND,
                                                       config.PERIPHERY_DNN_TARGET)

    paths_to_images = glob.glob(INPUT_IMAGES_DIR + "*.jpg")
    for full_img_path in paths_to_images:
        if os.stat(full_img_path).st_size > 0:
            img = cv.imread(full_img_path)
            plant_boxes = periphery_detector.detect(img)
            if len(plant_boxes) > 0:
                file_name = full_img_path.split(slash)[-1]
                os.replace(full_img_path, OUTPUT_FALSE_IMAGES_DIR + file_name)
        else:
            empty_files += 1

        counter += 1
        if counter % 100 == 0:
            print("Processed", counter, "of", len(paths_to_images), "images")
    print("Done. Found and passed", empty_files, "empty image files.")


if __name__ == '__main__':
    main()
