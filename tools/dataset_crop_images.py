"""
This script loads all images in the given directory and saves their cropped copies in another directory
"""

import cv2 as cv
import glob
import os
import platform

# paths
INPUT_DIR = ""  # "input/"
OUTPUT_DIR = "output/"

# images settings
SCENE_CENTER_X = 500
SCENE_CENTER_Y = 700
LEFT = 153
RIGHT = 300
TOP = 130
BOTTOM = 200


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


def main():
    create_directories(OUTPUT_DIR)

    slash = "\\" if platform.system() == "Windows" else "/"

    # compute cropping values
    w_from = SCENE_CENTER_X - LEFT
    w_to = SCENE_CENTER_X + RIGHT
    h_from = SCENE_CENTER_Y - TOP
    h_to = SCENE_CENTER_Y + BOTTOM

    print("wf", w_from)
    print("wt", w_to)
    print("hf", h_from)
    print("ht", h_to)

    paths_to_images = glob.glob(INPUT_DIR + "*.jpg")
    counter = 0
    for full_image_path in paths_to_images:
        img = cv.imread(full_image_path)
        img = img[h_from:h_to, w_from:w_to]
        file_name = full_image_path.split(slash)[-1]
        cv.imwrite(OUTPUT_DIR + file_name, img)
        counter += 1
        print("Processed", counter, "of", len(paths_to_images), "images")
    print("Done.")


if __name__ == '__main__':
    main()
