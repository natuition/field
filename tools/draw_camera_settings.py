"""Crops given image, draws working zone, scene center and control points on it"""

import cv2 as cv
from config import config
import numpy as np

# input
IMAGE_PATH = "1.jpg"

# output
IMAGE_RESULT_PATH = "1 result.jpg"


def crop_image(image):
    return image[config.CROP_H_FROM:config.CROP_H_TO, config.CROP_W_FROM:config.CROP_W_TO]


def draw_control_points_map(image):
    """Draws CPM on the given image"""

    print("Control points to draw:", len(config.IMAGE_CONTROL_POINTS_MAP))
    for point in config.IMAGE_CONTROL_POINTS_MAP:
        x, y = point[0], point[1]
        image = cv.circle(image, (x, y), 3, (0, 255, 0), thickness=2)
    return image


def draw_working_zone(image):
    np_points = np.array(config.WORKING_ZONE_POLY_POINTS, np.int32).reshape((-1, 1, 2))
    return cv.polylines(image, [np_points], isClosed=True, color=(255, 0, 0), thickness=3)


def draw_undistorted_zone(image):
    return cv.circle(image, (config.SCENE_CENTER_X, config.SCENE_CENTER_Y), config.UNDISTORTED_ZONE_RADIUS, (255, 0, 0),
                     thickness=3)


def draw_scene_center(image):
    return cv.circle(image, (config.SCENE_CENTER_X, config.SCENE_CENTER_Y), 3, (0, 0, 255),
                     thickness=3)


def main():
    print("Loading", IMAGE_PATH)
    image = cv.imread(IMAGE_PATH)
    image = crop_image(image)
    image = draw_undistorted_zone(image)
    image = draw_working_zone(image)
    image = draw_control_points_map(image)
    image = draw_scene_center(image)
    cv.imwrite(IMAGE_RESULT_PATH, image)
    print("Done; result saved as", IMAGE_RESULT_PATH)


if __name__ == '__main__':
    main()
