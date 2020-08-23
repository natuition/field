import traceback
import adapters
import cv2 as cv
from config import config
import time
import datetime
import os
from matplotlib.patches import Polygon
import math

# paths
WITH_PLANTS_DIR = "with plants/"
WITHOUT_PLANTS_DIR = "without plants/"


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


def get_current_time():
    """Returns formatted string with current time (YYYY-MM-DD HH-MM-SS)"""

    date = str(datetime.datetime.now())
    return date[:date.rindex(".")].replace(":", "-")


def save_image(path_to_save, image, counter, session_label, sep=" "):
    """Assembles image file name and saves received image under this name to specified directory"""

    session_label = session_label + sep if session_label != "" else ""
    cv.imwrite(path_to_save + session_label + get_current_time() + sep + str(counter) + ".jpg", image)


def is_point_in_poly(point_x, point_y, polygon: Polygon):
    """Returns True if received polygon object contains received point, False otherwise"""

    return polygon.contains_point([point_x, point_y])


def is_point_in_circle(point_x, point_y, circle_center_x, circle_center_y, circle_radius):
    """Returns True if (x,y) point in the circle or on it's border, False otherwise"""

    return math.sqrt((point_x - circle_center_x) ** 2 + (point_y - circle_center_y) ** 2) <= circle_radius


def px_to_smoothie_value(target_px, center_px, one_mm_in_px):
    """Converts the distance given in pixels to the distance in millimeters"""

    # returns wrong sign for x because of different 0 position between smoothie and image
    return (target_px - center_px) / one_mm_in_px


def get_closest_control_point(plant_px_x, plant_px_y, points_map):
    """Returns image control point, which is closest to the given plant center point"""

    index, min_distance = None, float("inf")
    for i in range(len(points_map)):
        cur_distance = math.sqrt((plant_px_x - points_map[i][0]) ** 2 + (plant_px_y - points_map[i][1]) ** 2)
        if cur_distance < min_distance:
            min_distance, index = cur_distance, i
    return points_map[index]


def min_plant_box_dist(boxes: list, current_px_x, current_px_y):
    """Returns plant box, which is closest to the given point coordinates"""

    return min(boxes, key=lambda box: box.get_distance_from(current_px_x, current_px_y))


def gather_data(smoothie: adapters.SmoothieAdapter, camera: adapters.CameraAdapterIMX219_170, counter, session_label,
                working_zone_polygon):
    """Gathers photos from robot's on-board camera (no position changes for robot's body).
    Saves single image if no plants were detected on view scan to separate directory.
    Saves images with few positions for each detected plant if any plants detected, and saves all that images to the
    separate directory."""

    # go to the view position (y min, x max / 2)
    smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2 / config.XY_COEFFICIENT_TO_MM, Y=config.Y_MIN)
    smoothie.wait_for_all_actions_done()

    image = camera.get_image()
    save_image(WITHOUT_PLANTS_DIR, image, counter, session_label)
    counter += 1
    return counter


def main():
    try:
        create_directories(WITH_PLANTS_DIR, WITHOUT_PLANTS_DIR)
        working_zone_polygon = Polygon(config.WORKING_ZONE_POLY_POINTS)

        with adapters.SmoothieAdapter(config.SMOOTHIE_HOST) as smoothie:
            with adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                                  config.CROP_H_TO, config.CV_ROTATE_CODE,
                                                  config.ISP_DIGITAL_GAIN_RANGE_FROM, config.ISP_DIGITAL_GAIN_RANGE_TO,
                                                  config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                                  config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                                  config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                                  config.CAMERA_H, config.CAMERA_FRAMERATE,
                                                  config.CAMERA_FLIP_METHOD) as camera:
                with adapters.VescAdapter(config.VESC_RPM_SLOW, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                                          config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE) as vesc_engine:

                    counter = 1
                    session_label = input("Set session label (added to names): ")
                    input("Press enter to start.")
                    print("Data gathering started.")

                    while True:
                        counter = gather_data(smoothie, camera, counter, session_label, working_zone_polygon)
                        # move_forward_smoothie(1100, -5.2, smoothie)  # F1100, B-5.2 = 30 cm with max speed (B-104 F1900 for min speed 30 cm)  # for smoothie moving forward control
                        vesc_engine.start_moving()
                        vesc_engine.wait_for_stop()
    except KeyboardInterrupt:
        print("Stopped by a keyboard interrupt")
    except:
        print(traceback.format_exc())
    finally:
        smoothie.disconnect()
        print("Data gathering done.", str(counter - 1), "images collected at this session.")


if __name__ == "__main__":
    main()
