"""
This script loads given json file and changes coordinates of zones due to given settings, and saves changed copy
Also draws black boxes on regions which is out of new image size
Input: json file, dir with images that included to given json (CARE!!! IMAGES ARE CHANGED DURING THIS SCRIPT WORK!)
Output: json changed copy, changed originals of the images (black boxes are drawn on regions that are outside of precize zone)
"""

import json
import datetime
from matplotlib.patches import Polygon
import os
import cv2 as cv

# paths
INPUT_JSON_FILE = "1.json"
INPUT_DATASET_IMAGES_DIR = "images/"
OUTPUT_JSON_FILE = "1 result.json"
UNSUPPORTED_REGIONS_LIST_FILE = "unsupported regions files.txt"
REGIONS_OUTSIDE_PRECISE_ZONE_LIST_FILE = "outside precise zone files.txt"
MISSING_DATASET_IMAGES_LIST_FILE = "missing images list.txt"

# images settings
SCENE_CENTER_X = 500
SCENE_CENTER_Y = 700
LEFT = 153
RIGHT = 300
TOP = 130
BOTTOM = 200


def get_current_time():
    """Returns formatted string with current time (YYYY-MM-DD HH-MM-SS)"""

    date = str(datetime.datetime.now())
    return date[:date.rindex(".")].replace(":", "-")


def is_point_in_poly(point_x, point_y, polygon: Polygon):
    """Returns True if received polygon object contains received point, False otherwise"""

    return polygon.contains_point([point_x, point_y])


def main():
    w_from = SCENE_CENTER_X - LEFT
    w_to = SCENE_CENTER_X + RIGHT
    h_from = SCENE_CENTER_Y - TOP
    h_to = SCENE_CENTER_Y + BOTTOM

    img_x_size = w_to - w_from
    img_y_size = h_to - h_from

    precise_zone = Polygon([[w_from, h_from], [w_to, h_from], [w_to, h_to], [w_from, h_to]])

    img_processed = 0
    regions_processed = 0
    regions_shifted = 0

    unsupported_regions_count = 0
    unsupported_regions_files = set()

    regions_outside_prec_area_count = 0
    regions_outside_prec_area_files = set()

    missing_dataset_images_count = 0
    missing_dataset_images_files = set()

    # load json
    with open(INPUT_JSON_FILE, "r") as file:
        data = json.loads(file.read())

    regions_to_remove = []

    # process regions
    for file_key in data["_via_img_metadata"]:
        file_name = data["_via_img_metadata"][file_key]["filename"]
        file_path = INPUT_DATASET_IMAGES_DIR + file_name

        img_processed += 1

        for region in data["_via_img_metadata"][file_key]["regions"]:
            regions_processed += 1

            if region["shape_attributes"]["name"] == "rect":
                x_label = "x"
                y_label = "y"

                # region center for out of new image checking
                # rcx = region["shape_attributes"][x_label] + int(region["shape_attributes"]["width"] / 2)
                # rcy = region["shape_attributes"][y_label] + int(region["shape_attributes"]["height"] / 2)
                """
            elif region["shape_attributes"]["name"] == "circle":
                x_label = "cx"
                y_label = "cy"

                # region center for out of new image checking
                rcx = region["shape_attributes"][x_label]
                rcy = region["shape_attributes"][y_label]
            """
            else:
                # print("Unsupported region shape '", region["shape_attributes"]["name"], "', THIS REGION WON'T BE SHIFTED. See info in file ", MISSED_REGIONS_FILE, sep="")
                unsupported_regions_count += 1
                unsupported_regions_files.add(file_name)
                continue

            # shift region using scene center points to compute shifted region coordinates
            # new coordinates are computed using distance from absolute region coordinates to scene center,
            # and computing this distance from new scene center
            cx = SCENE_CENTER_X  # scene center x
            cy = SCENE_CENTER_Y  # scene center y
            rx = region["shape_attributes"][x_label]  # current region x
            ry = region["shape_attributes"][y_label]  # current region y
            new_cx = LEFT  # new scene center x
            new_cy = TOP  # new scene center y
            dx = rx - cx  # distance from region x to old scene center x
            dy = ry - cy  # # distance from region y to old scene center y
            new_rx = new_cx + dx  # shifted region x
            new_ry = new_cy + dy  # shifted region y

            # add region to deletion list if any region's point is outside of new img size (precise zone)
            # and draw black box on image
            need_draw = False

            # define black box points TODO: a bug???
            # вместо обеих координат точки корректировать ту, которая вышла за предел
            reg_x_from = rx
            reg_x_to = rx + region["shape_attributes"]["width"]
            reg_y_from = ry
            reg_y_to = ry + region["shape_attributes"]["height"]
            draw_x_from = new_rx
            draw_x_to = new_rx + region["shape_attributes"]["width"]
            draw_y_from = new_ry
            draw_y_to = new_ry + region["shape_attributes"]["height"]
            if not is_point_in_poly(reg_x_from, reg_y_from, precise_zone):
                need_draw = True
                if reg_x_from <= w_from:
                    draw_x_from = 0
                if reg_y_from <= h_from:
                    draw_y_from = 0
            if not is_point_in_poly(reg_x_to, reg_y_from, precise_zone):
                need_draw = True
                if reg_x_to >= w_to:
                    draw_x_to = img_x_size
                if reg_y_from <= h_from:
                    draw_y_from = 0
            if not is_point_in_poly(reg_x_to, reg_y_to, precise_zone):
                need_draw = True
                if reg_x_to >= w_to:
                    draw_x_to = img_x_size
                if reg_y_to >= h_to:
                    draw_y_to = img_y_size
            if not is_point_in_poly(reg_x_from, reg_y_to, precise_zone):
                need_draw = True
                if reg_x_from <= w_from:
                    draw_x_from = 0
                if reg_y_to >= h_to:
                    draw_y_to = img_y_size

            # draw black box if region's any point is outside of the precise zone and remove this region from json
            if need_draw:
                # save info and stat about this region
                regions_outside_prec_area_count += 1
                regions_outside_prec_area_files.add(file_name)
                regions_to_remove.append(region)

                # drawing
                if os.path.isfile(file_path):
                    img = cv.imread(file_path)
                    img[draw_y_from:draw_y_to, draw_x_from:draw_x_to] = (0, 0, 0)
                    cv.imwrite(file_path, img)
                else:
                    if file_name not in missing_dataset_images_files:
                        missing_dataset_images_count += 1
                        missing_dataset_images_files.add(file_name)
                continue

            region["shape_attributes"][x_label] = new_rx
            region["shape_attributes"][y_label] = new_ry

            regions_shifted += 1

        # remove outside precise area regions
        for region in regions_to_remove:
            data["_via_img_metadata"][file_key]["regions"].remove(region)
        regions_to_remove.clear()

    current_time = get_current_time() + " "

    # save changed json
    with open(OUTPUT_JSON_FILE, "w") as file:
        file.write(json.dumps(data))

    # save passed unsupported regions info
    if len(unsupported_regions_files) > 0:
        with open(current_time + UNSUPPORTED_REGIONS_LIST_FILE, "w") as file:
            for item in unsupported_regions_files:
                file.write(item + "\n")

    # save list of files with regions outside the precise area
    if len(regions_outside_prec_area_files) > 0:
        with open(current_time + REGIONS_OUTSIDE_PRECISE_ZONE_LIST_FILE, "w") as file:
            file.write("List of files whose regions were outside of the precise zone (it's ok):\n")
            for item in regions_outside_prec_area_files:
                file.write(item + "\n")

    # save missing image files (present in json, absent in dir)
    if len(missing_dataset_images_files) > 0:
        with open(current_time + MISSING_DATASET_IMAGES_LIST_FILE, "w") as file:
            file.write("These images are present in VIA's json project but absent in given images directory (images are required for conversion to YOLO format):\n")
            for item in missing_dataset_images_files:
                file.write(item + "\n")

    # show report
    print("Processed", img_processed, "images")
    print("Processed", regions_processed, "regions")
    print("Shifted", regions_shifted, "regions")
    print("Resulting json is saved as", OUTPUT_JSON_FILE)

    conversion_failed = False

    if unsupported_regions_count > 0:
        conversion_failed = True
        print("Passed", unsupported_regions_count, "regions due to unsupported shape. See'", UNSUPPORTED_REGIONS_LIST_FILE, "'file for details.")

    if regions_outside_prec_area_count > 0:
        print("Removed", regions_outside_prec_area_count, "regions which were outside of precise area (it's ok). See'", REGIONS_OUTSIDE_PRECISE_ZONE_LIST_FILE, "'file for details.")

    if missing_dataset_images_count > 0:
        conversion_failed = True
        print("Missing", missing_dataset_images_count, "images in given images directory. See'", MISSING_DATASET_IMAGES_LIST_FILE, "'file for details.")

    if conversion_failed:
        print("Total: conversion is NOT successful! See report files mentioned above for details.")
    else:
        print("Total: conversion is successful!")


if __name__ == '__main__':
    main()
