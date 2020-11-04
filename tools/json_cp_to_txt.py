"""
Parses given json file and saves control points info in txt

REQUIRES!!! to set CP settings below (for each new json or changes) and right values in these keys in the config.py:
config.CROP_H_FROM
config.CROP_W_FROM
"""

import json
from config import config

# PATHS AND FILES
INPUT_FILE = "CP.json"
OUTPUT_FILE = "parsed_cp.txt"

# STARTING INFO (MUST BE SET FOR EACH JSON SEPARATELY AS POINTS FOR EACH JSON ARE LIKELY DIFFERENT AND POINTS CORRECT COMPUTING DIRECTLY DEPENDS ON THIS!)
# POINTS SHOULD BE MARKED CONSISTENTLY: START FROM CENTER COL (point 1, must start from 1) AND FILL ALL LEFT SIDE, THEN RIGHT SIDE:
# 9 8 7 14 15
# 6 5 4 12 13
# 3 2 1 10 11

# origin point is the first point, the closest one to the scene center, points building starts on this
# format is [px_x, px_y, mm_x, mm_y, point_number]
# point numbers MUST start from "1" (num 1 is the origin point)
ORIGIN_CONTROL_POINT = [1701, 1454, -3, 4, 1]
# how many points in one row. row is from center points column (it starts from origin point (counts) and goes up) to the left or right edge
# PAY ATTENTION: ALL MARKUP WITH CP SHOULD HAVE SIMILAR ROWS SIZES! This means if you have 11 points on the left side (from the center column) - you have to mark up strictly 11 points on the right side (from the center column), column is counting
# for example above row cnt will be = 3 (1, 2, 3 and 1, 10, 11)
POINTS_IN_ROW_COUNT = 11
# last point on the left side number, for example above it will be 9, as 9th point is last point that was marked on the left side
LAST_LEFT_SIDE_POINT_NUMBER = 110
CONTROL_POINT_MM_X_STEP = 40
CONTROL_POINT_MM_Y_STEP = 40


def adapt_control_points(control_points: list):
    adapted_control_points = []
    for point in control_points:
        px_x, px_y, mm_x, mm_y, number = point
        px_x -= config.CROP_W_FROM
        px_y -= config.CROP_H_FROM
        adapted_control_points.append([px_x, px_y, mm_x, mm_y, number])
    return adapted_control_points


def main():
    global POINTS_IN_ROW_COUNT

    # load json
    with open(INPUT_FILE) as file:
        data = json.loads(file.read())["_via_img_metadata"]

    control_points = []

    # loop over data and get settings for uncropped image
    for file_key in data:
        regions = data[file_key]["regions"]
        _, _, cur_mm_x, cur_mm_y, cur_num = ORIGIN_CONTROL_POINT
        origin_mm_x, origin_mm_y = cur_mm_x, cur_mm_y

        for region in regions:
            # add first point directly (as it is specified in settings)
            if cur_num == 1:
                control_points.append(ORIGIN_CONTROL_POINT)
                cur_mm_x -= CONTROL_POINT_MM_X_STEP
                cur_num += 1
                continue

            # if it is a point - extract and compute values
            if region["shape_attributes"]["name"] == "point":
                px_x = region["shape_attributes"]["cx"]
                px_y = region["shape_attributes"]["cy"]

                control_points.append([px_x, px_y, cur_mm_x, cur_mm_y, cur_num])

                # left side
                if cur_num < LAST_LEFT_SIDE_POINT_NUMBER:
                    # current row
                    if cur_num % POINTS_IN_ROW_COUNT != 0:
                        cur_mm_x -= CONTROL_POINT_MM_X_STEP
                    # transition to the next row
                    else:
                        cur_mm_x = origin_mm_x
                        cur_mm_y += CONTROL_POINT_MM_Y_STEP
                # right side
                elif cur_num > LAST_LEFT_SIDE_POINT_NUMBER:
                    # current row
                    if cur_num % POINTS_IN_ROW_COUNT != 0:
                        cur_mm_x += CONTROL_POINT_MM_X_STEP
                    # transition to the next row
                    else:
                        cur_mm_x = origin_mm_x + CONTROL_POINT_MM_X_STEP  # row count is reduced by 1 and x mm coord is increased as center points col. is already included
                        cur_mm_y += CONTROL_POINT_MM_Y_STEP
                # last point of the left side
                else:
                    cur_mm_x = origin_mm_x + CONTROL_POINT_MM_X_STEP
                    cur_mm_y = origin_mm_y
                    POINTS_IN_ROW_COUNT -= 1  # as center col. is already in CP list

                cur_num += 1
            else:
                print("Unsupported type:", region["shape_attributes"]["name"])
        break

    # adapt control points according to image crop values
    adapted_control_points = adapt_control_points(control_points)

    print("Adapted CP count:", len(adapted_control_points))
    print("Original CP count:", len(control_points))

    # save settings to the txt file
    with open(OUTPUT_FILE, "w") as file:
        # control points for cropped
        file.write("# CONTROL POINTS FOR CROPPED (this one is for config.py)\n")
        file.write("IMAGE_CONTROL_POINTS_MAP = " + str(adapted_control_points) + "\n")

        # control points for uncropped
        file.write("\n# CONTROL POINTS FOR UNCROPPED (may be useful for debug, DO NOT add this to config.py! It's not for that)\n")
        file.write("IMAGE_CONTROL_POINTS_MAP = " + str(control_points) + "\n")


if __name__ == '__main__':
    main()
