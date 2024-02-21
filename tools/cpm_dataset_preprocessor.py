"""Control points model dataset preparation script

Adds millimeter distances to marked up points contained in a given json and saves it as separate prepared dataset.

INPUT: VIA json file with control points (only positive X and Y), scene center px coordinates.
Each point has VIA "point region" format. Points are placed in sequence order:
X from center to the right, Y from center to the top:

7, 8, 9.
4, 5, 6,
1, 2, 3,

OUTPUT: csv file, each row is a prepared control point
"""
import sys
sys.path.append('../')

import json
import math
from config import config

# PATHS
INPUT_JSON_PATH = "./"+config.ROBOT_SN+"_cp.json"
OUTPUT_CSV_PATH = "../"+config.CONTROL_POINTS_CSV_PATH

# SETTINGS
ROW_PTS_CNT = 5  # how much points in a single X row
SC_X_OFFSET_MM = 0  # static offset applied to each control point's mm value
SC_Y_OFFSET_MM = 0  # static offset applied to each control point's mm value
X_STEP_MM = 50  # control point mm step, this will be added to each next CP during current row
Y_STEP_MM = 50  # control point mm step, this will be added to each next CP during current row
X_INIT_MM = 0  # control point mm initial value, this will set as CP's mm value for each CP in a new row
Y_INIT_MM = 0  # control point mm initial value, this will set as CP's mm value once at the start


def main():
    # load points
    with open(INPUT_JSON_PATH) as pts_file:
        js_pts = json.loads(pts_file.read())["_via_img_metadata"]

    # extract points and add mm distances
    prepared_pts = []  # format is (x coord, y coord, x center diff, y center diff, center dist, x mm, y mm)
    pt_no = 1  # current point counter for proper mm values handling
    cur_x_mm = X_INIT_MM  # current point x mm
    cur_y_mm = Y_INIT_MM  # current point y mm
    for file_key in js_pts:
        for point in js_pts[file_key]["regions"]:
            pt_x_px = point["shape_attributes"]["cx"]
            pt_y_px = point["shape_attributes"]["cy"]
            pt_x_px_diff = pt_x_px - config.SCENE_CENTER_X
            pt_y_px_diff = pt_y_px - config.SCENE_CENTER_Y
            pt_dist_from_sc_px = round(math.sqrt(pt_x_px_diff ** 2 + pt_y_px_diff ** 2), 3)
            prepared_pts.append((
                pt_x_px,
                pt_y_px,
                abs(pt_x_px_diff),
                abs(pt_y_px_diff),
                pt_dist_from_sc_px,
                cur_x_mm + SC_X_OFFSET_MM,
                cur_y_mm + SC_Y_OFFSET_MM
            ))

            pt_no += 1
            if pt_no > ROW_PTS_CNT:
                pt_no = 1
                cur_y_mm += Y_STEP_MM
                cur_x_mm = X_INIT_MM
            else:
                cur_x_mm += X_STEP_MM
        break  # process only first image file in given VIA json

    # save prepared data as txt
    with open(OUTPUT_CSV_PATH, "w") as output_csv:
        output_csv.write("X_px,Y_px,X_diff_px,Y_diff_px,Distance_px,X_mm,Y_mm\n")
        for item in prepared_pts:
            line = str(item).replace("(", "").replace(")", "").replace(" ", "") + "\n"
            output_csv.write(line)

    print("Done!")


if __name__ == '__main__':
    main()
