"""Parses given json file and saves working zone, crop settings and scene center info in txt"""

import json

INPUT_FILE = "settings.json"
OUTPUT_FILE = "parsed_settings.txt"


def process_working_zone(region: dict):
    points_x = region["shape_attributes"]["all_points_x"]
    points_y = region["shape_attributes"]["all_points_y"]
    res = []
    for el in zip(points_x, points_y):
        res.append(list(el))
    return res


def adapt_working_zone(working_zone: list, crop_w_from, crop_h_from):
    adapted_working_zone = []
    for i in range(len(working_zone)):
        adapted_working_zone.append([working_zone[i][0] - crop_w_from, working_zone[i][1] - crop_h_from])
    return adapted_working_zone


def process_scene_center(region: dict):
    return region["shape_attributes"]["cx"], region["shape_attributes"]["cy"]


def adapt_scene_center(scene_center_x, scene_center_y, crop_w_from, crop_h_from):
    return scene_center_x - crop_w_from, scene_center_y - crop_h_from


def process_crop_values(region: dict):
    crop_w_from = int(region["shape_attributes"]["x"])
    crop_w_to = int(crop_w_from + region["shape_attributes"]["width"])
    crop_h_from = int(region["shape_attributes"]["y"])
    crop_h_to = int(crop_h_from + region["shape_attributes"]["height"])
    return crop_w_from, crop_w_to, crop_h_from, crop_h_to


def main():
    # load json
    with open(INPUT_FILE) as file:
        data = json.loads(file.read())["_via_img_metadata"]

    # loop over data and get settings for uncropped image
    for file_key in data:
        regions = data[file_key]["regions"]
        for region in regions:
            if region["shape_attributes"]["name"] == "polygon":
                working_zone = process_working_zone(region)
            elif region["shape_attributes"]["name"] == "rect":
                crop_w_from, crop_w_to, crop_h_from, crop_h_to = process_crop_values(region)
            elif region["shape_attributes"]["name"] == "circle":
                scene_center_x, scene_center_y = process_scene_center(region)
            else:
                print("Unsupported type:", region["shape_attributes"]["name"])
        break

    # adapt settings according image crop values
    adapted_working_zone = adapt_working_zone(working_zone, crop_w_from, crop_h_from)
    adapted_scene_c_x, adapted_scene_c_y = adapt_scene_center(scene_center_x, scene_center_y, crop_w_from, crop_h_from)

    # save settings to the txt file
    with open(OUTPUT_FILE, "w") as file:
        # scene center
        file.write("# SCENE CENTER\n")
        file.write("SCENE_CENTER_X = " + str(adapted_scene_c_x) + "  # " + str(scene_center_x) + " for uncropped\n")
        file.write("SCENE_CENTER_Y = " + str(adapted_scene_c_y) + "  # " + str(scene_center_y) + " for uncropped\n")

        # image cropping info
        file.write("\n# CROPPING\n")
        file.write("CROP_W_FROM = " + str(crop_w_from) + "\n")
        file.write("CROP_W_TO = " + str(crop_w_to) + "\n")
        file.write("CROP_H_FROM = " + str(crop_h_from) + "\n")
        file.write("CROP_H_TO = " + str(crop_h_to) + "\n")

        # working zones
        file.write("\n# WORKING ZONE FOR CROPPED (this one is for config.py)\n")
        file.write("WORKING_ZONE_POLY_POINTS = " + str(adapted_working_zone) + "\n")
        # file.write("\n# WORKING ZONE FOR UNCROPPED (may be useful for debug, DO NOT add this to config.py!)\n")
        # file.write("WORKING_ZONE_POLY_POINTS = " + str(working_zone) + "\n")


if __name__ == '__main__':
    main()
