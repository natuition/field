"""
Loads given json and images, saves each region as separated image and corresponding regions txt (YOLO format)
"""

from matplotlib.patches import Polygon
import cv2 as cv
import json
import os
import utility
import detection

# SETTINGS
# paths
"""
# DATASET 7
INPUT_JSON_FILE = "../input/dataset5_7_periphery/dataset5_7_periphery.json"
INPUT_CLASSES_FILE_PATH = "../yolo/new_periphery (ds7).names"  # should be exactly same file as one which will be used for learning (types order in this file is strictly valuable, and can't be traced to warn user, so make sure you using same file here and at the NN learning)
INPUT_DATASET_IMAGES_DIR = "../input/dataset5_7_periphery/"
OUTPUT_DATA_DIR = "../input/dataset5_7_periphery/"
"""
# DATASET 2
INPUT_JSON_FILE = "../input/dataset2/Natuition_dataset16.json"
INPUT_CLASSES_FILE_PATH = "../yolo/new_periphery (ds2).names"  # should be exactly same file as one which will be used for learning (types order in this file is strictly valuable, and can't be traced to warn user, so make sure you using same file here and at the NN learning)
INPUT_DATASET_IMAGES_DIR = "../input/dataset2/"
OUTPUT_DATA_DIR = "../input/dataset2/"

# REGIONS SETTINGS
"""
REGION_TYPE_KEY = "plant type"  # ds 7
"""
REGION_TYPE_KEY = "type"  # ds 2

# PROCESSING SETTINGS
# process only regions which are in the given zone if set to True, process all regions if set to False
"""
APPLY_PRECISE_ZONE = True  # ds 7
SCENE_CENTER_X = 1200
SCENE_CENTER_Y = 1170
# precise zone sizes from scene center point
PRECISE_ZONE_LEFT = 550
PRECISE_ZONE_RIGHT = 550
PRECISE_ZONE_TOP = 550
PRECISE_ZONE_BOTTOM = 50
"""
APPLY_PRECISE_ZONE = True  # ds 2; True for precise, False otherwise
SCENE_CENTER_X = 1297  # old nano 1135; rpi 1297
SCENE_CENTER_Y = 1153  # old nano 1565; rpi 1153
# precise zone sizes from scene center point
PRECISE_ZONE_LEFT = 530  # old nano 595; rpi 530
PRECISE_ZONE_RIGHT = 530  # old nano 595; rpi 530
PRECISE_ZONE_TOP = 540  # old nano 900; rpi 540
PRECISE_ZONE_BOTTOM = 0

# process only regions from list below if set to True, process all regions if set to False
# APPLY_REGIONS_WHITE_LIST = False  # ds 7
# REGIONS_WHITE_LIST = ["Daisy", "Dandelion"]  # ds 7
APPLY_REGIONS_WHITE_LIST = True  # ds 2
REGIONS_WHITE_LIST = ["daisy", "dandelion"]  # ds 2

# will reduce regions sizes by given multiplier if set to True, will remain their initial sizes if set to False
"""
APPLY_REGIONS_SIZE_REDUCING = False  # ds 7
REGIONS_SIZE_REDUCING_PERCENT = 0.15  # ds7; 0.15 means that reduced region will be 85% of the source region size
"""
APPLY_REGIONS_SIZE_REDUCING = True  # ds 2
REGIONS_SIZE_REDUCING_PERCENT = 0.15  # ds 2; 0.15 means that reduced region will be 85% of the source region size

# script inner paths (don't change that)
UNSUPPORTED_REGIONS_LIST_FILE = "unsupported regions list.txt"
MISSING_DATASET_IMAGES_LIST_FILE = "missing images list.txt"
MISSING_JSON_REGION_TYPES_LIST_FILE = "missing in json types file list.txt"
MISSING_CLASSES_LIST_FILE = "missing classes list.txt"
REGIONS_OUTSIDE_PRECISE_ZONE_LIST_FILE = "outside precise zone files.txt"


def main():
    # check input dirs
    if not os.path.isfile(INPUT_JSON_FILE):
        print("Couldn't find json file:", INPUT_JSON_FILE)
        exit(0)
    if not os.path.isfile(INPUT_CLASSES_FILE_PATH):
        print("Couldn't find classes file:", INPUT_CLASSES_FILE_PATH)
        exit(0)
    if not os.path.exists(INPUT_DATASET_IMAGES_DIR):
        print("Couldn't find images directory:", INPUT_DATASET_IMAGES_DIR)
        exit(0)

    # define and create output dirs
    output_jpg_dir = OUTPUT_DATA_DIR + "jpg/"
    output_txt_dir = OUTPUT_DATA_DIR + "txt/"
    utility.create_directories(OUTPUT_DATA_DIR, output_jpg_dir, output_txt_dir)

    # statistics and report data
    img_processed = 0
    regions_processed = 0
    regions_converted = 0
    unsupported_regions_count = 0
    unsupported_regions_files = set()
    regions_outside_prec_area_count = 0
    regions_outside_prec_area_files = set()
    missing_dataset_images_count = 0
    missing_dataset_images_files = set()
    missing_json_region_types_count = 0
    missing_json_region_types_files = set()
    missing_classes_count = 0
    missing_classes = set()

    w_from = SCENE_CENTER_X - PRECISE_ZONE_LEFT
    w_to = SCENE_CENTER_X + PRECISE_ZONE_RIGHT
    h_from = SCENE_CENTER_Y - PRECISE_ZONE_TOP
    h_to = SCENE_CENTER_Y + PRECISE_ZONE_BOTTOM
    precise_zone = Polygon([[w_from, h_from], [w_to, h_from], [w_to, h_to], [w_from, h_to]])

    output_jpg_dir = OUTPUT_DATA_DIR + "jpg/"

    # load json
    with open(INPUT_JSON_FILE, "r") as file:
        data = json.loads(file.read())

    # load classes
    classes = detection.load_class_names(INPUT_CLASSES_FILE_PATH)

    # loop over images list in json
    cur_file_no = 1
    for file_key in data["_via_img_metadata"]:
        if cur_file_no % 10 == 0:
            print("Processed", cur_file_no, "of", len(data["_via_img_metadata"]), "files.")
        cur_file_no += 1

        file_name = data["_via_img_metadata"][file_key]["filename"]
        file_path = INPUT_DATASET_IMAGES_DIR + file_name

        # check if image jpg file exists and is not empty
        if os.path.isfile(file_path) and os.stat(file_path).st_size > 0:
            img = cv.imread(file_path)
            img_processed += 1
        else:
            missing_dataset_images_count += 1
            missing_dataset_images_files.add(file_name)
            continue

        reg_img_file_counter = 1

        # loop over regions of current image file
        for region in data["_via_img_metadata"][file_key]["regions"]:
            regions_processed += 1

            # check if region type is present
            if REGION_TYPE_KEY not in region["region_attributes"]:
                missing_json_region_types_count += 1
                missing_json_region_types_files.add(file_name)
                continue
            region_type = region["region_attributes"][REGION_TYPE_KEY]

            # regions whitelist check
            if APPLY_REGIONS_WHITE_LIST and region_type not in REGIONS_WHITE_LIST:
                continue

            # check if type is present in yolo names file
            if region_type not in classes:
                missing_classes_count += 1
                missing_classes.add(region_type)
                continue

            # get region info and convert to rect if needed
            if region["shape_attributes"]["name"] == "rect":
                crop_reg_left = int(region["shape_attributes"]["x"])
                crop_reg_right = int(region["shape_attributes"]["x"] + region["shape_attributes"]["width"])
                crop_reg_top = int(region["shape_attributes"]["y"])
                crop_reg_bottom = int(region["shape_attributes"]["y"] + region["shape_attributes"]["height"])
                reg_center_x = int(region["shape_attributes"]["x"] + region["shape_attributes"]["width"] / 2)
                reg_center_y = int(region["shape_attributes"]["y"] + region["shape_attributes"]["height"] / 2)
            elif region["shape_attributes"]["name"] == "circle":
                crop_reg_left = int(region["shape_attributes"]["cx"] - region["shape_attributes"]["r"])
                crop_reg_right = int(region["shape_attributes"]["cx"] + region["shape_attributes"]["r"])
                crop_reg_top = int(region["shape_attributes"]["cy"] - region["shape_attributes"]["r"])
                crop_reg_bottom = int(region["shape_attributes"]["cy"] + region["shape_attributes"]["r"])
                reg_center_x = int(region["shape_attributes"]["cx"])
                reg_center_y = int(region["shape_attributes"]["cy"])
            else:
                # print("Unsupported region shape '", region["shape_attributes"]["name"], "', THIS REGION WON'T BE SHIFTED. See info in file ", MISSED_REGIONS_FILE, sep="")
                unsupported_regions_count += 1
                unsupported_regions_files.add(file_name)
                continue

            # precise zone check
            if APPLY_PRECISE_ZONE and not precise_zone.contains_point([reg_center_x, reg_center_y]):
                regions_outside_prec_area_count += 1
                regions_outside_prec_area_files.add(file_name)
                continue

            # reduce region sizes if allowed
            if APPLY_REGIONS_SIZE_REDUCING:
                yolo_reg_left = crop_reg_left + int(crop_reg_left * REGIONS_SIZE_REDUCING_PERCENT / 2)
                yolo_reg_right = crop_reg_right - int(crop_reg_right * REGIONS_SIZE_REDUCING_PERCENT / 2)
                yolo_reg_top = crop_reg_top + int(crop_reg_top * REGIONS_SIZE_REDUCING_PERCENT / 2)
                yolo_reg_bottom = crop_reg_bottom - int(crop_reg_bottom * REGIONS_SIZE_REDUCING_PERCENT / 2)
            else:
                yolo_reg_left = crop_reg_left
                yolo_reg_right = crop_reg_right
                yolo_reg_top = crop_reg_top
                yolo_reg_bottom = crop_reg_bottom

            # crop and save image region
            reg_img = img[crop_reg_top:crop_reg_bottom, crop_reg_left:crop_reg_right]
            reg_img_name = file_name[:-4] + " " + str(reg_img_file_counter) + file_name[-4:]
            cv.imwrite(output_jpg_dir + reg_img_name, reg_img)

            # convert region to yolo format
            reg_center_x = int((yolo_reg_right - yolo_reg_left) / 2)  # TODO: a bug - not int not none happens here
            reg_center_y = int((yolo_reg_bottom - yolo_reg_top) / 2)
            reg_width = yolo_reg_right - yolo_reg_left
            reg_height = yolo_reg_bottom - yolo_reg_top
            img_x_size = crop_reg_right - crop_reg_left
            img_y_size = crop_reg_bottom - crop_reg_top

            obj_class = classes.index(region_type)
            x_center = round(reg_center_x / img_x_size, 6)
            y_center = round(reg_center_y / img_y_size, 6)
            width = round(reg_width / img_x_size, 6)
            height = round(reg_height / img_y_size, 6)

            # <object-class> <x_center> <y_center> <width> <height>
            record_line = str(obj_class) + " " + str(x_center) + " " + str(y_center) + " " + str(width) + " " + \
                          str(height) + "\n"

            # save converted to yolo region into txt
            with open(output_txt_dir + file_name[:-4] + " " + str(reg_img_file_counter) + ".txt", "w") as txt_file:
                txt_file.write(record_line)

            reg_img_file_counter += 1
            regions_converted += 1

    current_time = utility.get_current_time() + " "

    # save passed unsupported regions info
    if len(unsupported_regions_files) > 0:
        with open(current_time + UNSUPPORTED_REGIONS_LIST_FILE, "w") as file:
            file.write("These images are present in VIA's json project but their regions types are not supported:\n")
            for item in unsupported_regions_files:
                file.write(item + "\n")

    # save missing image files (present in json, absent in dir)
    if len(missing_dataset_images_files) > 0:
        with open(current_time + MISSING_DATASET_IMAGES_LIST_FILE, "w") as file:
            file.write("These images are present in VIA's json project but absent in given images directory (images are required for conversion to YOLO format):\n")
            for item in missing_dataset_images_files:
                file.write(item + "\n")

    # save missing in json region types
    if len(missing_json_region_types_files) > 0:
        with open(current_time + MISSING_JSON_REGION_TYPES_LIST_FILE, "w") as file:
            file.write("These images are added to VIA's json project but their regions have no information about region types:\n")
            for item in missing_json_region_types_files:
                file.write(item + "\n")

    # save missing classes in classes file
    if len(missing_classes) > 0:
        with open(current_time + MISSING_CLASSES_LIST_FILE, "w") as file:
            file.write("These classes are present in given json, but absent in " + INPUT_CLASSES_FILE_PATH + " classes file:\n")
            for item in missing_classes:
                file.write(item + "\n")

    # save list of files with regions outside the precise area
    if len(regions_outside_prec_area_files) > 0:
        with open(current_time + REGIONS_OUTSIDE_PRECISE_ZONE_LIST_FILE, "w") as file:
            file.write("List of files whose regions were outside of the precise zone (it's ok):\n")
            for item in regions_outside_prec_area_files:
                file.write(item + "\n")

    # show report
    print("Processed", img_processed, "images")
    print("Processed", regions_processed, "regions")
    print("Successfully converted", regions_converted, "regions")

    conversion_failed = False

    if unsupported_regions_count > 0:
        conversion_failed = True
        print("Passed", unsupported_regions_count, "regions due to unsupported shape. See'", UNSUPPORTED_REGIONS_LIST_FILE, "'file for details.")

    if missing_dataset_images_count > 0:
        conversion_failed = True
        print("Missing", missing_dataset_images_count, "images in given images directory. See'", MISSING_DATASET_IMAGES_LIST_FILE, "'file for details.")

    if missing_json_region_types_count > 0:
        conversion_failed = True
        print("Missing", missing_json_region_types_count, "regions types in given json file. See'", MISSING_JSON_REGION_TYPES_LIST_FILE, "'file for details.")

    if missing_classes_count > 0:
        conversion_failed = True
        print("Missing", missing_classes_count, "classes count in given classes file. See'", MISSING_CLASSES_LIST_FILE, "'file for details.")

    if regions_outside_prec_area_count > 0:
        print("Passed", regions_outside_prec_area_count, "regions which were outside of precise area (it's ok). See'", REGIONS_OUTSIDE_PRECISE_ZONE_LIST_FILE, "'file for details.")

    if conversion_failed:
        print("Total: conversion is FAILED! See report files mentioned above for details.")
    else:
        print("Total: conversion is SUCCESSFUL!")


if __name__ == '__main__':
    main()
