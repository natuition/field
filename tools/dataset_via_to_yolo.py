"""
This script is loading via's json, converts regions to yolo v4 format (also v3 if they has same structure)
and saves results.
Input: json file, classes file
Output: txt files with regions data for each image in json (placed into the directory), report txt files if something gone wrong
"""

import json
import datetime
import os
import cv2 as cv

# paths
INPUT_JSON_FILE_PATH = "1 result.json"
INPUT_CLASSES_FILE_PATH = "../yolo/v3_precise.names"  # should be exactly same file as one which will be used for learning (types order in this file is strictly valuable, and can't be traced to warn user, so make sure you using same file here and at the NN learning)
INPUT_DATASET_IMAGES_DIR = "images/"
OUTPUT_YOLO_TXT_DIR = "yolo txt regions/"
UNSUPPORTED_REGIONS_LIST_FILE = "unsupported regions list.txt"
MISSING_DATASET_IMAGES_LIST_FILE = "missing images list.txt"
MISSING_JSON_REGION_TYPES_LIST_FILE = "missing in json types file list.txt"
MISSING_CLASSES_LIST_FILE = "missing classes list.txt"


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


def load_class_names(labels_file):
    with open(labels_file, 'rt') as f:
        return f.read().rstrip('\n').split('\n')


def get_current_time():
    """Returns formatted string with current time (YYYY-MM-DD HH-MM-SS)"""

    date = str(datetime.datetime.now())
    return date[:date.rindex(".")].replace(":", "-")


def main():
    classes = load_class_names(INPUT_CLASSES_FILE_PATH)
    create_directories(OUTPUT_YOLO_TXT_DIR)

    img_processed = 0
    regions_processed = 0
    regions_converted = 0

    unsupported_regions_count = 0
    unsupported_regions_files = set()

    missing_dataset_images_count = 0
    missing_dataset_images_files = set()

    missing_json_region_types_count = 0
    missing_json_region_types_files = set()

    missing_classes_count = 0
    missing_classes = set()

    # load json
    with open(INPUT_JSON_FILE_PATH, "r") as file:
        data = json.loads(file.read())

    # process regions
    for file_key in data["_via_img_metadata"]:
        file_name = data["_via_img_metadata"][file_key]["filename"]
        file_path = INPUT_DATASET_IMAGES_DIR + file_name

        # check if jpg file exists
        if os.path.isfile(file_path):
            img = cv.imread(file_path)
            img_x_size, img_y_size = img.shape[1], img.shape[0]
            img_processed += 1

            for region in data["_via_img_metadata"][file_key]["regions"]:
                regions_processed += 1

                if region["shape_attributes"]["name"] == "rect":
                    x_label = "x"
                    y_label = "y"

                    # region center
                    rcx = region["shape_attributes"][x_label] + int(region["shape_attributes"]["width"] / 2)
                    rcy = region["shape_attributes"][y_label] + int(region["shape_attributes"]["height"] / 2)
                    """
                elif region["shape_attributes"]["name"] == "circle":
                    x_label = "cx"
                    y_label = "cy"

                    # region center
                    rcx = region["shape_attributes"][x_label]
                    rcy = region["shape_attributes"][y_label]
                """
                else:
                    """
                    print("Unsupported region shape '", region["shape_attributes"]["name"],
                          "', THIS REGION WON'T BE PROCESSED! Image name was saved in file ",
                          MISSING_REGIONS_FILE, sep="")
                    """
                    unsupported_regions_count += 1
                    unsupported_regions_files.add(file_name)
                    continue

                # check if region type is present
                if "type" not in region["region_attributes"]:
                    missing_json_region_types_count += 1
                    missing_json_region_types_files.add(file_name)
                    continue

                region_type = region["region_attributes"]["type"]

                # check if type is present in yolo names file
                if region_type not in classes:
                    """
                    print("Type", region_type, "is not in", INPUT_CLASSES_FILE_PATH, "; please make sure you added to",
                          "this file all class types that are used in input VIA's json file, remove all previous",
                          "results and run this script from the scratch.")
                    """
                    missing_classes_count += 1
                    missing_classes.add(region_type)
                    continue

                # convert region
                obj_class = classes.index(region_type)
                x_center = rcx / img_x_size
                y_center = rcy / img_y_size
                width = region["shape_attributes"]["width"] / img_x_size
                height = region["shape_attributes"]["height"] / img_y_size

                # save region
                with open(OUTPUT_YOLO_TXT_DIR + file_name[:-4] + ".txt", "w") as txt_file:
                    # <object-class> <x_center> <y_center> <width> <height>
                    record = str(obj_class) + " " + str(x_center) + " " + str(y_center) + " " + str(width) + " " + \
                             str(height) + "\n"
                    txt_file.write(record)

                regions_converted += 1
        else:
            missing_dataset_images_count += 1
            missing_dataset_images_files.add(file_name)

    current_time = get_current_time() + " "

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

    if conversion_failed:
        print("Total: conversion is FAILED! See report files mentioned above for details.")
    else:
        print("Total: conversion is SUCCESSFUL!")


if __name__ == '__main__':
    main()
