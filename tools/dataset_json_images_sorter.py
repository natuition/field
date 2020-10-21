"""
This script loads images list json file and copy listed images from dataset into rest scripts input dir
"""

import json
import shutil
import glob
import platform
import os

JSON_FILE_PATH = "input/dataset5 raw 1 sorted/dataset5_v23.json"  # json file with images annotations which will be used as list of images to copy
COPY_FROM_PATH = "input/dataset5 raw 1 sorted/"  # dir where images are copied from
COPY_TO_PATH = "input/dataset5 raw 1 sorted filtered/"  # dir where images will be copied to
EXCLUDED_IMAGES_LIST = "input/dataset5 raw 1 sorted filtered/excluded images list.txt"


def get_slash():
    return "\\" if platform.system() == "Windows" else "/"


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
    create_directories(COPY_TO_PATH)
    excluded_images_names = set()

    # extract list of file paths to copy
    with open(JSON_FILE_PATH, "r") as file:
        files_to_copy = []
        data = json.loads(file.read())["_via_img_metadata"]
        for key in data:
            files_to_copy.append(data[key]["filename"])

    copied = 0
    all_images_paths = glob.glob(COPY_FROM_PATH + "*.jpg")
    for image_path in all_images_paths:
        file_name = image_path.split(get_slash())[-1]
        if file_name in files_to_copy:
            shutil.copyfile(image_path, COPY_TO_PATH + file_name)
            copied += 1
        else:
            excluded_images_names.add(file_name)

    # save excluded images list
    if len(excluded_images_names) > 0:
        with open(EXCLUDED_IMAGES_LIST, "w") as file:
            file.write("These files were excluded (they are not present in given json):\n")
            for item in excluded_images_names:
                file.write(item + "\n")

    print("Found", len(all_images_paths), "images")
    print("Copied", copied, "images.")
    if len(excluded_images_names) > 0:
        print("Excluded", len(excluded_images_names), "images (not in the given json). See details in", EXCLUDED_IMAGES_LIST, "file.")
    print("Done.")


if __name__ == '__main__':
    main()
