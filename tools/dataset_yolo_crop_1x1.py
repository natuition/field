"""
Crops images in given directory to 1x1 size.
Takes .names file, and dataset in yolo format (set of .jpg-s and corresponding regions in .txt-s files) as input.
Crops each image by its H or W (lesser is used) making it square, also updates regions in corresponding txt.
Saves this as output (original dataset is NOT affected).
Also generates report txt files which contains info about empty/missing txt-s, or files, which regions were affected by
cropping (outside new borders) lists.

Requires utility.py (from Natuition) and opencv installed (tested on OpenCV 4.2.0).
"""

import cv2 as cv
import glob
import os
import utility
from matplotlib.patches import Polygon
import shutil

# SETTINGS
INPUT_DIR = "input/"  # dataset input path, should end with / if not script root dir
OUTPUT_DIR = "output/"  # dataset input path, should end with / if not script root dir
OUTPUT_EMPTY_DIR = "with_empty_txt/"  # dir to put images for which txts are empty
OUTPUT_DAMAGED_DIR = "with_damaged_txt/"  # dir to put images for which txts are not ok
OUTPUT_REGS_OUTSIDE_DIR = "with_regs_outside/"  # dir to put images which regions are outside borders of cropped image
IMAGES_EXTENSIONS = [".jpg"]  # example for multiple: [".jpg", ".jpeg", ".png"] and so on
PROGRESS_PRINT_RATE = 100  # print progress message each 100 images


def load_regions(txt_file_path: str):
    """Loads txt file by given path, parses this file and returns list of regions in yolo format"""

    regions = []
    with open(txt_file_path) as file:
        for line in file:
            line = line.rstrip("\n")
            if line != "":
                regions.append(line)
    return regions


def main():
    ok_txts_cnt = 0
    empty_txts_cnt = 0
    missed_txts_cnt = 0
    damaged_txts_cnt = 0
    regions_outside_cnt = 0  # total regions outside
    regions_outside_files_cnt = 0  # total files with regions outside

    # create output dir; nested dirs should be passed sequentially, each as separate parameter
    utility.create_directories(OUTPUT_DIR, OUTPUT_EMPTY_DIR, OUTPUT_DAMAGED_DIR, OUTPUT_REGS_OUTSIDE_DIR)
    slash = utility.get_path_slash()

    # create images list
    img_files_paths = []
    for extension in IMAGES_EXTENSIONS:
        img_files_paths += glob.glob(INPUT_DIR + "*" + extension)

    # check if there any images
    if len(img_files_paths) == 0:
        input(INPUT_DIR + " contains no files (images) with specified in settings extensions. Press enter to exit:")
        exit(0)

    print("Current progress print frequency is 1 per", PROGRESS_PRINT_RATE, "image(s).")
    print("Starting processing", len(img_files_paths), "images...")

    # do cropping and regions conversion for each image file
    for cur_file_no, in_img_path in enumerate(img_files_paths, start=1):
        # load image and do some pre calculations
        in_img = cv.imread(in_img_path)
        in_img_w, in_img_h = in_img.shape[1], in_img.shape[0]

        # make paths
        in_txt_path = in_img_path[:in_img_path.rfind(".")] + ".txt"

        out_img_path_ok = OUTPUT_DIR + in_img_path.split(slash)[-1]
        out_txt_path_ok = OUTPUT_DIR + in_txt_path.split(slash)[-1]

        out_img_path_empty = OUTPUT_EMPTY_DIR + in_img_path.split(slash)[-1]
        out_txt_path_empty = OUTPUT_EMPTY_DIR + in_txt_path.split(slash)[-1]

        out_img_path_damaged = OUTPUT_DAMAGED_DIR + in_img_path.split(slash)[-1]
        out_txt_path_damaged = OUTPUT_DAMAGED_DIR + in_txt_path.split(slash)[-1]

        out_img_path_reg_out = OUTPUT_REGS_OUTSIDE_DIR + in_img_path.split(slash)[-1]
        out_txt_path_reg_out = OUTPUT_REGS_OUTSIDE_DIR + in_txt_path.split(slash)[-1]

        # copy img and txt to output dir if img H==W
        if in_img_w == in_img_h:
            # check for txt availability
            if os.path.isfile(in_txt_path):
                # check for txt emptiness
                if len(load_regions(in_txt_path)) == 0:
                    empty_txts_cnt += 1
                    shutil.copyfile(in_img_path, out_img_path_empty)
                    shutil.copyfile(in_txt_path, out_txt_path_empty)
                # TODO: a bug: file is passed as ok if it is damaged (regions are present but len of any region is != 5)
                else:
                    ok_txts_cnt += 1
                    shutil.copyfile(in_img_path, out_img_path_ok)
                    shutil.copyfile(in_txt_path, out_txt_path_ok)
            else:
                missed_txts_cnt += 1
                shutil.copyfile(in_img_path, out_img_path_empty)
                with open(out_txt_path_empty, "w"):
                    pass
            continue

        # else process this image
        out_img_path_cur = out_img_path_ok
        out_txt_path_cur = out_txt_path_ok
        side_reduce_val = int(abs(in_img_w - in_img_h) / 2)

        if in_img_w > in_img_h:
            out_img = in_img[:, side_reduce_val:-side_reduce_val]
            out_img_rect_points = [
                [side_reduce_val, in_img_h],
                [side_reduce_val, 0],
                [in_img_w - side_reduce_val, 0],
                [in_img_w - side_reduce_val, in_img_h]
            ]
        else:
            out_img = in_img[side_reduce_val:-side_reduce_val, :]
            out_img_rect_points = [
                [0, in_img_h - side_reduce_val],
                [0, side_reduce_val],
                [in_img_w, side_reduce_val],
                [in_img_w, in_img_h - side_reduce_val]
            ]
        out_img_w, out_img_h = out_img.shape[1], out_img.shape[0]

        # check for regions txt file availability
        if not os.path.isfile(in_txt_path):
            missed_txts_cnt += 1
            cv.imwrite(out_img_path_empty, out_img)
            with open(out_txt_path_empty, "w"):
                pass
            continue

        # load regions and check is txt file empty
        in_regions = load_regions(in_txt_path)
        if len(in_regions) == 0:
            empty_txts_cnt += 1
            cv.imwrite(out_img_path_empty, out_img)
            shutil.copyfile(in_txt_path, out_txt_path_empty)
            continue

        out_regions = []
        out_img_rect = Polygon(out_img_rect_points)
        file_contains_out_regs = False

        # convert regions
        for in_region in in_regions:
            # [object_class, x_center, y_center, width, height]
            in_region_items = [float(item) for item in in_region.split(" ")]
            # check for region values count
            if len(in_region_items) != 5:
                damaged_txts_cnt += 1
                cv.imwrite(out_img_path_damaged, out_img)
                shutil.copyfile(in_txt_path, out_txt_path_damaged)
                break

            # convert to px (for input image size)
            in_reg_x_c_px, in_reg_y_c_px, in_reg_w_px, in_reg_h_px = int(in_region_items[1] * in_img_w), \
                                                                     int(in_region_items[2] * in_img_h), \
                                                                     int(in_region_items[3] * in_img_w), \
                                                                     int(in_region_items[4] * in_img_h)
            in_reg_left, in_reg_right, in_reg_top, in_reg_bot = int(in_reg_x_c_px - in_reg_w_px / 2), \
                                                                int(in_reg_x_c_px + in_reg_w_px / 2), \
                                                                int(in_reg_y_c_px - in_reg_h_px / 2), \
                                                                int(in_reg_y_c_px + in_reg_h_px / 2)
            in_reg_px_pts = [
                (in_reg_left, in_reg_bot),
                (in_reg_left, in_reg_top),
                (in_reg_right, in_reg_top),
                (in_reg_right, in_reg_bot)
            ]

            # check if px region is not outside cropped image borders
            for point in in_reg_px_pts:
                if not out_img_rect.contains_point(point):
                    if not file_contains_out_regs:
                        file_contains_out_regs = True
                        regions_outside_files_cnt += 1
                    regions_outside_cnt += 1
                    out_img_path_cur = out_img_path_reg_out
                    out_txt_path_cur = out_txt_path_reg_out
                    break
            else:
                # compute output px region
                if in_img_w > in_img_h:
                    out_reg_left, out_reg_right, out_reg_top, out_reg_bot = in_reg_left - side_reduce_val, \
                                                                            in_reg_right - side_reduce_val, \
                                                                            in_reg_top, in_reg_bot
                else:
                    out_reg_left, out_reg_right, out_reg_top, out_reg_bot = in_reg_left, in_reg_right, \
                                                                            in_reg_top - side_reduce_val, \
                                                                            in_reg_bot - side_reduce_val
                # no need to convert to int as these values will be converted to yolo format which are floats
                out_reg_x_c_px, out_reg_y_c_px = out_reg_left + in_reg_w_px / 2, out_reg_top + in_reg_h_px / 2

                # convert to yolo format
                x_center, y_center, width, height = round(out_reg_x_c_px / out_img_w, 6), \
                                                    round(out_reg_y_c_px / out_img_h, 6), \
                                                    round(in_reg_w_px / out_img_w, 6), \
                                                    round(in_reg_h_px / out_img_h, 6)
                # format as text: "object_class_index x_center y_center width height"
                converted_reg = in_region[0] + " " + str(x_center) + " " + str(y_center) + " " + str(width) + " " + \
                                str(height)

                out_regions.append(converted_reg)
        else:
            # save regions and image to defined during processing path
            if out_txt_path_cur == out_txt_path_ok:
                ok_txts_cnt += 1

            cv.imwrite(out_img_path_cur, out_img)
            with open(out_txt_path_cur, "w") as out_txt_file:
                for out_region in out_regions:
                    out_txt_file.write(out_region + "\n")

        # display progress
        if cur_file_no % PROGRESS_PRINT_RATE == 0:
            print("Processed", cur_file_no, "of", len(img_files_paths), "images...")

    print("Processed all of", len(img_files_paths), "images.")
    print("Ok txts:", ok_txts_cnt, "(replaced to", OUTPUT_DIR, "directory)")
    print("Empty txts:", empty_txts_cnt, "(replaced to", OUTPUT_EMPTY_DIR, "directory)")
    print("Missed txts:", missed_txts_cnt, "(replaced to", OUTPUT_EMPTY_DIR, "directory)")
    print("Damaged txts:", damaged_txts_cnt, "(replaced to", OUTPUT_DAMAGED_DIR, "directory)")
    print("Files with regions outside:", regions_outside_files_cnt, "(replaced to", OUTPUT_REGS_OUTSIDE_DIR,
          "directory)")
    print("Regions outside count:", regions_outside_cnt)
    input("Done! Press enter to exit:")


if __name__ == '__main__':
    main()
