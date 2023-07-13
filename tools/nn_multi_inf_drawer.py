"""
Script for drawing multiple NNs inference results (from nn_multi_inf_runner.py) and "compiling" them into
a single files for viewing.

Requires input:
1) directories (structure described below) with .names files and yolo txt markup for drawing them

Expected dirs structure:
2) classes file is present for each network
(even if all networks are using same .names file; in network_file_name dir must be only one *.names file)
root_dir/network_name/*.names

txt-s with plants boxes in yolo format are located like this
root_dir/network_name/network_conf_nms/*.txt (any amount of txt-s)

Output: new directories with similar to input structure, containing compiled inference results images
"""

import os
import cv2 as cv
import glob
import platform
import math
import numpy as np
import json

# SETTINGS
# PATHS
# yolo txt files with inference results
INFERENCE_INPUT_DIR = "D:/test/test_input_inference/"
# directory with .jpg and yolo .txt files containing dataset markup
CONTROL_DATASET_INPUT_DIR = "D:/test/test_input_dataset/"
# .names file containing all classes possible to encounter in all inference results and control dataset
CONTROL_DATASET_INPUT_NAMES = "D:/test/Y0016.names"
IMAGES_OUTPUT_DIR = "D:/test/test_output_compiled_images/"

# OTHER SETTINGS
# draws box with color specified in GOOD_BOX_COLOR if there's a box in control dataset within this px distance
# otherwise box is drawn with a color spefiied in BAD_BOX_COLOR
GOOD_BOX_MAX_DIST_PX = 10

# colors are in opencv's BGR format (Blue, Green, Red)
GOOD_BOX_COLOR = (0, 0, 255)
GOOD_BOX_LABEL_TEXT_COLOR = (255, 255, 255)

BAD_BOX_COLOR = (0, 255, 255)
BAD_BOX_LABEL_TEXT_COLOR = (0, 0, 0)

FRAME_INFO_LABEL_TEXT_COLOR = (0, 0, 0)
FRAME_INFO_LABEL_BACKGROUND_COLOR = (255, 255, 255)


class DetectedPlantBox:

    def __init__(self, left, top, right, bottom, name, name_id, confidence, img_w, img_h, center_x=None, center_y=None):
        self.__left = left
        self.__top = top
        self.__right = right
        self.__bottom = bottom
        self.__name = name
        self.__name_id = name_id
        self.__confidence = confidence
        self.__center_x = center_x if center_x is not None else int(left + (right - left) / 2)
        self.__center_y = center_y if center_y is not None else int(top + (bottom - top) / 2)
        self.__image_width = img_w
        self.__image_height = img_h

    @property
    def center_x(self):
        return self.__center_x

    @property
    def center_y(self):
        return self.__center_y

    def __str__(self):
        return "Plant Box L=" + str(self.__left) + " T=" + str(self.__top) + " R=" + str(self.__right) + " B=" + \
               str(self.__bottom) + " X=" + str(self.__center_x) + " Y=" + str(self.__center_y)

    def get_box_points(self):
        """Returns left, top, right, bottom points of the box"""

        return self.__left, self.__top, self.__right, self.__bottom

    def get_center_points(self):
        """Returns pair of x and y box center coordinates"""

        return self.__center_x, self.__center_y

    def get_sizes(self):
        """Returns x and y sizes of the box"""

        return self.__right - self.__left, self.__bottom - self.__top

    def get_name(self):
        """Returns plant class name"""

        return self.__name

    def get_name_id(self):
        """Returns plant class name id"""

        return self.__name_id

    def get_confidence(self):
        return self.__confidence

    def get_as_yolo(self, return_as_text=False):
        """Returns box in yolo format, as str text, or as sequence of numbers"""

        # convert to yolo format
        x_center, y_center = round(self.__center_x / self.__image_width, 6), \
                             round(self.__center_y / self.__image_height, 6)
        box_px_width, box_px_height = self.get_sizes()
        width, height = round(box_px_width / self.__image_width, 6), round(box_px_height / self.__image_height, 6)

        # format as text: "object_class_index x_center y_center width height
        if return_as_text:
            return str(self.__name_id) + " " + str(x_center) + " " + str(y_center) + " " + str(width) + " " + \
                   str(height)
        # return as numbers: object_class_index, x_center, y_center, width, height
        else:
            return self.__name_id, x_center, y_center, width, height

    def get_distance_from(self, px_point_x, px_point_y):
        return math.sqrt((self.__center_x - px_point_x) ** 2 + (self.__center_y - px_point_y) ** 2)

    @staticmethod
    def yolo_to_plant_box(yolo_name_id,
                          yolo_x_center,
                          yolo_y_center,
                          yolo_width,
                          yolo_height,
                          name,
                          confidence,
                          img_w,
                          img_h):
        center_x, center_y, box_w, box_h = img_w * yolo_x_center, \
                                           img_h * yolo_y_center, \
                                           img_w * yolo_width, \
                                           img_h * yolo_height
        left, right, top, bottom = round(center_x - box_w / 2), \
                                   round(center_x + box_w / 2), \
                                   round(center_y - box_h / 2), \
                                   round(center_y + box_h / 2)
        center_x, center_y = round(center_x), round(center_y)

        return \
            DetectedPlantBox(left, top, right, bottom, name, yolo_name_id, confidence, img_w, img_h, center_x, center_y)

    def get_as_dict(self):
        return {
            "left": self.__left,
            "top": self.__top,
            "right": self.__right,
            "bottom": self.__bottom,
            "name": self.__name,
            "name_id": self.__name_id,
            "confidence": self.__confidence,
            "center_x": self.__center_x,
            "center_y": self.__center_y,
            "image_width": self.__image_width,
            "image_height": self.__image_height
        }

    def get_as_json(self):
        return json.dumps(self.get_as_dict())

    @staticmethod
    def make_from_dict(plant_box: dict):
        return DetectedPlantBox(
            plant_box["left"],
            plant_box["top"],
            plant_box["right"],
            plant_box["bottom"],
            plant_box["name"],
            plant_box["name_id"],
            plant_box["confidence"],
            plant_box["image_width"],
            plant_box["image_height"],
            plant_box["center_x"],
            plant_box["center_y"]
        )

    @staticmethod
    def make_from_json(json_box: str):
        return DetectedPlantBox.make_from_dict(json.loads(json_box))


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


def load_from_txt(txt_path: str, img_shape: tuple, classes_names: list) -> list:
    plants_boxes = []
    with open(txt_path) as txt_file:
        for line in txt_file:
            line = line.strip()
            if line != "":
                yolo_box = line.split(" ")
                img_h, img_w, _ = img_shape
                yolo_name_id, yolo_x_c, yolo_y_c, yolo_w, yolo_h = int(yolo_box[0]), \
                                                                   float(yolo_box[1]), \
                                                                   float(yolo_box[2]), \
                                                                   float(yolo_box[3]), \
                                                                   float(yolo_box[4])

                plants_boxes.append(DetectedPlantBox.yolo_to_plant_box(
                    yolo_name_id,
                    yolo_x_c,
                    yolo_y_c,
                    yolo_w,
                    yolo_h,
                    classes_names[yolo_name_id],
                    1,
                    img_w,
                    img_h))
    return plants_boxes


def load_class_names(labels_file_path) -> list:
    """Reads and parses classes names file by given path

    Returns list of object class names"""

    with open(labels_file_path, 'rt') as f:
        return f.read().rstrip('\n').split('\n')


def draw_boxes(image, boxes: list):
    for i in range(len(boxes)):
        draw_box(image, boxes[i])
    return image


def draw_box(image, box: DetectedPlantBox, box_color=(0, 0, 255), text_color=(0, 0, 0)):
    left, top, right, bottom = box.get_box_points()
    # Draw a bounding box
    cv.rectangle(image, (left, top), (right, bottom), box_color, 3)
    # draw a center of that box
    cv.circle(image, (int(left + (right - left) / 2), int(top + (bottom - top) / 2)), 4, box_color, thickness=3)

    label = '%s:%.2f' % (box.get_name(), box.get_confidence())

    # Display the label at the top of the bounding box
    label_size, base_line = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    top = max(top, label_size[1])
    cv.rectangle(image,
                 (left, top - round(1.5 * label_size[1])),
                 (left + round(1.5 * label_size[0]), top + base_line),
                 box_color,
                 cv.FILLED)
    cv.putText(image, label, (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.75, text_color, 2)


def main():
    print("Launching...")

    db = dict()
    slash = get_slash()
    create_directories(IMAGES_OUTPUT_DIR)
    ds_classes_names = load_class_names(CONTROL_DATASET_INPUT_NAMES)

    # make list of control dataset images and their txt-s
    ds_images_paths = glob.glob(CONTROL_DATASET_INPUT_DIR + "*.jpg")
    # ds_txts = list(map(lambda item: item[item.rfind(slash) + 1:], glob.glob(CONTROL_DATASET_INPUT_DIR + "*.txt")))

    # CACHE INFERENCE DATA
    # make list of NNs (mean inference results, not .trt files themselves)
    models_paths = list(filter(os.path.isdir, glob.glob(INFERENCE_INPUT_DIR + "*")))
    # loop over models and make a DB with available inference txt files
    print("Making inference results cache...")
    for cur_model_path_idx, cur_model_path in enumerate(models_paths):
        print(f"Processing {cur_model_path_idx + 1} of {len(models_paths)} models...")

        cur_model_name = cur_model_path[cur_model_path.rfind(slash) + 1:] if slash in cur_model_path else cur_model_path
        db[cur_model_name] = dict()
        db[cur_model_name]["settings_names"] = dict()
        cur_model_path += "/"

        cur_model_classes_files_paths = glob.glob(cur_model_path + "*.names")
        if len(cur_model_classes_files_paths) == 0:
            print(f"Couldn't find any *.names file in '{cur_model_path}'")
            exit()
        elif len(cur_model_classes_files_paths) > 1:
            print(f"Found multiple *.names files in '{cur_model_path}' (must be only one to be sure which to use)")
            exit()
        db[cur_model_name]["classes"] = load_class_names(cur_model_classes_files_paths[0])

        cur_model_settings_paths = list(filter(os.path.isdir, glob.glob(cur_model_path + "*")))
        for cur_model_settings_path in cur_model_settings_paths:
            cur_model_settings_name = cur_model_settings_path[cur_model_settings_path.rfind(slash) + 1:]
            db[cur_model_name]["settings_names"][cur_model_settings_name] = list()
            cur_model_settings_path += "/"

            cur_inf_txt_paths = glob.glob(cur_model_settings_path + "*.txt")
            for cur_inf_txt_path in cur_inf_txt_paths:
                cur_inf_txt_name = cur_inf_txt_path[cur_inf_txt_path.rfind(slash) + 1:]
                db[cur_model_name]["settings_names"][cur_model_settings_name].append(cur_inf_txt_name)

    # loop over control dataset images and draw inferences results
    inf_to_draw = []
    for cur_ds_img_path_idx, cur_ds_img_path in enumerate(ds_images_paths):
        print(f"Processing {cur_ds_img_path_idx + 1} of {len(ds_images_paths)} control dataset images...")

        cur_ds_img_name = cur_ds_img_path[cur_ds_img_path.rfind(slash) + 1:] if slash in cur_ds_img_path else cur_ds_img_path
        cur_ds_txt_name = cur_ds_img_name[:-4] + ".txt"

        cur_ds_img_orig = cv.imread(cur_ds_img_path)

        # try to load dataset image's txt
        if os.path.isfile(cur_ds_img_path[:-4] + ".txt"):
            cur_ds_plants_boxes = load_from_txt(cur_ds_img_path[:-4] + ".txt", cur_ds_img_orig.shape, ds_classes_names)
        else:
            print(f"Couldn't find '{cur_ds_img_path[:-4] + '.txt'}' file, dataset boxes won't be drawn.")
            cur_ds_plants_boxes = []

        # make list of available inference results for current image
        for cur_model_name in db:
            for cur_model_settings_name in db[cur_model_name]["settings_names"]:
                if cur_ds_txt_name in db[cur_model_name]["settings_names"][cur_model_settings_name]:
                    with open(f"{INFERENCE_INPUT_DIR}{cur_model_name}/{cur_model_settings_name}/{cur_ds_txt_name}") as \
                            json_txt_file:
                        inf_to_draw.append({
                            "model": cur_model_name,
                            "settings": cur_model_settings_name,
                            "plants_boxes": list(map(DetectedPlantBox.make_from_dict, json.loads(json_txt_file.read())))
                        })

        # skip if no inference txt files are available for current dataset image
        if len(inf_to_draw) == 0:
            print(f"Couldn't find any inference txt files for '{cur_ds_img_name}' image file in dataset. Skipping.")
            continue

        img_h, img_w, _ = cur_ds_img_orig.shape

        # prepare image with drawn dataset boxes and label
        cur_ds_img = cur_ds_img_orig.__deepcopy__(cur_ds_img_orig)
        draw_boxes(cur_ds_img, cur_ds_plants_boxes)
        ds_img_label = f"TXT FILE: Parsed {str(len(cur_ds_plants_boxes))} boxes"
        ds_img_label_w, ds_img_label_h = cv.getTextSize(ds_img_label, cv.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0]
        ds_img_label_x_pos = 10
        ds_img_label_y_pos = 20
        cv.rectangle(
            cur_ds_img,
            (ds_img_label_x_pos, ds_img_label_y_pos - ds_img_label_h - 1),
            (ds_img_label_x_pos + ds_img_label_w, round(ds_img_label_y_pos + ds_img_label_h * 0.5)),
            FRAME_INFO_LABEL_BACKGROUND_COLOR,
            cv.FILLED)
        cv.putText(
            cur_ds_img,
            ds_img_label,
            (ds_img_label_x_pos, ds_img_label_y_pos),
            cv.FONT_HERSHEY_SIMPLEX,
            0.75,
            FRAME_INFO_LABEL_TEXT_COLOR,
            2)

        # draw and compile inferences boxes
        compiled_img = np.ndarray((cur_ds_img_orig.shape[0] * 2, 1, 3), dtype=np.uint8)
        compiled_img.fill(0)

        # loop over different inferences
        for inf_item in inf_to_draw:
            cur_inf_img = cur_ds_img_orig.__deepcopy__(cur_ds_img_orig)

            # loop over boxes of current inference and draw them using different colors depending on distance to
            # nearest dataset box
            for inf_box in inf_item["plants_boxes"]:  # cur inf boxes
                for ds_box in cur_ds_plants_boxes:  # txt boxes
                    if inf_box.get_distance_from(ds_box.center_x, ds_box.center_y) <= GOOD_BOX_MAX_DIST_PX:
                        draw_box(cur_inf_img, inf_box, GOOD_BOX_COLOR, GOOD_BOX_LABEL_TEXT_COLOR)
                        break
                else:
                    draw_box(cur_inf_img, inf_box, BAD_BOX_COLOR, BAD_BOX_LABEL_TEXT_COLOR)

            # draw label
            inf_img_label = f"{inf_item['model']}/{inf_item['settings']}: " \
                            f"Found {str(len(inf_item['plants_boxes']))} boxes"
            inf_img_label_w, inf_img_label_h = cv.getTextSize(inf_img_label, cv.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0]

            inf_img_label_x_pos = 10
            inf_img_label_y_pos = img_h - inf_img_label_h - 10

            cv.rectangle(
                cur_inf_img,
                (inf_img_label_x_pos, inf_img_label_y_pos - inf_img_label_h - 1),
                (inf_img_label_x_pos + inf_img_label_w, round(inf_img_label_y_pos + inf_img_label_h * 0.5)),
                FRAME_INFO_LABEL_BACKGROUND_COLOR,
                cv.FILLED)
            cv.putText(
                cur_inf_img,
                inf_img_label,
                (inf_img_label_x_pos, inf_img_label_y_pos),
                cv.FONT_HERSHEY_SIMPLEX,
                0.75,
                FRAME_INFO_LABEL_TEXT_COLOR,
                2)

            # pack images to a single file
            img_to_add = np.concatenate((cur_inf_img, cur_ds_img), axis=0)
            compiled_img = np.concatenate((compiled_img, img_to_add), axis=1)

        cv.imwrite(IMAGES_OUTPUT_DIR + cur_ds_img_name, compiled_img[:, 1:])
        inf_to_draw.clear()

    print(f"Done! Results are available at '{IMAGES_OUTPUT_DIR}'")


if __name__ == '__main__':
    main()
