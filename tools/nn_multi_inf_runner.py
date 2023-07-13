"""
Launches multiple NN models and saves their inference results.
Requires:
1) directory with any amount of *.trt files and .names files
(.names file name must be a NN name: Y0034.names both for Y0034_320_320.trt and
Y0034_anything1_anything2_anythingN_320_320.trt)
2) directory with images to do inference on (this script is not requiring .txt files with boxes present, but they will
be needed for image compiler script to draw results)
Loops over:
1) all NN models in given directory
2) confidence and NMS settings inside this script
3) all images in given directory
Saves txt files with inference results in structure of directories, which contain settings data. This structure is
used by image "compiler" script (nn_multi_inf_drawer.py), which will draw all results for viewing.
"""

import os
import detection
import cv2 as cv
import glob
import platform
import shutil
import json

# YOLO SETTINGS
# divided by 100. I.E. 25 means 0.25
CONFIDENCE_FROM = 11
CONFIDENCE_TO = 13
CONFIDENCE_STEP = 1
NMS_THRESHOLD_FROM = 40
NMS_THRESHOLD_TO = 42
NMS_THRESHOLD_STEP = 1

# PATHS SETTINGS
# models names expected format is UniqueModelName_resX_resY.trt, also there must be UniqueModelName.names file.
# IMPORTANT: object id in *.names files MUST be the same. I.E. if plantain has id=2 - it must be 2 in any *.names file.
MODELS_INPUT_PATH = "test_models/"
IMAGES_INPUT_DIR = "test/"
RESULTS_OUTPUT_DIR = "test_result/"


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
    slash = get_slash()

    create_directories(RESULTS_OUTPUT_DIR)

    # get list of available trt models
    models_paths = glob.glob(MODELS_INPUT_PATH + "*.trt")
    if len(models_paths) < 1:
        print(f"Couldn't find any *.trt files in given models '{MODELS_INPUT_PATH}' directory.")
        exit()

    for model_path in models_paths:
        # define and check current .trt and its .names file and these files names
        model_file_name = model_path[model_path.rfind(slash) + 1:] if slash in model_path else model_path
        spl_model_name = model_file_name.split("_")
        if len(spl_model_name) < 3:
            print(f"Model '{model_file_name}' name can't be parsed due to wrong name format. "
                  f"See expected format in explanations of MODELS_INPUT_PATH key at the beginning of this script.")
            exit()
        input_size = (int(spl_model_name[-2]), int(spl_model_name[-1][:-4]))
        model_name = spl_model_name[0]
        names_file_name = model_name + ".names"
        names_file_path_full = MODELS_INPUT_PATH + names_file_name
        if not os.path.isfile(names_file_path_full):
            print(f"Couldn't find '{names_file_path_full}' file of '{model_path}' model.")
            exit()

        # load current model's names file
        classes_names = detection.load_class_names(names_file_path_full)

        cur_out_model_dir = f"{RESULTS_OUTPUT_DIR}{model_file_name[:-4]}/"
        create_directories(cur_out_model_dir)

        # copy names file to current NN output dir
        shutil.copy(names_file_path_full, cur_out_model_dir + names_file_name)

        # loop over confidence and NMS ranges (given in settings)
        for confidence in range(CONFIDENCE_FROM, CONFIDENCE_TO, CONFIDENCE_STEP):
            confidence = round(confidence / 100, 2)
            for nms_threshold in range(NMS_THRESHOLD_FROM, NMS_THRESHOLD_TO, NMS_THRESHOLD_STEP):
                nms_threshold = round(nms_threshold / 100, 2)

                # define cur path and create dir
                cur_out_model_conf_nms_dir = f"{cur_out_model_dir}conf_{str(confidence)}_nms_{str(nms_threshold)}/"
                create_directories(cur_out_model_conf_nms_dir)

                # create detector with current settings
                print(f"Loading model '{model_file_name}' with confidence={confidence}, nms={nms_threshold}...")
                detector = detection.YoloTRTDetector(
                    model_path,
                    names_file_path_full,
                    confidence,
                    nms_threshold,
                    input_size)

                # loop over images in dataset
                images_paths = glob.glob(IMAGES_INPUT_DIR + "*.jpg")
                counter = 1
                for image_path in images_paths:
                    print("Processing", counter, "of", len(images_paths), "images")
                    counter += 1

                    image = cv.imread(image_path)
                    image_name = image_path[image_path.rfind(slash) + 1:] if slash in image_path else image_path
                    plants_boxes = detector.detect(image)

                    # save detected objects in yolo format
                    with open(cur_out_model_conf_nms_dir + image_name[:-4] + ".txt", "w") as out_txt_file:
                        out_txt_file.write(
                            json.dumps(
                                list(
                                    map(
                                        lambda plant_box: plant_box.get_as_dict(),
                                        plants_boxes
                                    )
                                )
                            )
                        )


if __name__ == '__main__':
    main()
