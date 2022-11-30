"""
Loops over CONFIDENCE and NMS_THRESHOLD ranges, applies these settings to yolo network NN_1 and compares it to another
network NN_2 (same settings applied) or yolo txt labels. Saves all results for further inspections.
"""

import os
import detection
import cv2 as cv
import glob
import platform
import numpy as np
import time

# YOLO SETTINGS
# divided by 100. I.E. 25 means 0.25
CONFIDENCE_FROM = 10
CONFIDENCE_TO = 21
CONFIDENCE_STEP = 5
NMS_THRESHOLD_FROM = 40
NMS_THRESHOLD_TO = 41
NMS_THRESHOLD_STEP = 1

# NN 1; will be compared to txt or to NN 2 dependent on NN_TO_TXT setting
NN_1_MODEL_PATH = "yolo/Y0016.trt"
NN_1_CLASSES_FILE = "yolo/Y0023.names"
# NN 2
NN_2_MODEL_PATH = "yolo/Y0025d_mono_416.trt"
NN_2_CLASSES_FILE = "yolo/Y0025d_mono.names"

# PATHS SETTINGS
IMAGES_INPUT_DIR = "test_simple/"
IMAGES_OUTPUT_DIR = "test_simple_output/"

# OTHER SETTINGS
# False: compare given NN_1 to txt markup; True: compare NN_1 to NN_2
COMPARE_NN_TO_NN = True
# used in NN to txt mode; draws red box if there's a box in txt at this px distance, draws yellow box otherwise
GOOD_BOX_MAX_DIST_PX = 10
# colors are in opencv's BGR format (Blue, Green, Red)
GOOD_BOX_COLOR = (0, 0, 255)
GOOD_BOX_LABEL_TEXT_COLOR = (0, 0, 0)
BAD_BOX_COLOR = (0, 255, 255)
BAD_BOX_LABEL_TEXT_COLOR = (255, 255, 255)
FRAME_INFO_LABEL_TEXT_COLOR = (0, 0, 0)
FRAME_INFO_LABEL_BACKGROUND_COLOR = (255, 255, 255)


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


def nn_to_txt_process_all(nn_1: detection.YoloTRTDetector,
                          nn_1_classes_names: list,
                          input_dir: str,
                          output_dir: str):

    nn_1_total_detections = txt_total_detections = 0
    counter = 1
    paths_to_images = glob.glob(input_dir + "*.jpg")

    for image_path in paths_to_images:
        print("Processing", counter, "of", len(paths_to_images), "images")
        counter += 1

        nn_1_box_cnt, txt_box_cnt = nn_to_txt_process_single(nn_1, nn_1_classes_names, image_path, output_dir)
        nn_1_total_detections += nn_1_box_cnt
        txt_total_detections += txt_box_cnt

    return nn_1_total_detections, txt_total_detections


def nn_to_txt_process_single(nn_1: detection.YoloTRTDetector,
                             nn_1_classes_names: list,
                             input_full_path: str,
                             output_dir: str):
    # nn_2 is txt parsed boxes

    txt_path = input_full_path[:-4] + ".txt"
    if not os.path.isfile(txt_path):
        raise RuntimeError(f"{txt_path} file is missing!")

    nn_1_img = cv.imread(input_full_path)
    nn_2_img = nn_1_img.__deepcopy__(nn_1_img)

    nn_1_img_h, nn_1_img_w, _ = nn_1_img.shape
    nn_2_img_h, nn_2_img_w = nn_1_img_h, nn_1_img_w

    nn_1_start_t = time.time()
    nn_1_boxes = nn_1.detect(nn_1_img)
    nn_1_inf_t = time.time() - nn_1_start_t
    nn_2_boxes = []
    with open(txt_path) as txt_file:
        for line in txt_file:
            line = line.strip()
            if line != "":
                yolo_box = line.split(" ")
                img_h, img_w, _ = nn_2_img.shape
                yolo_name_id, yolo_x_c, yolo_y_c, yolo_w, yolo_h = int(yolo_box[0]), \
                                                                   float(yolo_box[1]), \
                                                                   float(yolo_box[2]), \
                                                                   float(yolo_box[3]), \
                                                                   float(yolo_box[4])

                nn_2_boxes.append(detection.DetectedPlantBox.yolo_to_plant_box(
                    yolo_name_id,
                    yolo_x_c,
                    yolo_y_c,
                    yolo_w,
                    yolo_h,
                    nn_1_classes_names[yolo_name_id],
                    1,
                    img_w,
                    img_h))

    # draw txt boxes
    detection.draw_boxes(nn_2_img, nn_2_boxes)
    # draw nn_1 inference boxes dependent on distance to closest txt box
    for nn_1_box in nn_1_boxes:
        for nn_2_box in nn_2_boxes:  # txt boxes
            if nn_1_box.get_distance_from(nn_2_box.center_x, nn_2_box.center_y) <= GOOD_BOX_MAX_DIST_PX:
                detection.draw_box(nn_1_img, nn_1_box, GOOD_BOX_COLOR, GOOD_BOX_LABEL_TEXT_COLOR)
                break
        else:
            detection.draw_box(nn_1_img, nn_1_box, BAD_BOX_COLOR, BAD_BOX_LABEL_TEXT_COLOR)

    nn_1_label = f"{NN_1_MODEL_PATH}: Found {str(len(nn_1_boxes))} boxes in {str(round(nn_1_inf_t, 3))} sec"
    nn_2_label = f"TXT FILE: Parsed {str(len(nn_2_boxes))} boxes"

    nn_1_label_w, nn_1_label_h = cv.getTextSize(nn_1_label, cv.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0]
    nn_2_label_w, nn_2_label_h = cv.getTextSize(nn_2_label, cv.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0]

    nn_1_label_x_pos = nn_1_img_w - nn_1_label_w - 10
    nn_1_label_y_pos = nn_1_img_h - nn_1_label_h - 10
    nn_2_label_x_pos = 10
    nn_2_label_y_pos = nn_2_img_h - nn_2_label_h - 10

    cv.rectangle(nn_1_img,
                 (nn_1_label_x_pos, nn_1_label_y_pos - nn_1_label_h - 1),
                 (nn_1_label_x_pos + nn_1_label_w, round(nn_1_label_y_pos + nn_1_label_h * 0.5)),
                 FRAME_INFO_LABEL_BACKGROUND_COLOR,
                 cv.FILLED)
    cv.rectangle(nn_2_img,
                 (nn_2_label_x_pos, nn_2_label_y_pos - nn_2_label_h - 1),
                 (nn_2_label_x_pos + nn_2_label_w, round(nn_2_label_y_pos + nn_2_label_h * 0.5)),
                 FRAME_INFO_LABEL_BACKGROUND_COLOR,
                 cv.FILLED)

    cv.putText(nn_1_img,
               nn_1_label,
               (nn_1_label_x_pos, nn_1_label_y_pos),
               cv.FONT_HERSHEY_SIMPLEX,
               0.75,
               FRAME_INFO_LABEL_TEXT_COLOR,
               2)
    cv.putText(nn_2_img,
               nn_2_label,
               (nn_2_label_x_pos, nn_2_label_y_pos),
               cv.FONT_HERSHEY_SIMPLEX,
               0.75,
               FRAME_INFO_LABEL_TEXT_COLOR,
               2)

    res_img = np.concatenate((nn_1_img, nn_2_img), axis=1)

    input_file_name = input_full_path.split(get_slash())[-1]
    output_file_name = f"{output_dir}nn1_{str(len(nn_1_boxes))}_txt_{str(len(nn_2_boxes))}_{input_file_name}"
    cv.imwrite(output_file_name, res_img)

    return len(nn_1_boxes), len(nn_2_boxes)


def nn_to_nn_process_all(nn_1: detection.YoloTRTDetector,
                         nn_2: detection.YoloTRTDetector,
                         input_dir: str,
                         output_dir: str):
    nn_1_total_detections = nn_2_total_detections = 0
    paths_to_images = glob.glob(input_dir + "*.jpg")
    counter = 1

    for image_path in paths_to_images:
        print("Processing", counter, "of", len(paths_to_images), "images")
        counter += 1
        nn_1_box_cnt, nn_2_box_cnt = nn_to_nn_process_single(nn_1, nn_2, image_path, output_dir)
        nn_1_total_detections += nn_1_box_cnt
        nn_2_total_detections += nn_2_box_cnt

    return nn_1_total_detections, nn_2_total_detections


def nn_to_nn_process_single(nn_1: detection.YoloTRTDetector,
                            nn_2: detection.YoloTRTDetector,
                            input_full_path: str,
                            output_dir: str):
    nn_1_img = cv.imread(input_full_path)
    nn_2_img = nn_1_img.__deepcopy__(nn_1_img)

    nn_1_img_h, nn_1_img_w, _ = nn_1_img.shape
    nn_2_img_h, nn_2_img_w = nn_1_img_h, nn_1_img_w

    nn_1_start_t = time.time()
    nn_1_boxes = nn_1.detect(nn_1_img)
    nn_1_inf_t = time.time() - nn_1_start_t
    nn_2_start_t = time.time()
    nn_2_boxes = nn_2.detect(nn_2_img)
    nn_2_inf_t = time.time() - nn_2_start_t

    detection.draw_boxes(nn_1_img, nn_1_boxes)
    detection.draw_boxes(nn_2_img, nn_2_boxes)

    nn_1_label = f"{NN_1_MODEL_PATH}: Found {str(len(nn_1_boxes))} boxes in {str(round(nn_1_inf_t, 3))} sec"
    nn_2_label = f"{NN_2_MODEL_PATH}: Found {str(len(nn_2_boxes))} boxes in {str(round(nn_2_inf_t, 3))} sec"

    nn_1_label_w, nn_1_label_h = cv.getTextSize(nn_1_label, cv.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0]
    nn_2_label_w, nn_2_label_h = cv.getTextSize(nn_2_label, cv.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0]

    nn_1_label_x_pos = nn_1_img_w - nn_1_label_w - 10
    nn_1_label_y_pos = nn_1_img_h - nn_1_label_h - 10
    nn_2_label_x_pos = 10
    nn_2_label_y_pos = nn_2_img_h - nn_2_label_h - 10

    cv.rectangle(nn_1_img,
                 (nn_1_label_x_pos, nn_1_label_y_pos - nn_1_label_h - 1),
                 (nn_1_label_x_pos + nn_1_label_w, round(nn_1_label_y_pos + nn_1_label_h * 0.5)),
                 FRAME_INFO_LABEL_BACKGROUND_COLOR,
                 cv.FILLED)
    cv.rectangle(nn_2_img,
                 (nn_2_label_x_pos, nn_2_label_y_pos - nn_2_label_h - 1),
                 (nn_2_label_x_pos + nn_2_label_w, round(nn_2_label_y_pos + nn_2_label_h * 0.5)),
                 FRAME_INFO_LABEL_BACKGROUND_COLOR,
                 cv.FILLED)

    cv.putText(nn_1_img,
               nn_1_label,
               (nn_1_label_x_pos, nn_1_label_y_pos),
               cv.FONT_HERSHEY_SIMPLEX,
               0.75,
               FRAME_INFO_LABEL_TEXT_COLOR,
               2)
    cv.putText(nn_2_img,
               nn_2_label,
               (nn_2_label_x_pos, nn_2_label_y_pos),
               cv.FONT_HERSHEY_SIMPLEX,
               0.75,
               FRAME_INFO_LABEL_TEXT_COLOR,
               2)

    res_img = np.concatenate((nn_1_img, nn_2_img), axis=1)

    input_file_name = input_full_path.split(get_slash())[-1]
    output_file_name = f"{output_dir}nn1_{str(len(nn_1_boxes))}_nn2_{str(len(nn_2_boxes))}_{input_file_name}"
    cv.imwrite(output_file_name, res_img)

    return len(nn_1_boxes), len(nn_2_boxes)


def main():
    create_directories(IMAGES_OUTPUT_DIR)

    nn_1_classes_names = detection.load_class_names(NN_1_CLASSES_FILE)

    # loop over settings ranges
    for confidence in range(CONFIDENCE_FROM, CONFIDENCE_TO, CONFIDENCE_STEP):
        confidence = round(confidence / 100, 2)
        for nms_threshold in range(NMS_THRESHOLD_FROM, NMS_THRESHOLD_TO, NMS_THRESHOLD_STEP):
            nms_threshold = round(nms_threshold / 100, 2)

            # define cur path and create dir
            current_dir = IMAGES_OUTPUT_DIR + "conf " + str(confidence) + " nms " + str(nms_threshold) + get_slash()
            create_directories(current_dir)

            # create detector with current settings
            nn_1 = detection.YoloTRTDetector(NN_1_MODEL_PATH, NN_1_CLASSES_FILE, confidence, nms_threshold)

            # do comparison to NN_2
            if COMPARE_NN_TO_NN:
                nn_2 = detection.YoloTRTDetector(NN_2_MODEL_PATH, NN_2_CLASSES_FILE, confidence, nms_threshold)
                nn_1_det_cnt, nn_2_det_cnt = nn_to_nn_process_all(nn_1, nn_2, IMAGES_INPUT_DIR, current_dir)
                os.rename(current_dir,
                          current_dir[:-len(get_slash())] +
                          " (NN_1_" + str(nn_1_det_cnt) +
                          "_NN_2_" + str(nn_2_det_cnt) + ")" + get_slash())
            else:
                nn_1_det_cnt, txt_det_cnt = nn_to_txt_process_all(
                    nn_1,
                    nn_1_classes_names,
                    IMAGES_INPUT_DIR,
                    current_dir)
                os.rename(current_dir,
                          current_dir[:-len(get_slash())] +
                          " (NN_1_" + str(nn_1_det_cnt) +
                          "_TXT_" + str(txt_det_cnt) + ")" + get_slash())


if __name__ == '__main__':
    main()
