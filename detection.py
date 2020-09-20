import cv2 as cv
import numpy as np
import os
from config import config
import glob
import math
import platform


class YoloOpenCVDetection:

    def __init__(self, yolo_classes_path, yolo_config_path, yolo_weights_path, input_size, confidence_threshold,
                 nms_threshold, backend, target):
        self._input_size = input_size
        self._confidence_threshold = confidence_threshold
        self._nms_threshold = nms_threshold  # non-max suppression
        self.classes = self.load_class_names(yolo_classes_path)
        self.net = cv.dnn.readNetFromDarknet(yolo_config_path, yolo_weights_path)
        self.net.setPreferableBackend(backend)  # DNN_BACKEND_OPENCV for CPU usage
        self.net.setPreferableTarget(target)  # DNN_TARGET_CPU for CPU usage

    def detect(self, image):
        # Create a 4D blob from a frame.
        blob = cv.dnn.blobFromImage(image, 1 / 255, self._input_size, [0, 0, 0], 1, crop=False)

        # Sets the input to the network
        self.net.setInput(blob)

        # Runs the forward pass to get output of the output layers
        out_layers = self._get_output_names(self.net)
        outs = self.net.forward(out_layers)

        # Remove the bounding boxes with low confidence
        return self._post_process(image, outs, self.classes)

    # Get the names of the output layers
    def _get_output_names(self, net):
        # Get the names of all the layers in the network
        layers_names = net.getLayerNames()
        # Get the names of the output layers, i.e. the layers with unconnected outputs
        return [layers_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # Remove the bounding boxes with low confidence using non-maxima suppression
    def _post_process(self, image, outs, classes):
        frame_height = image.shape[0]
        frame_width = image.shape[1]
        class_ids = []
        confidences = []
        boxes = []
        final_plant_boxes = []

        # Scan through all the bounding boxes output from the network and keep only the
        # ones with high confidence scores. Assign the box's class label as the class with the highest score.
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                # if detection[4] > conf_threshold:
                #     print(detection[4], " - ", scores[class_id], " - th : ", conf_threshold)
                #     print(detection)

                if confidence > self._confidence_threshold:
                    center_x = int(detection[0] * frame_width)
                    center_y = int(detection[1] * frame_height)
                    width = int(detection[2] * frame_width)
                    height = int(detection[3] * frame_height)
                    left = int(center_x - width / 2)
                    top = int(center_y - height / 2)
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([left, top, width, height])

        # Perform non maximum suppression to eliminate redundant overlapping boxes with lower confidences.
        indices = cv.dnn.NMSBoxes(boxes, confidences, self._confidence_threshold, self._nms_threshold)

        for i in indices:
            i = i[0]
            box = boxes[i]
            left, top, width, height = box[0], box[1], box[2], box[3]
            final_plant_boxes.append(DetectedPlantBox(left, top, left + width, top + height, classes, class_ids[i],
                                                      confidences[i]))
        return final_plant_boxes

    # Load names of classes
    @staticmethod
    def load_class_names(labels_file):
        with open(labels_file, 'rt') as f:
            return f.read().rstrip('\n').split('\n')


class DetectedPlantBox:

    def __init__(self, left, top, right, bottom, classes, class_id, confidence):
        self._left = left
        self._top = top
        self._right = right
        self._bottom = bottom
        self._center_x = int(left + (right - left) / 2)
        self._center_y = int(top + (bottom - top) / 2)
        self._classes = classes
        self._class_id = class_id
        self._confidence = confidence

    def __str__(self):
        return "Plant Box L=" + str(self._left) + " T=" + str(self._top) + " R=" + str(self._right) + " B=" + \
               str(self._bottom) + " X=" + str(self._center_x) + " Y=" + str(self._center_y)

    def get_box_points(self):
        """Returns left, top, right, bottom points of the box"""

        return self._left, self._top, self._right, self._bottom

    def get_center_points(self):
        """Returns pair of x and y box center coordinates"""

        return self._center_x, self._center_y

    def get_sizes(self):
        """Returns x and y sizes of the box"""

        return self._right - self._left, self._bottom - self._top

    def get_name(self):
        """Returns plant class name"""

        return self._classes[self._class_id]

    def get_class_id(self):
        """Returns plant class index in the classes list"""

        return self._class_id

    def get_confidence(self):
        return self._confidence

    def get_distance_from(self, px_point_x, px_point_y):
        return math.sqrt((self._center_x - px_point_x) ** 2 + (self._center_y - px_point_y) ** 2)


# Draw the predicted bounding box
def draw_box(image, box: DetectedPlantBox):
    left, top, right, bottom = box.get_box_points()
    # Draw a bounding box
    cv.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 3)
    # draw a center of that box
    cv.circle(image, (int(left + (right - left) / 2), int(top + (bottom - top) / 2)), 4, (0, 0, 255), thickness=3)

    label = '%s:%.2f' % (box.get_name(), box.get_confidence())

    # Display the label at the top of the bounding box
    label_size, base_line = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    top = max(top, label_size[1])
    cv.rectangle(image, (left, top - round(1.5 * label_size[1])), (left + round(1.5 * label_size[0]), top + base_line),
                 (0, 0, 255), cv.FILLED)
    cv.putText(image, label, (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 2)


def draw_boxes(image, boxes: list):
    for i in range(len(boxes)):
        draw_box(image, boxes[i])
    return image


def detect_all_in_directory(detector: YoloOpenCVDetection, input_dir, output_dir):
    paths_to_images = glob.glob(input_dir + "*.jpg")
    counter = 1
    for image_path in paths_to_images:
        print("Processing", counter, "of", len(paths_to_images), "images")
        counter += 1
        detect_single(detector, image_path, output_dir)


def detect_single(detector: YoloOpenCVDetection, input_full_path, output_dir):
    img = cv.imread(input_full_path)
    boxes = detector.detect(img)
    draw_boxes(img, boxes)
    slash = "\\" if platform.system() == "Windows" else "/"
    file_name = input_full_path.split(slash)[-1]
    cv.imwrite(output_dir + file_name, img)


def _create_directories(*args):
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


def _test():
    precise_input_dir = "precise input/"
    precise_output_dir = "precise ouput/"
    periphery_input_dir = "periphery input/"
    periphery_output_dir = "periphery ouput/"

    _create_directories(precise_input_dir, precise_output_dir, periphery_input_dir, periphery_output_dir)

    periphery_detector = YoloOpenCVDetection(config.PERIPHERY_CLASSES_FILE, config.PERIPHERY_CONFIG_FILE,
                                             config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_INPUT_SIZE,
                                             config.PERIPHERY_CONFIDENCE_THRESHOLD,
                                             config.PERIPHERY_NMS_THRESHOLD, config.PERIPHERY_DNN_BACKEND,
                                             config.PERIPHERY_DNN_TARGET)
    precise_detector = YoloOpenCVDetection(config.PRECISE_CLASSES_FILE, config.PRECISE_CONFIG_FILE,
                                           config.PRECISE_WEIGHTS_FILE, config.PRECISE_INPUT_SIZE,
                                           config.PRECISE_CONFIDENCE_THRESHOLD, config.PRECISE_NMS_THRESHOLD,
                                           config.PRECISE_DNN_BACKEND,config.PRECISE_DNN_TARGET)

    detect_all_in_directory(precise_detector, precise_input_dir, precise_output_dir)
    detect_all_in_directory(periphery_detector, periphery_input_dir, periphery_output_dir)
    print("Done!")


if __name__ == '__main__':
    _test()
