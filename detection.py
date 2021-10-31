import cv2 as cv
import numpy as np
import os
import glob
import math
import platform
import darknet

from config import config
import posix_ipc
from mmap import mmap
import time
from multiprocessing import Process
from liveMain import webstreaming
from flask import Flask
import logging
from flask_cors import CORS

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
        out_layers = self.__get_output_names(self.net)
        outs = self.net.forward(out_layers)

        # Remove the bounding boxes with low confidence
        return self.__post_process(image, outs, self.classes)

    # Get the names of the output layers
    def __get_output_names(self, net):
        # Get the names of all the layers in the network
        layers_names = net.getLayerNames()
        # Get the names of the output layers, i.e. the layers with unconnected outputs
        return [layers_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # Remove the bounding boxes with low confidence using non-maxima suppression
    def __post_process(self, image, outs, classes):
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
            final_plant_boxes.append(DetectedPlantBox(left, top, left + width, top + height, classes[class_ids[i]],
                                                      class_ids[i], confidences[i], frame_width, frame_height))
        return final_plant_boxes

    # Load names of classes
    @staticmethod
    def load_class_names(labels_file):
        with open(labels_file, 'rt') as f:
            return f.read().rstrip('\n').split('\n')


class YoloDarknetDetector:
    # TODO: images are distorted a bit during strict resize wo/o saving ratio, this may affect detections
    # TODO: check for interpolation (INTER_AREA should be better than INTER_LINEAR)

    WEBSTREAM = False
    webStream = None

    def __init__(self,
                 weights_file_path,  # yolo weights path
                 config_file_path,  # path to config file
                 data_file_path,  # path to data file"
                 confidence_threshold,  # remove detections with lower than this confidence
                 hier_threshold,
                 nms_threshold):
        # check for args errors
        assert 0 < confidence_threshold < 1, "Confidence threshold should be a float between zero and one (non-inclusive)"
        assert 0 < nms_threshold < 1, "NMS Threshold should be a float between zero and one (non-inclusive)"
        if not os.path.exists(config_file_path):
            raise ValueError("Invalid config path {}".format(os.path.abspath(config_file_path)))
        if not os.path.exists(weights_file_path):
            raise ValueError("Invalid weight path {}".format(os.path.abspath(weights_file_path)))
        if not os.path.exists(data_file_path):
            raise ValueError("Invalid data file path {}".format(os.path.abspath(data_file_path)))

        self.__confidence_threshold = confidence_threshold
        self.__nms_threshold = nms_threshold
        self.__hier_threshold = hier_threshold
        self.__network, self.__class_names, self.__class_colors = darknet.load_network(config_file_path, data_file_path,
                                                                                       weights_file_path)
        # current network input size
        self.__width, self.__height = darknet.network_width(self.__network), darknet.network_height(self.__network)

        self.sharedArray = None

    def get_classes_names(self):
        return self.__class_names

    def detect(self, image, disable_frame_show=False):
        # Darknet doesn't accept numpy images.
        # Create one with image we reuse for each detect

        image_prepared = cv.resize(image, (self.__width, self.__height), interpolation=cv.INTER_LINEAR)
        image_prepared = cv.cvtColor(image_prepared, cv.COLOR_BGR2RGB)
        darknet_image = darknet.make_image(self.__width, self.__height, 3)
        darknet.copy_image_from_bytes(darknet_image, image_prepared.tobytes())
        detections = darknet.detect_image(self.__network, self.__class_names, darknet_image,
                                          thresh=self.__confidence_threshold, hier_thresh=self.__hier_threshold,
                                          nms=self.__nms_threshold)
        darknet.free_image(darknet_image)

        # convert detections to Natuition format
        # darknet detections structure and example:
        # [(label, confidence, center_x, center_y, full_size_x, full_size_y)]
        # [('Dandellion', '40.87', (356.56658935546875, 433.2556457519531, 96.55016326904297, 83.57292175292969)),
        # ('Dandellion', '77.86', (465.81317138671875, 367.474365234375, 152.96287536621094, 109.23639678955078))]
        plant_boxes, height_ratio, width_ratio = [], image.shape[0] / self.__height, image.shape[1] / self.__width
        for detection in detections:
            center_x, center_y, box_size_x, box_size_y = detection[2][0] * width_ratio, detection[2][1] * height_ratio,\
                                                         detection[2][2] * width_ratio, detection[2][3] * height_ratio
            top, left = center_y - box_size_y / 2, center_x - box_size_x / 2
            bot, right = top + box_size_y, left + box_size_x
            plant_boxes.append(DetectedPlantBox(round(left), round(top), round(right), round(bot), detection[0],
                                                self.__class_names.index(detection[0]),
                                                float(detection[1]), image.shape[1], image.shape[0],
                                                center_x=round(center_x), center_y=round(center_y)))

        if config.FRAME_SHOW and not disable_frame_show:

            t1 = time.time()

            #img = draw_boxes(image, plant_boxes)
            img = image

            if self.sharedArray is None:
                try:
                    sharedMemory = posix_ipc.SharedMemory(config.SHARED_MEMORY_NAME_DETECTED_FRAME, posix_ipc.O_CREX, size=img.nbytes)
                except posix_ipc.ExistentialError:
                    sharedMemory = posix_ipc.SharedMemory(config.SHARED_MEMORY_NAME_DETECTED_FRAME)
                sharedMem = mmap(fileno=sharedMemory.fd, length=img.nbytes)
                sharedMemory.close_fd()
                self.sharedArray = np.ndarray(img.shape, dtype=img.dtype, buffer=sharedMem)

            self.sharedArray[:] = img[:]

            draw_boxes(self.sharedArray, plant_boxes)

            #print(time.time() - t1)

            if not YoloDarknetDetector.WEBSTREAM:
                template_dir = os.path.abspath('./liveMain')
                app = Flask("webstreaming", template_folder=template_dir)
                CORS(app)
                app.add_url_rule('/', view_func=webstreaming.index)
                app.add_url_rule('/video_feed', view_func=webstreaming.video_feed)

                logging.getLogger('werkzeug').disabled = True
                os.environ['WERKZEUG_RUN_MAIN'] = 'true'

                YoloDarknetDetector.webStream = Process(target=app.run, args=("0.0.0.0",8888,False))
                YoloDarknetDetector.webStream.start()
                YoloDarknetDetector.WEBSTREAM = True
        
        return plant_boxes


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
