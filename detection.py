import cv2 as cv
import numpy as np
import os
from config import config


# Load names of classes
def load_class_names(labels_file):
    with open(labels_file, 'rt') as f:
        return f.read().rstrip('\n').split('\n')


# Get the names of the output layers
def get_output_names(net):
    # Get the names of all the layers in the network
    layers_names = net.getLayerNames()
    # Get the names of the output layers, i.e. the layers with unconnected outputs
    return [layers_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]


# Draw the predicted bounding box
def draw_bbox(image, classes, class_id, conf, left, top, right, bottom):
    # Draw a bounding box.
    cv.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 3)
    # Draw a center of that box
    cv.circle(image, (int(left + (right - left) / 2), int(top + (bottom - top) / 2)), 4, (0, 0, 255), thickness=3)

    label = '%.2f' % conf

    # Get the label for the class name and its confidence
    if classes:
        assert (class_id < len(classes))
        label = '%s:%s' % (classes[class_id], label)

    # Display the label at the top of the bounding box
    label_size, base_line = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    top = max(top, label_size[1])
    cv.rectangle(image, (left, top - round(1.5 * label_size[1])), (left + round(1.5 * label_size[0]), top + base_line),
                 (0, 0, 255), cv.FILLED)
    cv.putText(image, label, (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 2)


# Remove the bounding boxes with low confidence using non-maxima suppression
def post_process(image, outs, classes):
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

            if confidence > config.CONFIDENCE_THRESHOLD:
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
    indices = cv.dnn.NMSBoxes(boxes, confidences, config.CONFIDENCE_THRESHOLD, config.NMS_THRESHOLD)

    for i in indices:
        i = i[0]
        box = boxes[i]
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]
        final_plant_boxes.append(DetectedPlantBox(left, top, left + width, top + height, classes, class_ids[i],
                                                  confidences[i]))
    return final_plant_boxes


def detect(image, net, classes):
    # Create a 4D blob from a frame.
    blob = cv.dnn.blobFromImage(image, 1 / 255, config.INPUT_SIZE, [0, 0, 0], 1, crop=False)

    # Sets the input to the network
    net.setInput(blob)

    # Runs the forward pass to get output of the output layers
    out_layers = get_output_names(net)
    outs = net.forward(out_layers)

    # Remove the bounding boxes with low confidence
    return post_process(image, outs, classes)


def main():
    if not os.path.exists(config.OUTPUT_IMG_DIR):
        try:
            os.mkdir(config.OUTPUT_IMG_DIR)
        except OSError:
            print("Creation of the directory %s failed" % config.OUTPUT_IMG_DIR)
        else:
            print("Successfully created the directory %s " % config.OUTPUT_IMG_DIR)

    classes = load_class_names(config.YOLO_CLASSES_FILE)
    net = cv.dnn.readNetFromDarknet(config.YOLO_CONFIG_FILE, config.YOLO_WEIGHTS_FILE)
    net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

    img = cv.imread(config.INPUT_IMG_DIR + config.INPUT_IMG_FILE)
    boxes = detect(img, net, classes)

    for i in range(len(boxes)):
        left, top, right, bottom = boxes[i].get_box_points()
        draw_bbox(img, classes, boxes[i].get_class_id(), boxes[i].get_confidence(), left, top, right, bottom)

    cv.imwrite(config.OUTPUT_IMG_DIR + "Result " + config.INPUT_IMG_FILE, img)


class YoloOpenCVDetection:
    pass


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

    def get_box_points(self):
        return self._left, self._top, self._right, self._bottom

    def get_center_point(self):
        return self._center_x, self._center_y

    def get_sizes(self):
        return self._right - self._left, self._bottom - self._top

    def get_label(self):
        return self._classes[self._class_id]

    def get_class_id(self):
        return self._class_id

    def get_confidence(self):
        return self._confidence


if __name__ == '__main__':
    main()
