import sys
import onnx
import os
import argparse
import numpy as np
import cv2
import onnxruntime

from tool.utils import *
from tool.darknet2onnx import *


def main(cfg_file, namesfile, weight_file, image_path, network_name="yolov4_-1_3", batch_size=1):

    if batch_size <= 0:
        onnx_path_demo = transform_to_onnx(cfg_file, weight_file, batch_size, network_name)
    else:
        # Transform to onnx as specified batch size
        transform_to_onnx(cfg_file, weight_file, batch_size, network_name)
        # Transform to onnx as demo
        onnx_path_demo = transform_to_onnx(cfg_file, weight_file, 1, network_name)

    session = onnxruntime.InferenceSession(onnx_path_demo, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
    # session = onnx.load(onnx_path)
    print("The model expects input shape: ", session.get_inputs()[0].shape)

    outputs = session.get_outputs()
    print(f'The model has {len(outputs)} outputs')
    for outp in outputs:
        print(f'The model delivers output shape: {outp.shape}')

    image_src = cv2.imread(image_path)
    detect(session, image_src, namesfile, onnx_path_demo.replace(".onnx",".jpg"))

    return onnx_path_demo



def detect(session, image_src, namesfile, image_path_out):
    IN_IMAGE_H = session.get_inputs()[0].shape[2]
    IN_IMAGE_W = session.get_inputs()[0].shape[3]

    # Input
    resized = cv2.resize(image_src, (IN_IMAGE_W, IN_IMAGE_H), interpolation=cv2.INTER_LINEAR)
    img_in = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    img_in = np.transpose(img_in, (2, 0, 1)).astype(np.float32)
    img_in = np.expand_dims(img_in, axis=0)
    img_in /= 255.0
    print("Shape of the network input: ", img_in.shape)

    # Compute
    input_name = session.get_inputs()[0].name

    outputs = session.run(None, {input_name: img_in})

    boxes = post_processing(img_in, 0.4, 0.6, outputs)

    class_names = load_class_names(namesfile)
    plot_boxes_cv2(image_src, boxes[0], savename=image_path_out, class_names=class_names)



if __name__ == '__main__':
    print("Converting to onnx and running demo ...")
    cfg_file = sys.argv[1]
    namesfile = sys.argv[2]
    weight_file = sys.argv[3]
    image_path = sys.argv[4]
    main(cfg_file, namesfile, weight_file, image_path)