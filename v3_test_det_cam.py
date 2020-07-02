import detection
import cv2 as cv
import adapters
import datetime
import os
from config import config
import time


def _create_directories(*args):
    """Create two photos output directories: with detected plants, and without them"""

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


def test_detection():
    print("Loading detector...")

    detector = detection.YoloOpenCVDetection(config.PERIPHERY_CLASSES_FILE, config.PERIPHERY_CONFIG_FILE,
                                             config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_INPUT_SIZE,
                                             config.PERIPHERY_CONFIDENCE_THRESHOLD, config.PERIPHERY_NMS_THRESHOLD,
                                             config.PERIPHERY_DNN_BACKEND, config.PERIPHERY_DNN_TARGET)
    
    """
    detector = detection.YoloOpenCVDetection(config.PRECISE_CLASSES_FILE, config.PRECISE_CONFIG_FILE,
                                             config.PRECISE_WEIGHTS_FILE, config.PRECISE_INPUT_SIZE,
                                             config.PRECISE_CONFIDENCE_THRESHOLD, config.PRECISE_NMS_THRESHOLD
                                             config.PRECISE_DNN_BACKEND, config.PRECISE_DNN_TARGET)
    """
    img = cv.imread("1.jpg")
    i = 1

    while True:
        prev_time = time.time()
        plants = detector.detect(img)
        cur_time = time.time()
        print("Detected", len(plants), "plants on", i, "iteration in", cur_time - prev_time, "seconds")
        i += 1


def test_camera():
    i = 0
    time_list = []
    with adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                          config.CROP_H_TO, config.CV_ROTATE_CODE,
                                          config.ISP_DIGITAL_GAIN_RANGE_FROM, config.ISP_DIGITAL_GAIN_RANGE_TO,
                                          config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                          config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                          config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                          config.CAMERA_H, config.CAMERA_FRAMERATE, config.CAMERA_FLIP_METHOD) \
            as camera:
        print("Test started, info will be shown for each 100 taken frames")
        while True:
            prev_time = time.time()
            _ = camera.get_image()
            cur_time = time.time()
            time_list.append(cur_time - prev_time)
            i += 1
            if i % 100 == 0:
                print("Took", i, "frames, for these frames: Avg frame time is", sum(time_list) / len(time_list), "Min time is",
                      min(time_list), "Max time is", max(time_list))
                i = 0
                time_list.clear()


def test_camera_detection():
    detector = detection.YoloOpenCVDetection(config.PERIPHERY_CLASSES_FILE, config.PERIPHERY_CONFIG_FILE,
                                             config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_INPUT_SIZE,
                                             config.PERIPHERY_CONFIDENCE_THRESHOLD, config.PERIPHERY_NMS_THRESHOLD,
                                             config.PERIPHERY_DNN_BACKEND, config.PERIPHERY_DNN_TARGET)
    i = 1

    with adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                          config.CROP_H_TO, config.CV_ROTATE_CODE,
                                          config.ISP_DIGITAL_GAIN_RANGE_FROM, config.ISP_DIGITAL_GAIN_RANGE_TO,
                                          config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                          config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                          config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                          config.CAMERA_H, config.CAMERA_FRAMERATE, config.CAMERA_FLIP_METHOD) \
            as camera:
        print("Test started, time values are in seconds")
        while True:
            start_time = time.time()
            frame = camera.get_image()
            frame_time = time.time()
            plants = detector.detect(frame)
            detector_time = time.time()

            print("Frame time is", frame_time - start_time, "Detection time is", detector_time - frame_time, "Detected",
                  len(plants), "plants on", i, "iteration")
            i += 1


def test_camera_detection_saving():
    output_path = "tests_output/"
    _create_directories(output_path)
    detector = detection.YoloOpenCVDetection(config.PERIPHERY_CLASSES_FILE, config.PERIPHERY_CONFIG_FILE,
                                             config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_INPUT_SIZE,
                                             config.PERIPHERY_CONFIDENCE_THRESHOLD, config.PERIPHERY_NMS_THRESHOLD,
                                             config.PERIPHERY_DNN_BACKEND, config.PERIPHERY_DNN_TARGET)
    i = 1

    with adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                          config.CROP_H_TO, config.CV_ROTATE_CODE,
                                          config.ISP_DIGITAL_GAIN_RANGE_FROM, config.ISP_DIGITAL_GAIN_RANGE_TO,
                                          config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                          config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                          config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                          config.CAMERA_H, config.CAMERA_FRAMERATE, config.CAMERA_FLIP_METHOD) \
            as camera:
        print("Test started, time values are in seconds")
        while True:
            start_time = time.time()
            frame = camera.get_image()
            frame_time = time.time()
            plants = detector.detect(frame)
            detection_time = time.time()
            cv.imwrite(output_path + str(datetime.datetime.now()) + ".jpg", frame)
            saving_time = time.time()

            print("Frame time is", frame_time - start_time, "Detection time is", detection_time - frame_time,
                  "Image saving time is", saving_time - detection_time, "Detected", len(plants), "plants on", i,
                  "iteration")
            i += 1


def test_saving(images_to_save):
    output_path = "tests_output/"
    _create_directories(output_path)
    image = cv.imread("1.jpg")
    times_list = []

    for i in range(images_to_save):
        start_time = time.time()
        cv.imwrite(output_path + str(datetime.datetime.now()) + ".jpg", image)
        save_time = time.time()
        times_list.append(save_time - start_time)

        if i % 100 == 0:
            print("Saved", i, "of", images_to_save, "images, for last 100 images: Avg save time is", sum(times_list) /
                  len(times_list), "Min save time is", min(times_list), "Max save time is", max(times_list))
            times_list.clear()


if __name__ == '__main__':
    # should be called only one of them:
    test_detection()
    # test_camera()
    # test_camera_detection()
    # test_camera_detection_saving()
    # test_saving(images_to_save=2000)
