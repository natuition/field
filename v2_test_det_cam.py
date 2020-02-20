import detection
import cv2 as cv
import adapters
import datetime
import os


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
    detector = detection.YoloOpenCVDetection()
    img = cv.imread("1.jpg")
    i = 1

    while True:
        plants = detector.detect(img)
        print("Detected", len(plants), "plants on", str(i), "iteration")
        i += 1


def test_camera():
    i = 0
    with adapters.CameraAdapterIMX219_170() as camera:
        while True:
            image = camera.get_image()
            _ = image.shape[0]
            i += 1
            if i % 100 == 0:
                print(str(i), "frames read")


def test_camera_detection():
    detector = detection.YoloOpenCVDetection()
    i = 1

    with adapters.CameraAdapterIMX219_170() as camera:
        while True:
            frame = camera.get_image()
            plants = detector.detect(frame)
            print("Detected", len(plants), "plants on", str(i), "iteration")
            i += 1


def test_camera_detection_saving():
    output_path = "tests_output/"
    _create_directories(output_path)
    detector = detection.YoloOpenCVDetection()
    i = 1

    with adapters.CameraAdapterIMX219_170() as camera:
        while True:
            frame = camera.get_image()
            plants = detector.detect(frame)
            print("Detected", len(plants), "plants on", str(i), "iteration")
            i += 1
            cv.imwrite(output_path + str(datetime.datetime.now()) + ".jpg", frame)


if __name__ == '__main__':
    # should be called only one of them:
    #test_detection()
    #test_camera()
    #test_camera_detection()
    test_camera_detection_saving()
