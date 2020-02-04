import detection
import cv2 as cv
import adapters


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


def test_both():
    detector = detection.YoloOpenCVDetection()
    i = 1

    with adapters.CameraAdapterIMX219_170() as camera:
        while True:
            frame = camera.get_image()
            plants = detector.detect(frame)
            print("Detected", len(plants), "plants on", str(i), "iteration")
            i += 1


if __name__ == '__main__':
    # should be called only one of them:
    test_detection()
    #test_camera()
    #test_both()
