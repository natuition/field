import adapters
import cv2 as cv
import time

delay = 3
image_count = 20


def main():
    cam = adapters.CameraAdapterIMX219_170()

    for i in range(image_count):
        img = cam.get_image()
        cv.imwrite("test_camera_" + str(i) + ".jpg", img)
        time.sleep(delay)


if __name__ == '__main__':
    main()
