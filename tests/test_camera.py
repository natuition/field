import adapters
import cv2 as cv
import time

delay = 3
image_count = 20


def main():
    print("Loading camera...")
    with adapters.CameraAdapterIMX219_170_BS1() as cam:
        print("Camera warming up...")
        time.sleep(3)
        print("Loading done.")

        for i in range(image_count):
            print("Processing", str(i), "of", str(image_count), "image...")
            img = cam.get_image()
            cv.imwrite("test_camera_" + str(i) + ".jpg", img)
            time.sleep(delay)

        print("Done!")


if __name__ == '__main__':
    main()
