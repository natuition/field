import adapters
import cv2 as cv


def main():
    cam = adapters.CameraAdapterIMX219_170()

    for i in range(3):
        input("Press enter to get an image")
        img = cam.get_image()
        cv.imwrite("test_camera_" + str(i) + ".jpg", img)


if __name__ == '__main__':
    main()
