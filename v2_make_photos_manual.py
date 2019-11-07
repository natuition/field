import adapters
import cv2 as cv
import datetime
import os
import time


def main():
    output_dir = "gathered_data/"
    if not os.path.exists(output_dir):
        try:
            os.mkdir(output_dir)
        except OSError:
            print("Creation of the directory %s failed" % output_dir)
        else:
            print("Successfully created the directory %s " % output_dir)

    print("Loading...")
    camera = adapters.CameraAdapterIMX219_170()
    time.sleep(2)
    print("Loading complete.")

    label = input("Please type label, which should be added to photos: ")
    sep = " "
    counter = 1

    while True:
        action = input("Type empty string (just hit enter key) to get an image, type anything to stop: ")
        if action != "":
            break

        image = camera.get_image()
        cv.imwrite(output_dir + label + sep + str(counter) + sep + str(datetime.datetime.now()) + ".jpg", image)
        counter += 1

    print("Done.")


if __name__ == "__main__":
    main()
