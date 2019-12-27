import adapters
import cv2 as cv
import datetime
import os
import time


def markup_5_points(image):
    img_y_c, img_x_c = int(image.shape[0] / 2), int(image.shape[1] / 2)
    # center
    image = cv.circle(image, (img_x_c, img_y_c), 3, (0, 0, 255), thickness=3)
    # top
    image = cv.circle(image, (img_x_c, img_y_c - 400), 3, (0, 0, 255), thickness=3)
    # bottom
    image = cv.circle(image, (img_x_c, img_y_c + 400), 3, (0, 0, 255), thickness=3)
    # left
    image = cv.circle(image, (img_x_c - 400, img_y_c), 3, (0, 0, 255), thickness=3)
    # right
    image = cv.circle(image, (img_x_c + 400, img_y_c), 3, (0, 0, 255), thickness=3)
    return image


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
    with adapters.CameraAdapterIMX219_170() as camera:
        time.sleep(2)
        print("Loading complete.")

        draw_markup = input(
            "Type t to draw markup points on images, type empty string (just hit enter key) to leave images clean: ")
        draw_markup = True if draw_markup == "t" else False

        label = input("Please type label, which should be added to photos: ")

        sep = " "
        counter = 1

        while True:
            action = input("Type empty string (just hit enter key) to get an image, type anything to stop: ")
            if action != "":
                break

            image = camera.get_image()
            if draw_markup:
                image = markup_5_points(image)
            cv.imwrite(output_dir + label + sep + str(counter) + sep + str(datetime.datetime.now()) + ".jpg", image)
            counter += 1

        print("Done.")


if __name__ == "__main__":
    main()
