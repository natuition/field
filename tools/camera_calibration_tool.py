from config import config
import adapters
import cv2 as cv
import time
import utility

OUTPUT_DIR = "camera_calibration_tool/"


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


def manual_photos_making(camera):
    draw_markup = input("Draw markup points on images? (y/n): ")
    draw_markup = draw_markup.lower() == "y"

    label = input("Please type label, which should be added to photos: ")
    sep = " "
    counter = 1
    path_piece = OUTPUT_DIR + label + sep

    while True:
        action = input("Hit enter to get an image, type anything to stop: ")
        if action != "":
            break

        frame = camera.get_image()

        if draw_markup == "y":
            frame = markup_5_points(frame)

        cv.imwrite(path_piece + str(counter) + sep + utility.get_current_time() + ".jpg", frame)
        counter += 1


def main():
    utility.create_directories(OUTPUT_DIR)

    print("Loading...")
    with adapters.CameraAdapterIMX219_170(0, 999999, 0, 999999, config.CV_ROTATE_CODE, 1, 1, 1, 1, 1000000, 1000000,
                                          config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                          config.CAMERA_H, config.CAMERA_FRAMERATE, config.CAMERA_FLIP_METHOD) \
            as camera:

        time.sleep(2)
        print("Loading complete.")
        manual_photos_making(camera)
        print("Done.")


if __name__ == "__main__":
    main()
