from config import config
import adapters
import cv2 as cv
import time
import utility

OUTPUT_DIR = "manual_photos_maker/"


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

        if draw_markup:
            frame = cv.circle(frame, (config.SCENE_CENTER_X, config.SCENE_CENTER_Y), 2, (0, 0, 255), thickness=2)
            # frame = markup_5_points(frame)

        cv.imwrite(path_piece + str(counter) + sep + utility.get_current_time() + ".jpg", frame)
        counter += 1


def run_performance_test(camera):
    label = input("Please type label, which should be added to photos: ")
    sep = " "
    counter = 1
    path_piece = OUTPUT_DIR + label + sep
    saving_time_delta = "None"

    try:
        while True:
            frame = camera.get_image()
            cur_time = utility.get_current_time()  # timestamp when frame was read

            saving_start_t = time.time()
            cv.imwrite(path_piece +
                       str(counter) +
                       sep +
                       cur_time +
                       # sep +
                       # "(prev. imwrite time " +
                       # str(saving_time_delta) +
                       # " seconds)" +
                       ".jpg",
                       frame)
            # saving_time_delta = time.time() - saving_start_t

            counter += 1
    except KeyboardInterrupt:
        return


def main():
    utility.create_directories(OUTPUT_DIR)

    print("Loading...")
    with adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                          config.CROP_H_TO, config.CV_ROTATE_CODE,
                                          config.ISP_DIGITAL_GAIN_RANGE_FROM, config.ISP_DIGITAL_GAIN_RANGE_TO,
                                          config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                          config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                          config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                          config.CAMERA_H, config.CAMERA_FRAMERATE, config.CAMERA_FLIP_METHOD) \
            as camera:

        time.sleep(2)

        print("Loading complete.")

        mode = input("Type y to start in manual photos making mode, type n to start performance test (y/n): ")
        if mode.lower() == "y":
            manual_photos_making(camera)
        else:
            run_performance_test(camera)

        print("Done.")


if __name__ == "__main__":
    main()
