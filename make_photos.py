import adapters
import cv2 as cv
from config import config
import time
import datetime


def main():
    print("Loading...")
    sma = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
    if config.USE_X_AXIS_CALIBRATION and config.USE_Y_AXIS_CALIBRATION:
        sma.ext_align_cork_center(config.XY_F_MAX)
    pca = adapters.PiCameraAdapter()
    print("Loading complete.")

    dist = float(input("Move straight for (meters): "))
    dist *= 1000  # convert to mm
    dist = int(dist)

    # offset = int(input("Take image every (mm): "))
    offset = 75  # 151 mm is full AOI

    for B in range(0, dist, offset):
        image = pca.get_image()
        cv.imwrite("gathered_data/" + str(datetime.datetime.now()) + " - " + str(B) + ".jpg", image)

        sma.nav_move_forward(int(offset * config.ONE_MM_IN_SMOOTHIE), config.B_F_MAX)

        sma.wait_for_all_actions_done()
        # time.sleep(2) # if function above is not working properly

    # take last image
    image = pca.get_image()
    cv.imwrite("/gathered_data/" + str(datetime.datetime.now()) + " - last" + ".jpg", image)

    print("Done.")


if __name__ == "__main__":
    main()
