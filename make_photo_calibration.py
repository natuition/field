from config import config
import adapters
import cv2 as cv
import time
import utility

OUTPUT_DIR = ""

def photo_making(camera):
	
	frame = camera.get_image()
	path_piece = OUTPUT_DIR + "calibration"
	cv.imwrite(path_piece + ".jpg", frame)
	
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
        photo_making(camera)
        print("Done.")


if __name__ == "__main__":
    main()