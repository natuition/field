from config import config
import adapters
import detection
import cv2 as cv
import time
import utility

OUTPUT_DIR = "manual_photos_maker/"

def generateGstConfig():
    aelock = "aelock=true " if config.AE_LOCK else ""

    gst_config_start = (
        "nvarguscamerasrc "
        "ispdigitalgainrange=\"%.2f %.2f\" "
        "gainrange=\"%.2f %.2f\" "
        "exposuretimerange=\"%d %d\" "
        "%s"
        "! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        
        % (
            config.ISP_DIGITAL_GAIN_RANGE_FROM,
            config.ISP_DIGITAL_GAIN_RANGE_TO,
            config.GAIN_RANGE_FROM,
            config.GAIN_RANGE_TO,
            config.EXPOSURE_TIME_RANGE_FROM,
            config.EXPOSURE_TIME_RANGE_TO,
            aelock,
            config.CAMERA_W,
            config.CAMERA_H,
            config.CAMERA_FRAMERATE,
            
        )
    )

    if config.APPLY_IMAGE_CROPPING:
        gst_config_end = (
                "nvvidconv top=%d bottom=%d left=%d right=%d flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
            % (
                config.CROP_H_FROM,
                config.CROP_H_TO,
                config.CROP_W_FROM,
                config.CROP_W_TO,
                config.CAMERA_FLIP_METHOD,
                config.CROP_W_TO-config.CROP_W_FROM,
                config.CROP_H_TO-config.CROP_H_FROM
            )
        )
    else:
        gst_config_end = (
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                config.CAMERA_FLIP_METHOD,
                config.CAMERA_W,
                config.CAMERA_H
            )
        )
    
    return gst_config_start+gst_config_end

def manual_photos_making(camera):
    action = input("Hit enter to get an image, type anything to stop: ")
    if action != "":
        return None

    frame = camera.get_image()

    return frame

def main():
    utility.create_directories(OUTPUT_DIR)

    print("Loading...")
    with \
        adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                          config.CROP_H_TO, config.CV_ROTATE_CODE,
                                          config.ISP_DIGITAL_GAIN_RANGE_FROM, config.ISP_DIGITAL_GAIN_RANGE_TO,
                                          config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                          config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                          config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                          config.CAMERA_H, config.CAMERA_FRAMERATE, config.CAMERA_FLIP_METHOD) \
            as camera :
        
        detector = detection.YoloTRTDetector( config.PERIPHERY_MODEL_PATH,
                config.PERIPHERY_CLASSES_FILE,
                config.PERIPHERY_CONFIDENCE_THRESHOLD,
                config.PERIPHERY_NMS_THRESHOLD,
                config.PERIPHERY_INPUT_SIZE)

        time.sleep(2)

        print("Loading complete.")

        frame = manual_photos_making(camera)

        if frame is not None:
            try:
                count = 0
                while True:
                    plants_boxes = detector.detect(frame, True)
                    print(f"{count} | {len(plants_boxes)}")
                    count +=1
            except KeyboardInterrupt:
                pass
        


if __name__ == "__main__":
    main()
