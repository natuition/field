from config import config
import detection
import adapters
import os
import logging
import datetime
import multiprocessing

# paths
LOG_DIR = "log/"
LOG_FILE = "v2_test_autonomy.log"

# logging settings
logging.basicConfig(format='%(asctime)s > %(module)s.%(funcName)s %(levelname)s: %(message)s (line %(lineno)d)',
                    datefmt='%I:%M:%S %p',
                    filename=LOG_DIR + LOG_FILE,
                    filemode='w',
                    level=logging.DEBUG)


def detection_process():
    """This function is running as separate process, simulates YOLO and camera work"""

    detector = detection.YoloOpenCVDetection()
    with adapters.CameraAdapterIMX219_170() as camera:
        while True:
            frame = camera.get_image()
            plants = detector.detect(frame)
            _ = len(plants)


def do_extractions(smoothie: adapters.SmoothieAdapter):
    for y in [100, 150]:
        for x in range(10, 400, 40):
            log_msg = "Moving to a plant coordinates at X=" + str(x) + " Y=" + str(y)
            print(log_msg)
            logging.debug(log_msg)

            # move to a plant
            res = smoothie.custom_move_to(config.XY_F_MAX, X=x, Y=y)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                log_msg = "Couldn't move cork over plant, smoothie error occurred: " + str(res)
                print(log_msg)
                logging.critical(log_msg)
                # exit(1)

            # extraction, cork down
            log_msg = "Extracting plant (cork down)"
            print(log_msg)
            logging.info(log_msg)

            res = smoothie.custom_move_for(config.Z_F_MAX, Z=-30)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                log_msg = "Couldn't move the extractor down, smoothie error occurred:" + str(res)
                print(log_msg)
                logging.critical(log_msg)
                # exit(1)

            # extraction, cork up
            log_msg = "Extracting plant (cork up)"
            print(log_msg)
            logging.info(log_msg)

            res = smoothie.ext_cork_up()
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                log_msg = "Couldn't move the extractor up, smoothie error occurred:" + str(res)
                print(log_msg)
                logging.critical(log_msg)
                # exit(1)


def move_forward(smoothie: adapters.SmoothieAdapter):
    log_msg = "Moving forward for 30 cm"
    print(log_msg)
    logging.info(log_msg)

    # move forward for 30 cm
    res = smoothie.custom_move_for(config.B_F_MAX, B=5.43)
    smoothie.wait_for_all_actions_done()
    if res != smoothie.RESPONSE_OK:
        log_msg = "Couldn't move forward (for 30 cm), smoothie error occurred: " + str(res)
        print(log_msg)
        logging.critical(log_msg)
        # exit(1)


def main():
    if not os.path.exists(LOG_DIR):
        try:
            os.mkdir(LOG_DIR)
        except OSError:
            print("Creation of the directory %s failed" % LOG_DIR)
            logging.error("Creation of the directory %s failed" % LOG_DIR)
        else:
            print("Successfully created the directory %s " % LOG_DIR)
            logging.info("Successfully created the directory %s " % LOG_DIR)

    # create smoothieboard adapter (API for access and control smoothieboard)
    while True:
        try:
            smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
            log_msg = "Successfully connected to smoothie"
            print(log_msg)
            logging.info(log_msg)
            break
        except OSError as error:
            logging.warning(repr(error))
            print(repr(error))

    detector_simulation = multiprocessing.Process(target=detection_process)
    detector_simulation.start()

    log_msg = "Started working at " + str(str(datetime.datetime.now()).split(".")[:-1])[2:-2].replace(":", "-")
    print(log_msg)
    logging.info(log_msg)
    
    while True:
        do_extractions(smoothie)
        move_forward(smoothie)


if __name__ == "__main__":
    main()
