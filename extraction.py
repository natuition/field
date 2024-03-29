from config import config
import adapters
import detection
import math
import utility
import datacollection
from matplotlib.patches import Polygon
import time
import pickle
from sklearn.preprocessing import PolynomialFeatures


class ExtractionManagerV3:
    """Implements extraction logic and control"""

    def __init__(self,
                 smoothie: adapters.SmoothieAdapter,
                 camera: adapters.CameraAdapterIMX219_170,
                 logger_full: utility.Logger,
                 data_collector: datacollection.DataCollector,
                 image_saver: utility.ImageSaver,
                 log_cur_dir,
                 periphery_det: detection.YoloOpenCVDetection,  # not used atm
                 precise_det: detection.YoloOpenCVDetection,
                 camera_positions: list,
                 pdz_distances: list,
                 vesc_engine: adapters.VescAdapterV4):

        self.__smoothie = smoothie
        self.__camera = camera
        self.__logger_full = logger_full
        self.__data_collector = data_collector
        self.__image_saver = image_saver
        self.__log_cur_dir = log_cur_dir
        self.__periphery_det = periphery_det
        self.__precise_det = precise_det
        self.__camera_positions = camera_positions
        self.__pdz_polygons = self.pdz_dist_to_poly(pdz_distances)
        self.__pdz_cv_rects = self.pdz_dist_to_rect_cv(pdz_distances)
        self.__extraction_map = ExtractionMap(config.EXTRACTION_MAP_CELL_SIZE_MM)
        # self.__converter = PxToMMConverter()
        self.__converter = PxToMmPredictorCP(
            config.SCENE_CENTER_X,
            config.SCENE_CENTER_Y,
            config.CONTROL_POINTS_CSV_PATH)
        self.__vesc_engine = vesc_engine

        # check if PDZ data is correct
        if len(self.__camera_positions) != len(self.__pdz_polygons) != len(self.__pdz_cv_rects):
            msg = f"PDZ data is not correct: camera positions ({len(self.__camera_positions)} items), " \
                  f"pdz polygons ({len(self.__pdz_polygons)} items) " \
                  f"and pdz cv polygons ({len(self.__pdz_cv_rects)} items) lists are not the same length"
            raise ValueError(msg)

        # demo pauses
        if config.ALLOW_DEMO_PAUSES:
            self.__demo_server = utility.DemoPauseServer(config.DEMO_PAUSES_HOST, config.DEMO_PAUSES_PORT)
        else:
            self.__demo_server: utility.DemoPauseServer = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

    @staticmethod
    def pdz_dist_to_poly(pdz_zones: list):
        """Converts related to scene center PDZ distances into px coordinates and returns that area as list of
        matplotlib.patches.Polygon items.
        """

        pdz_distances_poly = []
        for pdz_zone in pdz_zones:
            pdz_distances_poly.append(Polygon([
                [config.SCENE_CENTER_X - pdz_zone["left"], config.SCENE_CENTER_Y - pdz_zone["top"]],
                [config.SCENE_CENTER_X + pdz_zone["right"], config.SCENE_CENTER_Y - pdz_zone["top"]],
                [config.SCENE_CENTER_X + pdz_zone["right"], config.SCENE_CENTER_Y + pdz_zone["bot"]],
                [config.SCENE_CENTER_X - pdz_zone["left"], config.SCENE_CENTER_Y + pdz_zone["bot"]]
            ]))
        return pdz_distances_poly

    @staticmethod
    def pdz_dist_to_rect_cv(pdz_zones: list):
        """Converts related to scene center PDZ distances for pdz zone display on images with opencv. Returns list of
        converted zones.
        """

        pdz_zones_cv = []
        for pdz_zone in pdz_zones:
            pdz_zones_cv.append([
                (config.SCENE_CENTER_X - pdz_zone["left"], config.SCENE_CENTER_Y - pdz_zone["top"]),
                (config.SCENE_CENTER_X + pdz_zone["right"], config.SCENE_CENTER_Y + pdz_zone["bot"])
            ])
        return pdz_zones_cv

    def scan_sectors(self):
        """Detects plants under robot's working zone, optimizes extractions order and returns list of plants as smoothie
        absolute coordinates (coordinates out of working range are skipped).
        """

        pdz_start_t = time.time()

        smoothie_plants_positions = []

        # loop over camera positions (cam positions, pdz zones and pdz cv zones lists must have a same length)
        for i in range(len(self.__camera_positions)):
            cam_sm_x, cam_sm_y = self.__camera_positions[i]

            # move cork to camera position
            res = self.__smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                             Y_F=config.Y_F_MAX,
                                                             X=cam_sm_x,
                                                             Y=cam_sm_y)
            self.__smoothie.wait_for_all_actions_done()
            if res != self.__smoothie.RESPONSE_OK:
                msg = f"Could not move cork to camera position (x={cam_sm_x}, y={cam_sm_y}) - \
                smoothie error occurred:\n" + res
                self.__logger_full.write(msg + "\n")
                continue

            # take a photo and look for a plants
            time.sleep(config.DELAY_BEFORE_2ND_SCAN)
            frame = self.__camera.get_image()
            plants_boxes = self.__precise_det.detect(frame)
            # get plants boxes and keep only that are in PDZ
            cur_pos_plant_boxes_pdz = list(filter(
                lambda box: self.is_point_in_poly(
                    box.center_x,
                    box.center_y,
                    self.__pdz_polygons[i]),
                plants_boxes
            ))

            if config.SAVE_DEBUG_IMAGES:
                frame = utility.ImageSaver.draw_data_in_frame(
                    frame,
                    pdz_cv_rect=self.__pdz_cv_rects[i],
                    plants_boxes=cur_pos_plant_boxes_pdz)
                self.__image_saver.save_image(
                    frame,
                    config.DEBUG_IMAGES_PATH,
                    label=f"PR_PDZ_view_at_{round(cam_sm_x, 1)}_{round(cam_sm_y, 1)}",
                    plants_boxes=plants_boxes)

            # convert plants boxes px coordinates into absolute smoothie coordinates
            for plant_box in cur_pos_plant_boxes_pdz:
                # calculate values to move camera over a plant from current position (coords are relative)
                # rel_sm_x, rel_sm_y = self.__converter.convert_px_to_mm(plant_box.center_x, plant_box.center_y)
                rel_sm_x, rel_sm_y, cp_idx = self.__converter.predict(plant_box.center_x, plant_box.center_y)

                # convert smoothie relative coordinates to absolute
                cur_sm_pos = self.__smoothie.get_adapter_current_coordinates()
                abs_sm_x, abs_sm_y = cur_sm_pos["X"] + float(rel_sm_x), cur_sm_pos["Y"] + float(rel_sm_y)

                # skip coordinates that are out of working range
                if not (config.X_MIN < abs_sm_x < config.X_MAX and config.Y_MIN < abs_sm_y < config.Y_MAX):
                    continue

                # add absolute coordinates to the result list
                smoothie_plants_positions.append((abs_sm_x, abs_sm_y))

        sorted_smoothie_plants_positions = self.optimize_corkscrew_way(smoothie_plants_positions)
        self.__data_collector.add_pdz_scan_t(time.time() - pdz_start_t)
        return sorted_smoothie_plants_positions

    def extract_all_plants(self):
        """Find and extract all plants found in current robot's position
        """

        extract_all_plants_start_t = time.time()

        self.__extraction_map.clear()

        if config.VERBOSE_EXTRACT:
            msg = "[VERBOSE EXTRACT] Run PDZ scan..."
            self.__logger_full.write_and_flush(msg+"\n")
        # do sectored scans
        smoothie_positions = self.scan_sectors()
        if config.VERBOSE_EXTRACT:
            msg = "[VERBOSE EXTRACT] Found " + str(len(smoothie_positions)) + " plants after PDZ scan"
            self.__logger_full.write_and_flush(msg + "\n")
            if config.VERBOSE:
                print(msg)
        # round coords before logging them
        if len(smoothie_positions) != 0:
            log_sm_positions = list(map(lambda item: (round(item[0], 2), round(item[1], 2)), smoothie_positions))
            msg = "PDZ plants smoothie coordinates:\n" + str(log_sm_positions)
            self.__logger_full.write(msg + "\n")
            if config.VERBOSE:
                print(msg)

        if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
            msg = f"EXT. PAUSE: Found {str(len(smoothie_positions))} plants after PDZ scan; press enter:"
            self.__logger_full.write(msg + "\n")
            input(msg)
        # demo pause
        if config.ALLOW_DEMO_PAUSES and self.__demo_server is not None:
            self.__demo_server.wait_for_resume_cmd()

        plant_index = 0
        # loop over plants that were detected during PDZ sectored scans and extract them (main ext loop)
        for init_pos_sm_x, init_pos_sm_y in smoothie_positions:
            plant_index += 1
            if config.VERBOSE_EXTRACT:
                msg = f"[VERBOSE EXTRACT] Start extraction for plant {plant_index} of {len(smoothie_positions)}."
                self.__logger_full.write_and_flush(msg+"\n")
            cur_pos_sm_x, cur_pos_sm_y = init_pos_sm_x, init_pos_sm_y

            if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
                msg = f"EXT. PAUSE: Starting plant {str(plant_index)} of {str(len(smoothie_positions))} extraction."
                self.__logger_full.write(msg + "\n")
                print(msg)
                msg = f"EXT. PAUSE: Target is X={str(cur_pos_sm_x)}, Y={str(cur_pos_sm_y)}; press enter:"
                self.__logger_full.write(msg + "\n")
                input(msg)
            # demo pause
            if config.ALLOW_DEMO_PAUSES and self.__demo_server is not None:
                self.__demo_server.wait_for_resume_cmd()

            # modify coordinates to try to avoid corkscrew tube view obscuring
            if config.AVOID_CORK_VIEW_OBSCURING:
                # determine shift direction depending at which working area quarter target is
                obscuring_offset_x = config.AVOID_CORK_VIEW_OBSCURING_DIST_X if init_pos_sm_x < config.X_MAX / 2 \
                    else -config.AVOID_CORK_VIEW_OBSCURING_DIST_X
                if config.X_MIN < init_pos_sm_x + obscuring_offset_x < config.X_MAX:
                    cur_pos_sm_x += obscuring_offset_x
                    if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
                        msg = f"EXT. PAUSE: Due to obscuring avoidance target has changed to " \
                              f"X={str(cur_pos_sm_x)}, Y={str(cur_pos_sm_y)}; press enter:"
                        self.__logger_full.write(msg + "\n")
                        input(msg)
                if config.Y_MIN < init_pos_sm_y + config.AVOID_CORK_VIEW_OBSCURING_DIST_Y < config.Y_MAX:
                    cur_pos_sm_y += config.AVOID_CORK_VIEW_OBSCURING_DIST_Y
                    if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
                        msg = f"EXT. PAUSE: Due to obscuring avoidance target has changed to " \
                              f"X={str(cur_pos_sm_x)}, Y={str(cur_pos_sm_y)}; press enter:"
                        self.__logger_full.write(msg + "\n")
                        input(msg)

            # affects robot's behaviour if no plants were detected; responsible for delta scans and extractions checking
            scan_is_first = True

            # this loop determines plant (or plants group if they were not detected during PDZ) extractions tries count
            for _ in range(config.EXTRACTION_TRIES_PER_PLANT):
                movement_messages = [
                    "Failed to move cork to the plant rescan position, smoothie's response:\n",
                    "Failed to move cork to the plant rescan position Y_MIN (after calibration), smoothie's response:\n"
                ]
                # try to move to the plant rescan position, calibrate and try again if failed, exit app otherwise
                ext_xy_start_t = time.time()
                for idx, movement_message in enumerate(movement_messages):
                    res = self.__smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                                     Y_F=config.Y_F_MAX,
                                                                     X=cur_pos_sm_x,
                                                                     Y=cur_pos_sm_y)
                    self.__smoothie.wait_for_all_actions_done()
                    if res == self.__smoothie.RESPONSE_OK:
                        break
                    else:
                        msg = movement_message + res
                        self.__logger_full.write(msg + "\n")

                        # try calibration (only once, at first movement fail)
                        if idx != 0:
                            break
                        msg = "Trying to calibrate cork"
                        self.__logger_full.write(msg + "\n")
                        res = self.__smoothie.ext_calibrate_cork()
                        if res != self.__smoothie.RESPONSE_OK:
                            msg = "Failed to calibrate cork, smoothie's response:\n" + res
                            self.__logger_full.write(msg + "\n")
                            self.__data_collector.add_all_ext_xy_t(time.time() - ext_xy_start_t)
                            exit(1)
                self.__data_collector.add_all_ext_xy_t(time.time() - ext_xy_start_t)

                if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
                    msg = f"EXT. PAUSE: Arrived to plant {str(plant_index)}, preparing to specify scan; press enter:"
                    self.__logger_full.write(msg + "\n")
                    input(msg)

                # make a scan, keep only plants that are in undistorted zone
                # TODO: possibly here will be multiple scans with average coordinates
                ext_img_start_t = time.time()
                time.sleep(config.DELAY_BEFORE_2ND_SCAN)
                frame = self.__camera.get_image()
                plants_boxes = self.__precise_det.detect(frame)
                self.__data_collector.add_all_ext_img_t(time.time() - ext_img_start_t)
                cur_pos_plant_boxes_undist = list(filter(
                    lambda plant_box_1: self.is_point_in_circle(
                        plant_box_1.center_x,
                        plant_box_1.center_y,
                        config.SCENE_CENTER_X,
                        config.SCENE_CENTER_Y,
                        config.UNDISTORTED_ZONE_RADIUS),
                    plants_boxes
                ))

                if config.SAVE_DEBUG_IMAGES:
                    if len(cur_pos_plant_boxes_undist) > 0:
                        frame = utility.ImageSaver.draw_data_in_frame(
                            frame,
                            undistorted_zone_radius=config.UNDISTORTED_ZONE_RADIUS,
                            plants_boxes=cur_pos_plant_boxes_undist)
                        self.__image_saver.save_image(
                            frame,
                            config.DEBUG_IMAGES_PATH,
                            label=f"(PR_view_at_{round(cur_pos_sm_x, 1)}_{round(cur_pos_sm_y, 1)}),weeds_in_undist",
                            plants_boxes=cur_pos_plant_boxes_undist)
                    else:
                        frame = utility.ImageSaver.draw_data_in_frame(
                            frame,
                            undistorted_zone_radius=config.UNDISTORTED_ZONE_RADIUS,
                            plants_boxes=plants_boxes)
                        self.__image_saver.save_image(
                            frame,
                            config.DEBUG_IMAGES_PATH,
                            label=f"(PR_view_at_{round(cur_pos_sm_x, 1)}_{round(cur_pos_sm_y, 1)}),no_weeds_in_undist",
                            plants_boxes=plants_boxes)

                if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
                    msg = f"EXT. PAUSE: Found {str(len(cur_pos_plant_boxes_undist))} plants in undistorted zone " \
                          f"during specify scan (saved in debug images if allowed, going to do delta scans if 0 and" \
                          f" allowed); press enter:"
                    self.__logger_full.write(msg + "\n")
                    input(msg)
                # demo pause
                if config.ALLOW_DEMO_PAUSES and self.__demo_server is not None:
                    self.__demo_server.wait_for_resume_cmd()

                # do rescan using delta seeking if nothing detected, it was 1rst scan and delta seeking is allowed
                if len(cur_pos_plant_boxes_undist) == 0:
                    if scan_is_first:
                        msg = "No plants detected (plant was in working zone before)"
                        self.__logger_full.write(msg + "\n")
                        if config.VERBOSE:
                            print(msg)

                        if config.ALLOW_DELTA_SEEKING:
                            delta_start_t = time.time()
                            delta_shifts_count = 0

                            msg = "Trying to do delta scans and find this plant"
                            self.__logger_full.write(msg + "\n")

                            if config.DELTA_SEEKING_IGNORE_OBSCURING:
                                dl_sm_init_x, dl_sm_init_y = init_pos_sm_x, init_pos_sm_y
                            else:
                                dl_sm_init_x, dl_sm_init_y = cur_pos_sm_x, cur_pos_sm_y

                            # do delta movements and seek for plant near
                            delta_seeking_target_positions = [
                                [dl_sm_init_x, dl_sm_init_y - config.SEEK_DELTA_DISTANCE],
                                [dl_sm_init_x - config.SEEK_DELTA_DISTANCE, dl_sm_init_y],
                                [dl_sm_init_x, dl_sm_init_y + config.SEEK_DELTA_DISTANCE],
                                [dl_sm_init_x + config.SEEK_DELTA_DISTANCE, dl_sm_init_y]
                            ]
                            for delta_sm_x, delta_sm_y in delta_seeking_target_positions:
                                if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
                                    msg = f"EXT. PAUSE: Starting delta X={str(delta_sm_x)} Y={str(delta_sm_y)} " \
                                          f"scan (will be skipped if out of working range); press enter:"
                                    self.__logger_full.write(msg + "\n")
                                    input(msg)
                                # demo pause
                                if config.ALLOW_DEMO_PAUSES and self.__demo_server is not None:
                                    self.__demo_server.wait_for_resume_cmd()

                                # check if coordinates are in working range
                                if not (config.X_MIN < delta_sm_x < config.X_MAX and
                                        config.Y_MIN < delta_sm_y < config.Y_MAX):
                                    continue

                                delta_shifts_count += 1

                                # do delta movement
                                ext_xy_start_t = time.time()
                                res = self.__smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                                                 Y_F=config.Y_F_MAX,
                                                                                 X=delta_sm_x,
                                                                                 Y=delta_sm_y)
                                self.__smoothie.wait_for_all_actions_done()
                                self.__data_collector.add_all_ext_xy_t(time.time() - ext_xy_start_t)
                                if res != self.__smoothie.RESPONSE_OK:
                                    msg = "Couldn't do delta move to X" + str(delta_sm_x) + " Y" + \
                                          str(delta_sm_y) + ", smoothie response:\n" + res
                                    self.__logger_full.write(msg + "\n")
                                    continue

                                # make a scan, keep only plants that are in undistorted zone
                                # TODO data collection image analysis time in case if delta's should be included into it
                                time.sleep(config.DELAY_BEFORE_2ND_SCAN)
                                frame = self.__camera.get_image()
                                cur_pos_plant_boxes_undist = list(filter(
                                    lambda plant_box_1: self.is_point_in_circle(
                                        plant_box_1.center_x,
                                        plant_box_1.center_y,
                                        config.SCENE_CENTER_X,
                                        config.SCENE_CENTER_Y,
                                        config.UNDISTORTED_ZONE_RADIUS),
                                    self.__precise_det.detect(frame)
                                ))

                                # stop seeking and save current position as new current position. This position is also
                                # used during check rescan after extractions
                                if len(cur_pos_plant_boxes_undist) > 0:
                                    cur_pos_sm_x, cur_pos_sm_y = delta_sm_x, delta_sm_y

                                    if config.SAVE_DEBUG_IMAGES:
                                        frame = utility.ImageSaver.draw_data_in_frame(
                                            frame,
                                            undistorted_zone_radius=config.UNDISTORTED_ZONE_RADIUS,
                                            plants_boxes=cur_pos_plant_boxes_undist)
                                        self.__image_saver.save_image(
                                            frame,
                                            config.DEBUG_IMAGES_PATH,
                                            label=f"(PR_view_at_{round(cur_pos_sm_x, 1)}_{round(cur_pos_sm_y, 1)}),delta_weeds_in_undist",
                                            plants_boxes=cur_pos_plant_boxes_undist)

                                    if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
                                        msg = f"EXT. PAUSE: Found {str(len(cur_pos_plant_boxes_undist))} plants " \
                                              f"in undistorted zone during delta scan, going to extract, changed " \
                                              f"plant {str(plant_index)} start position to X={str(cur_pos_sm_x)}, " \
                                              f"Y={str(cur_pos_sm_y)}; press enter:"
                                        self.__logger_full.write(msg + "\n")
                                        input(msg)
                                    # demo pause
                                    if config.ALLOW_DEMO_PAUSES and self.__demo_server is not None:
                                        self.__demo_server.wait_for_resume_cmd()

                                    break
                                else:
                                    msg = "No plants found during delta scan iteration"
                                    self.__logger_full.write(msg + "\n")
                            # go to next plant in PDZ list if delta seeking loop found nothing
                            else:
                                msg = "No plants found during all delta scans"
                                self.__logger_full.write(msg + "\n")
                                self.__data_collector.add_all_ext_delta_info(
                                    time.time() - delta_start_t,
                                    delta_shifts_count
                                )
                                break

                            self.__data_collector.add_all_ext_delta_info(
                                time.time() - delta_start_t,
                                delta_shifts_count
                            )
                    # go to next plant in PDZ list if no plants detected after extraction attempting
                    else:
                        msg = "No plants detected - assuming last extraction was successful; coming to next PDZ item"
                        self.__logger_full.write(msg + "\n")
                        if config.VERBOSE:
                            print(msg)
                        break

                # try to filter extracted plants by comparison new plants list and initial plants list
                # if plant in a new list is away from all plants in old - then it probably was extracted and shifted
                # so skip it
                if config.FILTER_EXTRACTED_PLANTS:
                    if scan_is_first:
                        initial_plants = cur_pos_plant_boxes_undist
                    else:
                        cur_pos_plant_boxes_undist = self.__filter_extracted_plants(
                            initial_plants,
                            cur_pos_plant_boxes_undist,
                            config.FILTER_EXT_PLANTS_TRIGGER_DIST,
                            self.__logger_full)

                        if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE and \
                                len(initial_plants) != len(cur_pos_plant_boxes_undist):
                            msg = f"EXT. PAUSE: Plants list was filtered due to config.FILTER_EXTRACTED_PLANTS " \
                                  f"setting, check log file for details; press enter:"
                            self.__logger_full.write_and_flush(msg + "\n")
                            input(msg)

                scan_is_first = False

                # convert plant boxes into smoothie absolute coordinates pairs and her type
                smoothie_plants_positions = []
                for plant_box in cur_pos_plant_boxes_undist:
                    plant_box: detection.DetectedPlantBox = plant_box
                    rel_sm_x = self.px_to_smoothie_value(plant_box.center_x, config.SCENE_CENTER_X, config.ONE_MM_IN_PX)
                    rel_sm_y = -self.px_to_smoothie_value(plant_box.center_y, config.SCENE_CENTER_Y,
                                                          config.ONE_MM_IN_PX)

                    # swap camera and cork for extraction immediately (coords are relative)
                    rel_sm_x += config.CORK_TO_CAMERA_DISTANCE_X
                    rel_sm_y += config.CORK_TO_CAMERA_DISTANCE_Y

                    # convert smoothie relative coordinates to absolute
                    cur_sm_pos = self.__smoothie.get_adapter_current_coordinates()
                    abs_sm_x, abs_sm_y = cur_sm_pos["X"] + rel_sm_x, cur_sm_pos["Y"] + rel_sm_y

                    # skip coordinates that are out of working range
                    if not (config.X_MIN < abs_sm_x < config.X_MAX or config.Y_MIN < abs_sm_y < config.Y_MAX):
                        continue

                    # add absolute coordinates to the result list
                    smoothie_plants_positions.append((abs_sm_x, abs_sm_y, plant_box.get_name()))

                # extract these plants
                for ext_sm_x, ext_sm_y, type_name in smoothie_plants_positions:
                    extraction_pattern = self.__extraction_map.get_strategy(ext_sm_x, ext_sm_y)

                    if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
                        msg = f"EXT. PAUSE: Going to plant AbsX={str(ext_sm_x)}, AbsY={str(ext_sm_y)}; press enter:"
                        self.__logger_full.write(msg + "\n")
                        input(msg)
                    # demo pause
                    if config.ALLOW_DEMO_PAUSES and self.__demo_server is not None:
                        self.__demo_server.wait_for_resume_cmd()

                    if extraction_pattern:
                        # go to position
                        ext_xy_start_t = time.time()
                        res = self.__smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                                         Y_F=config.Y_F_MAX,
                                                                         X=ext_sm_x,
                                                                         Y=ext_sm_y)
                        self.__smoothie.wait_for_all_actions_done()
                        self.__data_collector.add_all_ext_xy_t(time.time() - ext_xy_start_t)
                        if res != self.__smoothie.RESPONSE_OK:
                            msg = f"Passing this plant by as could not move cork to position " \
                                  f"X={ext_sm_x} Y={ext_sm_y}, smoothie res:"
                            msg += "\n" + res
                            self.__logger_full.write(msg + "\n")
                            continue

                        # extract
                        res, cork_is_stuck = extraction_pattern(self.__smoothie,
                                                                self.__vesc_engine,
                                                                self.__extraction_map,
                                                                self.__data_collector,
                                                                self.__demo_server)
                        if res != self.__smoothie.RESPONSE_OK:
                            msg = "Something gone wrong during extractions, smoothie's response:\n" + res
                            self.__logger_full.write(msg + "\n")
                            if cork_is_stuck:  # danger flag is True if smoothie couldn't pick up the corkscrew
                                msg = "Corkscrew is stuck! Emergency stopping."
                                self.__logger_full.write(msg + "\n")
                                exit(1)
                        else:
                            if extraction_pattern == self.__extraction_map.strategies[0]:
                                self.__data_collector.add_extractions_data(type_name, 1)
                                self.__data_collector.save_all_data(self.__log_cur_dir + config.STATISTICS_OUTPUT_FILE)
                    else:
                        msg = "Did too many extraction tries at this position, no strategies to try left"
                        self.__logger_full.write(msg + "\n")

                        if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
                            msg = f"EXT. PAUSE: Skipped plant as extraction strategy was None (already tried all " \
                                  f"strategies); press enter:"
                            self.__logger_full.write(msg + "\n")
                            input(msg)

        # set camera back to the Y min X max / 2
        ext_xy_start_t = time.time()
        res = self.__smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                         Y_F=config.Y_F_MAX,
                                                         X=self.__smoothie.smoothie_to_mm(
                                                             (config.X_MAX - config.X_MIN) / 2, "X"),
                                                         Y=self.__smoothie.smoothie_to_mm(config.Y_MIN, "Y"))
        self.__smoothie.wait_for_all_actions_done()
        self.__data_collector.add_all_ext_xy_t(time.time() - ext_xy_start_t)
        if res != self.__smoothie.RESPONSE_OK:
            msg = "Couldn't set camera back to cruise scan position after extractions, smoothie's response:\n" + res
            self.__logger_full.write(msg + "\n")

        self.__data_collector.add_all_ext_t(time.time() - extract_all_plants_start_t)

    def mill_all_plants(self):
        """Find and mill all plants found in current robot's position
        """

        mill_all_plants_start_t = time.time()

        # self.__extraction_map.clear()

        # do sectored scans
        smoothie_positions = self.scan_sectors()
        msg = "Found " + str(len(smoothie_positions)) + " plants after PDZ scan"
        self.__logger_full.write(msg + "\n")
        if config.VERBOSE:
            print(msg)
        # round coords before logging them
        if len(smoothie_positions) != 0:
            log_sm_positions = list(map(lambda item: (round(item[0], 2), round(item[1], 2)), smoothie_positions))
            msg = "PDZ plants smoothie coordinates:\n" + str(log_sm_positions)
            self.__logger_full.write(msg + "\n")
            if config.VERBOSE:
                print(msg)

        # loop over plants that were detected during PDZ sectored scans and extract them (main ext loop)
        for init_pos_sm_x, init_pos_sm_y in smoothie_positions:
            cur_pos_sm_x, cur_pos_sm_y = init_pos_sm_x, init_pos_sm_y

            # modify coordinates to try to avoid corkscrew tube view obscuring
            if config.AVOID_CORK_VIEW_OBSCURING:
                # determine shift direction depending at which working area quarter target is
                obscuring_offset_x = config.AVOID_CORK_VIEW_OBSCURING_DIST_X if init_pos_sm_x < config.X_MAX / 2 \
                    else -config.AVOID_CORK_VIEW_OBSCURING_DIST_X
                if config.X_MIN < init_pos_sm_x + obscuring_offset_x < config.X_MAX:
                    cur_pos_sm_x += obscuring_offset_x
                if config.Y_MIN < init_pos_sm_y + config.AVOID_CORK_VIEW_OBSCURING_DIST_Y < config.Y_MAX:
                    cur_pos_sm_y += config.AVOID_CORK_VIEW_OBSCURING_DIST_Y

            # affects robot's behaviour if no plants were detected; responsible for delta scans and extractions checking
            scan_is_first = True

            # this loop determines plant (or plants group if they were not detected during PDZ) extractions tries count
            for _ in range(config.EXTRACTION_TRIES_PER_PLANT):
                movement_messages = [
                    "Failed to move cork to the plant rescan position, smoothie's response:\n",
                    "Failed to move cork to the plant rescan position Y_MIN (after calibration), smoothie's response:\n"
                ]
                # try to move to the plant rescan position, calibrate and try again if failed, exit app otherwise
                for idx, movement_message in enumerate(movement_messages):
                    res = self.__smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                                     Y_F=config.Y_F_MAX,
                                                                     X=cur_pos_sm_x,
                                                                     Y=cur_pos_sm_y)
                    self.__smoothie.wait_for_all_actions_done()
                    if res == self.__smoothie.RESPONSE_OK:
                        break
                    else:
                        msg = movement_message + res
                        self.__logger_full.write(msg + "\n")

                        # try calibration (only once, at first movement fail)
                        if idx != 0:
                            break
                        msg = "Trying to calibrate cork"
                        self.__logger_full.write(msg + "\n")
                        res = self.__smoothie.ext_calibrate_cork()
                        if res != self.__smoothie.RESPONSE_OK:
                            msg = "Failed to calibrate cork, smoothie's response:\n" + res
                            self.__logger_full.write(msg + "\n")
                            exit(1)

                # make a scan, keep only plants that are in undistorted zone
                # TODO: possibly here will be multiple scans with average coordinates
                time.sleep(config.DELAY_BEFORE_2ND_SCAN)
                frame = self.__camera.get_image()
                plants_boxes = self.__precise_det.detect(frame)
                cur_pos_plant_boxes_undist = list(filter(
                    lambda plant_box_1: self.is_point_in_circle(
                        plant_box_1.center_x,
                        plant_box_1.center_y,
                        config.SCENE_CENTER_X,
                        config.SCENE_CENTER_Y,
                        config.UNDISTORTED_ZONE_RADIUS),
                    plants_boxes
                ))

                if config.SAVE_DEBUG_IMAGES:
                    if len(cur_pos_plant_boxes_undist) > 0:
                        frame = utility.ImageSaver.draw_data_in_frame(
                            frame,
                            undistorted_zone_radius=config.UNDISTORTED_ZONE_RADIUS,
                            plants_boxes=cur_pos_plant_boxes_undist)
                        self.__image_saver.save_image(
                            frame,
                            config.DEBUG_IMAGES_PATH,
                            label=f"(PR_view_at_{round(cur_pos_sm_x, 1)}_{round(cur_pos_sm_y, 1)}),weeds_in_undist",
                            plants_boxes=cur_pos_plant_boxes_undist)
                    else:
                        frame = utility.ImageSaver.draw_data_in_frame(
                            frame,
                            undistorted_zone_radius=config.UNDISTORTED_ZONE_RADIUS,
                            plants_boxes=plants_boxes)
                        self.__image_saver.save_image(
                            frame, config.DEBUG_IMAGES_PATH,
                            label=f"(PR_view_at_{round(cur_pos_sm_x, 1)}_{round(cur_pos_sm_y, 1)}),no_weeds_in_undist",
                            plants_boxes=plants_boxes)

                # do rescan using delta seeking if nothing detected, it was 1rst scan and delta seeking is allowed
                if len(cur_pos_plant_boxes_undist) == 0:
                    if scan_is_first:
                        msg = "No plants detected (plant was in working zone before)," \
                              "trying to do delta movement and find this plant"
                        self.__logger_full.write(msg + "\n")
                        if config.VERBOSE:
                            print(msg)

                        if config.ALLOW_DELTA_SEEKING:
                            delta_start_t = time.time()
                            delta_shifts_count = 0

                            if config.DELTA_SEEKING_IGNORE_OBSCURING:
                                dl_sm_init_x, dl_sm_init_y = init_pos_sm_x, init_pos_sm_y
                            else:
                                dl_sm_init_x, dl_sm_init_y = cur_pos_sm_x, cur_pos_sm_y

                            # do delta movements and seek for plant near
                            delta_seeking_target_positions = [
                                [dl_sm_init_x, dl_sm_init_y - config.SEEK_DELTA_DISTANCE],
                                [dl_sm_init_x - config.SEEK_DELTA_DISTANCE, dl_sm_init_y],
                                [dl_sm_init_x, dl_sm_init_y + config.SEEK_DELTA_DISTANCE],
                                [dl_sm_init_x + config.SEEK_DELTA_DISTANCE, dl_sm_init_y]
                            ]
                            for delta_sm_x, delta_sm_y in delta_seeking_target_positions:
                                delta_shifts_count += 1

                                # check if coordinates are in working range
                                if not (config.X_MIN < delta_sm_x < config.X_MAX and
                                        config.Y_MIN < delta_sm_y < config.Y_MAX):
                                    continue

                                # do delta movement
                                res = self.__smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                                                 Y_F=config.Y_F_MAX,
                                                                                 X=delta_sm_x,
                                                                                 Y=delta_sm_y)
                                self.__smoothie.wait_for_all_actions_done()
                                if res != self.__smoothie.RESPONSE_OK:
                                    msg = "Couldn't do delta move to X" + str(delta_sm_x) + " Y" + \
                                          str(delta_sm_y) + ", smoothie response:\n" + res
                                    self.__logger_full.write(msg + "\n")
                                    continue

                                # make a scan, keep only plants that are in undistorted zone
                                time.sleep(config.DELAY_BEFORE_2ND_SCAN)
                                frame = self.__camera.get_image()
                                cur_pos_plant_boxes_undist = list(filter(
                                    lambda plant_box_1: self.is_point_in_circle(
                                        plant_box_1.center_x,
                                        plant_box_1.center_y,
                                        config.SCENE_CENTER_X,
                                        config.SCENE_CENTER_Y,
                                        config.UNDISTORTED_ZONE_RADIUS),
                                    self.__precise_det.detect(frame)
                                ))

                                # stop seeking and save current position as new current position. This position is also
                                # used during check rescan after extractions
                                if len(cur_pos_plant_boxes_undist) > 0:
                                    cur_pos_sm_x, cur_pos_sm_y = delta_sm_x, delta_sm_y

                                    if config.SAVE_DEBUG_IMAGES:
                                        frame = utility.ImageSaver.draw_data_in_frame(
                                            frame,
                                            undistorted_zone_radius=config.UNDISTORTED_ZONE_RADIUS,
                                            plants_boxes=cur_pos_plant_boxes_undist)
                                        self.__image_saver.save_image(
                                            frame, config.DEBUG_IMAGES_PATH,
                                            label=f"(PR_view_at_{round(cur_pos_sm_x, 1)}_{round(cur_pos_sm_y, 1)}),"
                                                  f"delta_weeds_in_undist",
                                            plants_boxes=cur_pos_plant_boxes_undist)
                                    break
                                else:
                                    msg = "No plants found during delta scan iteration"
                                    self.__logger_full.write(msg + "\n")
                            # go to next plant in PDZ list if delta seeking loop found nothing
                            else:
                                msg = "No plants found during all delta scans"
                                self.__logger_full.write(msg + "\n")
                                self.__data_collector.add_all_ext_delta_info(
                                    time.time() - delta_start_t,
                                    delta_shifts_count
                                )
                                break

                            self.__data_collector.add_all_ext_delta_info(
                                time.time() - delta_start_t,
                                delta_shifts_count
                            )
                    # go to next plant in PDZ list if no plants detected after extraction attempting
                    else:
                        msg = "No plants detected - assuming last extraction was successful; coming to next PDZ item"
                        self.__logger_full.write(msg + "\n")
                        if config.VERBOSE:
                            print(msg)
                        break

                # try to filter extracted plants by comparison new plants list and initial plants list
                # if plant in a new list is away from all plants in old - then it probably was extracted and shifted
                # so skip it
                if config.FILTER_EXTRACTED_PLANTS and scan_is_first:
                    initial_plants = cur_pos_plant_boxes_undist
                if config.FILTER_EXTRACTED_PLANTS and not scan_is_first:
                    cur_pos_plant_boxes_undist = self.__filter_extracted_plants(initial_plants,
                                                                                cur_pos_plant_boxes_undist,
                                                                                config.FILTER_EXT_PLANTS_TRIGGER_DIST,
                                                                                self.__logger_full)

                scan_is_first = False

                # convert plant boxes into smoothie absolute coordinates pairs and her type
                smoothie_plants_positions = []
                for plant_box in cur_pos_plant_boxes_undist:
                    plant_box: detection.DetectedPlantBox = plant_box
                    rel_sm_x = self.px_to_smoothie_value(plant_box.center_x, config.SCENE_CENTER_X, config.ONE_MM_IN_PX)
                    rel_sm_y = -self.px_to_smoothie_value(plant_box.center_y, config.SCENE_CENTER_Y,
                                                          config.ONE_MM_IN_PX)

                    # swap camera and cork for extraction immediately (coords are relative)
                    rel_sm_x += config.CORK_TO_CAMERA_DISTANCE_X
                    rel_sm_y += config.CORK_TO_CAMERA_DISTANCE_Y

                    # convert smoothie relative coordinates to absolute
                    cur_sm_pos = self.__smoothie.get_adapter_current_coordinates()
                    abs_sm_x, abs_sm_y = cur_sm_pos["X"] + rel_sm_x, cur_sm_pos["Y"] + rel_sm_y

                    # skip box if it's center is out of working zone
                    if not (config.X_MIN < abs_sm_x < config.X_MAX or config.Y_MIN < abs_sm_y < config.Y_MAX):
                        continue

                    # compute smoothie absolute coords of the box edges
                    box_x_size, box_y_size = plant_box.get_sizes()
                    box_x_size *= config.MILLING_PLANT_BOX_X_SIZE_SCALE
                    box_y_size *= config.MILLING_PLANT_BOX_Y_SIZE_SCALE
                    abs_sm_left_edge_x = abs_sm_x + self.px_to_smoothie_value(plant_box.center_x - box_x_size / 2,
                                                                              plant_box.center_x,
                                                                              config.ONE_MM_IN_PX)
                    abs_sm_right_edge_x = abs_sm_x + self.px_to_smoothie_value(plant_box.center_x + box_x_size / 2,
                                                                               plant_box.center_x,
                                                                               config.ONE_MM_IN_PX)
                    abs_sm_top_edge_y = abs_sm_y + self.px_to_smoothie_value(plant_box.center_y + box_y_size / 2,
                                                                             plant_box.center_y,
                                                                             config.ONE_MM_IN_PX)
                    abs_sm_bot_edge_y = abs_sm_y + self.px_to_smoothie_value(plant_box.center_y - box_y_size / 2,
                                                                             plant_box.center_y,
                                                                             config.ONE_MM_IN_PX)

                    # reduce smoothie box sizes if it gets outside of robot's working zone
                    # reduce X size
                    if abs_sm_left_edge_x < config.X_MIN:
                        abs_sm_left_edge_x = config.X_MIN + 1  # to avoid float trait inside adapter (-0.00000001 < 0)
                    elif abs_sm_left_edge_x > config.X_MAX:
                        abs_sm_left_edge_x = config.X_MAX - 1  # to avoid float trait inside adapter (40.00000001 > 40)
                    if abs_sm_right_edge_x < config.X_MIN:
                        abs_sm_right_edge_x = config.X_MIN + 1
                    elif abs_sm_right_edge_x > config.X_MAX:
                        abs_sm_right_edge_x = config.X_MAX - 1
                    # reduce Y size
                    if abs_sm_top_edge_y < config.Y_MIN:
                        abs_sm_top_edge_y = config.Y_MIN + 1
                    elif abs_sm_top_edge_y > config.Y_MAX:
                        abs_sm_top_edge_y = config.Y_MAX - 1
                    if abs_sm_bot_edge_y < config.Y_MIN:
                        abs_sm_bot_edge_y = config.Y_MIN + 1
                    elif abs_sm_bot_edge_y > config.Y_MAX:
                        abs_sm_bot_edge_y = config.Y_MAX - 1

                    # add absolute coordinates to the result list
                    smoothie_plants_positions.append((abs_sm_left_edge_x,
                                                      abs_sm_right_edge_x,
                                                      abs_sm_bot_edge_y,
                                                      abs_sm_top_edge_y,
                                                      plant_box.get_name()))

                # mill these plants; loop over plants
                for sm_left_x, sm_right_x, sm_bot_y, sm_top_y, plant_name in smoothie_plants_positions:
                    # go to starting position
                    res = self.__smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                                     Y_F=config.Y_F_MAX,
                                                                     X=sm_left_x,
                                                                     Y=sm_bot_y)
                    self.__smoothie.wait_for_all_actions_done()
                    if res != self.__smoothie.RESPONSE_OK:
                        msg = f"Skipping this plant by as could not move cork to initial position" \
                              f"X={sm_left_x} Y={sm_bot_y}, smoothie res:"
                        msg += "\n" + res
                        self.__logger_full.write(msg + "\n")
                        continue

                    # compute Y axis milling shifts coords
                    sm_milling_y_points = [i for i in range(int(sm_bot_y), int(sm_top_y), config.MILLING_Y_STEP)]
                    if len(sm_milling_y_points) == 0:
                        msg = "Skipping plant because of too narrow plant box after reduction to working zone borders"
                        self.__logger_full.write(msg + "\n")
                        continue
                    if sm_top_y - sm_milling_y_points[-1] > config.MILLING_LAST_LINE_THRESHOLD:
                        sm_milling_y_points.append(sm_top_y)

                    # put miller down and keep enabled
                    self.__vesc_engine.set_target_rpm(config.MILLING_CORK_DOWN_RPM, self.__vesc_engine.EXTRACTION_KEY)
                    self.__vesc_engine.set_time_to_move(float("inf"), self.__vesc_engine.EXTRACTION_KEY)
                    self.__vesc_engine.start_moving(self.__vesc_engine.EXTRACTION_KEY)
                    time.sleep(config.MILLING_CORK_GROUND_REACHING_DELAY)

                    # milling movement loop ("fills" plant's figure with miller device enabled)
                    x_go_to_right = True
                    for abs_sm_y in sm_milling_y_points:
                        # Y shift
                        res = self.__smoothie.custom_move_to(Y_F=config.MILLING_Y_F, Y=abs_sm_y)
                        if res != self.__smoothie.RESPONSE_OK:
                            msg = f"Could not move to Y{str(abs_sm_y)} during milling, smoothie response:\n{res}\n"
                            self.__logger_full.write(msg)
                        self.__smoothie.wait_for_all_actions_done()

                        # X line
                        if x_go_to_right:
                            x_go_to_right = False
                            res = self.__smoothie.custom_move_to(X_F=config.MILLING_X_F, X=sm_right_x)
                            if res != self.__smoothie.RESPONSE_OK:
                                msg = f"Could not move to X{str(sm_right_x)} during milling, smoothie response:\n{res}\n"
                                self.__logger_full.write(msg)
                        else:
                            x_go_to_right = True
                            res = self.__smoothie.custom_move_to(X_F=config.MILLING_X_F, X=sm_left_x)
                            if res != self.__smoothie.RESPONSE_OK:
                                msg = f"Could not move to X{str(sm_left_x)} during milling, smoothie response:\n{res}\n"
                                self.__logger_full.write(msg)
                        self.__smoothie.wait_for_all_actions_done()

                    # stop and pick up the cork after current plant milling is done
                    self.__vesc_engine.stop_moving(self.__vesc_engine.EXTRACTION_KEY)
                    self.__vesc_engine.set_target_rpm(config.MILLING_CORK_UP_RPM, self.__vesc_engine.EXTRACTION_KEY)
                    self.__vesc_engine.set_time_to_move(
                        config.MILLING_CORK_STOPPER_REACHING_MAX_TIME,
                        self.__vesc_engine.EXTRACTION_KEY)  # stop by movement timeout if GPIO fails
                    self.__vesc_engine.start_moving(self.__vesc_engine.EXTRACTION_KEY)
                    res = self.__vesc_engine.wait_for_stopper_hit(
                        self.__vesc_engine.EXTRACTION_KEY,
                        config.MILLING_CORK_STOPPER_REACHING_MAX_TIME)
                    self.__vesc_engine.stop_moving(self.__vesc_engine.EXTRACTION_KEY)
                    if not res:
                        msg = "WARNING: Miller vesc engine was stopped by timeout, not by stopper!"
                        print(msg)
                        self.__logger_full.write(msg + "\n")

        # set camera back to the Y min X max / 2
        res = self.__smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                         Y_F=config.Y_F_MAX,
                                                         X=self.__smoothie.smoothie_to_mm(
                                                             (config.X_MAX - config.X_MIN) / 2, "X"),
                                                         Y=self.__smoothie.smoothie_to_mm(config.Y_MIN, "Y"))
        self.__smoothie.wait_for_all_actions_done()
        if res != self.__smoothie.RESPONSE_OK:
            msg = "Couldn't set camera back to cruise scan position after extractions, smoothie's response:\n" + res
            self.__logger_full.write(msg + "\n")

        self.__data_collector.add_all_ext_t(time.time() - mill_all_plants_start_t)

    def optimize_corkscrew_way(self, smoothie_coordinates: list):
        """Optimizes order of smoothie points to visit to reduce cork total path length during this list extractions"""

        # TODO: implement TSP optimization algorithm instead
        # reverse list to start from last (closest) positions checked
        return sorted(smoothie_coordinates, key=lambda x: (x[0], x[1]), reverse=True)
        # return list(reversed(smoothie_coordinates))

    @staticmethod
    def __filter_extracted_plants(initial_scan_plants: list,
                                  new_scan_plants: list,
                                  trigger_distance: float,
                                  logger_full: utility.Logger):
        """Removes each plant from new scan list if all plants in initial list are further than trigger distance.
        Does no changes to argument lists, returns result as a new list.
        """

        filtered_plants = []
        for new_plant_box in new_scan_plants:
            new_plant_box: detection.DetectedPlantBox
            for init_plant_box in initial_scan_plants:
                init_plant_box: detection.DetectedPlantBox
                if new_plant_box.get_distance_from(init_plant_box.center_x,
                                                   init_plant_box.center_y) <= trigger_distance:
                    filtered_plants.append(new_plant_box)
                    break
            else:
                msg = f"Removed {str(new_plant_box)} box from undist plants list as this box is far away from his " \
                      f"initial position (likely was extracted)"
                logger_full.write(msg + "\n")
                if config.VERBOSE_EXTRACT:
                    print(msg)
        return filtered_plants

    @staticmethod
    def any_plant_in_zone(plant_boxes: list, zone_polygon: Polygon):
        """
        Returns True if at least one plant box center is in given polygon, False otherwise.
        :param plant_boxes:
        :param zone_polygon:
        :return:
        """

        for box in plant_boxes:
            box_x, box_y = box.get_center_points()
            if ExtractionManagerV3.is_point_in_poly(box_x, box_y, zone_polygon):
                return True
        return False

    @staticmethod
    def get_plants_in_circle(plants_boxes, circle_center_x, circle_center_y, circle_radius):
        """Returns None if not have plant in circle, list of plants otherwise"""

        plants = list()

        for plant in plants_boxes:
            box_x, box_y = plant.get_center_points()
            if ExtractionManagerV3.is_point_in_circle(box_x, box_y, circle_center_x, circle_center_y, circle_radius):
                plants.append(plant)

        if not plants:
            return None

        return plants

    @staticmethod
    def is_point_in_circle(point_x, point_y, circle_center_x, circle_center_y, circle_radius):
        """Returns True if (x,y) point in the circle or on it's border, False otherwise"""

        return math.sqrt((point_x - circle_center_x) ** 2 + (point_y - circle_center_y) ** 2) <= circle_radius

    @staticmethod
    def is_point_in_poly(point_x, point_y, polygon: Polygon):
        """Returns True if received polygon object contains received point, False otherwise"""

        return polygon.contains_point((point_x, point_y))

    @staticmethod
    def px_to_smoothie_value(target_px, center_px, one_mm_in_px):
        """Converts the distance given in pixels to the distance in millimeters"""

        # returns wrong sign for x because of different 0 position between smoothie and image
        return (target_px - center_px) / one_mm_in_px

    @staticmethod
    def min_plant_box_dist(boxes: list, current_px_x, current_px_y):
        """Returns plant box, which is closest to the given point coordinates"""

        return min(boxes, key=lambda box: box.get_distance_from(current_px_x, current_px_y))


class ExtractionMap:
    """Contains info about extractions count at position, provides extraction strategy based on this info
    """

    def __init__(self, cell_size_mm):
        self.__cell_size_mm = cell_size_mm
        self.__matrix = dict()
        # list items order is strategies order which defines what strategies should be used first
        self.__strategies = [
            ExtractionMethods.single_center_drop,
            ExtractionMethods.pattern_plus,
            ExtractionMethods.pattern_x
        ]

    def record_extraction(self, x: float, y: float):
        x, y = x // self.__cell_size_mm, y // self.__cell_size_mm

        if x not in self.__matrix:
            self.__matrix[x] = dict()
        if y not in self.__matrix[x]:
            self.__matrix[x][y] = 0

        self.__matrix[x][y] += 1

    def get_strategy(self, x: float, y: float):
        x, y = x // self.__cell_size_mm, y // self.__cell_size_mm

        # values in matrix are indexes in a self.__strategies list. If index is out of list range -
        # then do nothing (return None) as we tried to do all we can (values is being incremented at each extraction)
        # if cell wasn't used - then use first "starting" strategy at [0]
        if x in self.__matrix and y in self.__matrix[x]:
            return self.__strategies[self.__matrix[x][y]] if self.__matrix[x][y] < len(self.__strategies) else None
        return self.__strategies[0]

    def clear(self):
        self.__matrix.clear()

    def save_to_file(self, output_file_path: str):
        raise NotImplementedError("this function is not implemented yet")

    @property
    def strategies(self):
        return self.__strategies


class ExtractionMethods:
    """Contains methods for different plants extraction strategies. All methods should have similar signatures.
    """

    @staticmethod
    def single_center_drop(smoothie: adapters.SmoothieAdapter,
                           vesc_adapter: adapters.VescAdapterV4,
                           extraction_map: ExtractionMap,
                           data_collector: datacollection.DataCollector,
                           demo_server: utility.DemoPauseServer):
        """Extract a plant with a single corkscrew drop to the center"""

        if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
            msg = f"EXP. PAUSE: Ready to put cork down; press enter:"
            print(msg)
        # demo pause
        if config.ALLOW_DEMO_PAUSES and demo_server is not None:
            demo_server.wait_for_resume_cmd()

        res = smoothie.RESPONSE_OK
        all_ext_z_start_t = start_t = time.time()

        # this loop implements multiple tries of cork re-picking
        for _ in range(config.VESC_CORK_PICKUP_MAX_TRIES):
            # extraction, cork down
            if config.EXTRACTION_CONTROLLER == 1:  # if Z axis controller is smoothie
                res = smoothie.custom_move_for(Z_F=config.Z_F_EXTRACTION_DOWN, Z=config.EXTRACTION_Z)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:  # or smoothie.checkendstop("Z")==1
                    data_collector.add_all_ext_z_t(time.time() - all_ext_z_start_t)
                    msg = "Couldn't move the extractor down, smoothie error occurred:\n" + res
                    return msg, False
            elif config.EXTRACTION_CONTROLLER == 2:  # if Z axis controller is vesc
                vesc_adapter.set_target_rpm(config.EXTRACTION_CORK_DOWN_RPM, vesc_adapter.EXTRACTION_KEY)
                vesc_adapter.set_time_to_move(config.EXTRACTION_CORK_DOWN_TIME, vesc_adapter.EXTRACTION_KEY)
                vesc_adapter.start_moving(vesc_adapter.EXTRACTION_KEY)
                vesc_adapter.wait_for_stop(vesc_adapter.EXTRACTION_KEY)
            else:
                msg = f"config.EXTRACTION_CONTROLLER={str(config.EXTRACTION_CONTROLLER)} is not implemented"
                raise NotImplementedError(msg)

            if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
                msg = f"EXT. PAUSE: Cork is down, going to pick it up; press enter:"
                input(msg)
            # demo pause
            if config.ALLOW_DEMO_PAUSES and demo_server is not None:
                demo_server.wait_for_resume_cmd()

            # extraction, cork up
            if config.EXTRACTION_CONTROLLER == 1:  # if Z axis controller is smoothie
                res = smoothie.ext_cork_up()
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    data_collector.add_all_ext_z_t(time.time() - all_ext_z_start_t)
                    msg = "Couldn't move the extractor up, smoothie error occurred:\n" + res + \
                          "\nemergency exit as I don't want break the corkscrew."
                    return msg, True
            elif config.EXTRACTION_CONTROLLER == 2:  # if Z axis controller is vesc
                vesc_adapter.set_target_rpm(config.EXTRACTION_CORK_UP_RPM, vesc_adapter.EXTRACTION_KEY)
                vesc_adapter.set_time_to_move(
                    config.EXTRACTION_CORK_STOPPER_REACHING_MAX_TIME,
                    vesc_adapter.EXTRACTION_KEY)

                cork_pick_start_t = time.time()

                vesc_adapter.start_moving(vesc_adapter.EXTRACTION_KEY)
                cork_is_stuck = not vesc_adapter.wait_for_stopper_hit(
                    vesc_adapter.EXTRACTION_KEY,
                    config.EXTRACTION_CORK_STOPPER_REACHING_MAX_TIME)
                vesc_adapter.stop_moving(vesc_adapter.EXTRACTION_KEY)

                cork_pick_end_t = time.time()

                if config.ALLOW_VESC_CORK_PICKUP_MIN_TIME and \
                        cork_pick_end_t - cork_pick_start_t < config.VESC_CORK_PICKUP_MIN_TIME:
                    continue

                if cork_is_stuck:
                    data_collector.add_all_ext_z_t(time.time() - all_ext_z_start_t)
                    msg = "Not sure that extractor was properly picked up as vesc stopper hit wasn't registered " \
                          "(cork picking up was stopped by security timeout). Emergency exit as I don't want break " \
                          "the corkscrew."
                    return msg, True

                # if execution came here - cork picking time is between min and max time so everything's ok - stop loop
                break
            else:
                msg = f"config.EXTRACTION_CONTROLLER={str(config.EXTRACTION_CONTROLLER)} is not implemented"
                raise NotImplementedError(msg)

            # don't do cork pick re-tries if they are disabled or if Z axis controller is smoothie
            if not config.ALLOW_VESC_CORK_PICKUP_MIN_TIME or config.EXTRACTION_CONTROLLER == 1:
                break
        else:
            data_collector.add_all_ext_z_t(time.time() - all_ext_z_start_t)
            msg = "Not sure that extractor was properly picked up as vesc stopper hit was registered too early " \
                  "(cork pickup time is lesser than VESC_CORK_PICKUP_MIN_TIME). " \
                  "Emergency exit as I don't want break the corkscrew."
            return msg, True

        data_collector.add_all_ext_z_t(time.time() - all_ext_z_start_t)
        data_collector.add_cork_moving_time_data(time.time() - start_t)

        # add an extraction record to the extraction matrix and extraction time to the statistic collector
        sm_cur_coords = smoothie.get_adapter_current_coordinates()
        extraction_map.record_extraction(sm_cur_coords["X"], sm_cur_coords["Y"])

        # seeder
        for _ in range(config.SEEDER_QUANTITY):
            do_x_offset = config.SEEDER_EXT_OFFSET_X != 0
            do_y_offset = config.SEEDER_EXT_OFFSET_Y != 0

            error_offset = False

            # do x offset if needed
            if do_x_offset:
                res = smoothie.custom_move_for(X_F=config.SEEDER_EXT_OFFSET_X_F, X=config.SEEDER_EXT_OFFSET_X)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't do seeder X axis offset, smoothie response:\n" + res
                    print(msg)
                    do_x_offset = False
                    error_offset = True

            # do y offset if needed
            if do_y_offset:
                res = smoothie.custom_move_for(Y_F=config.SEEDER_EXT_OFFSET_Y_F, Y=config.SEEDER_EXT_OFFSET_Y)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't do seeder Y axis offset, smoothie response:\n" + res
                    print(msg)
                    do_y_offset = False
                    error_offset = True

            if not error_offset:
                # seeder open
                res = smoothie.seeder_open()
                if res != smoothie.RESPONSE_OK:
                    msg = "Error during open seeder, smoothie's output:\n" + res
                    print(msg)  # TODO probably need to return later both answers to let outer methods log this error
                time.sleep(config.SEEDER_FILL_DELAY)

                # seeder close
                res = smoothie.seeder_close()
                if res != smoothie.RESPONSE_OK:
                    msg = "Error during close seeder, smoothie's output:\n" + res
                    return msg, False
                time.sleep(config.SEEDER_FILL_DELAY)

                # shake seeder
                for delta in [-1,2,-1]:
                    res = smoothie.custom_move_for(X_F=config.X_F_MAX, Y_F=config.Y_F_MAX, Y=delta, X=delta)
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        msg = "Couldn't do seeder shake, smoothie response:\n" + res
                        print(msg)
            else:
                print("Error_offset")

            # get back to init position if there was offset on X axis
            if do_x_offset:
                res = smoothie.custom_move_for(X_F=config.SEEDER_EXT_OFFSET_X_F, X=-config.SEEDER_EXT_OFFSET_X)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't do seeder X axis offset (back), smoothie response:\n" + res
                    print(msg)

            # get back to init position if there was offset on Y axis
            if do_y_offset:
                res = smoothie.custom_move_for(Y_F=config.SEEDER_EXT_OFFSET_Y_F, Y=-config.SEEDER_EXT_OFFSET_Y)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't do seeder Y axis offset (back), smoothie response:\n" + res
                    print(msg)

        # save current matrix state to a file
        if config.DEBUG_MATRIX_FILE:
            extraction_map.save_to_file("last_extraction_map.txt")

        return res, False

    @staticmethod
    def five_drops_near_center(smoothie: adapters.SmoothieAdapter, extraction_map: ExtractionMap):
        """Extract a plant with a single corkscrew drop to the center and four drops (distances are defined in config)
        """

        raise NotImplementedError("this function need to be updated as it's using old extraction matrix")

        """
        # mms to move near plant's center
        x = config.ADDITIONAL_EXTRACTIONS_DISTANCE_X
        y = config.ADDITIONAL_EXTRACTIONS_DISTANCE_Y

        # drop cork to the center
        res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, extraction_map)
        if res != smoothie.RESPONSE_OK:
            return res, cork_is_stuck

        for x_shift, y_shift in [[0, y], [-x, -y], [x, -y], [x, y]]:
            # move to the position
            res = smoothie.custom_move_for(config.XY_F_MAX, X=x_shift, Y=y_shift)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                msg = "Aborting movement to the corner (couldn't reach): " + res
                return msg, False

            # extraction, cork down
            res = smoothie.custom_move_for(F=config.Z_F_EXTRACTION_DOWN, Z=config.EXTRACTION_Z)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                msg = "Couldn't move the extractor down, smoothie error occurred: " + res
                return msg, False

            # extraction, cork up
            res = smoothie.ext_cork_up()
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                msg = "Couldn't move the extractor up, smoothie error occurred: " + res + \
                      "\nemergency exit as I don't want break corkscrew."
                return msg, True
            else:
                sm_x = smoothie.get_smoothie_current_coordinates()["X"]
                sm_y = smoothie.get_smoothie_current_coordinates()["Y"]
                x = math.floor(sm_x / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
                y = math.floor(sm_y / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
                extraction_map[y, x].setPatternExtraction("five_drops_near_center", parent=extraction_map[sm_y, sm_x])
                if config.DEBUG_MATRIX_FILE:
                    ExtractionManager.save_matrix("last_extraction_map.txt", extraction_map, header=True)

        return res, False
        """

    @staticmethod
    def Daisy(smoothie: adapters.SmoothieAdapter, extraction_map: ExtractionMap):
        """This method is used for a daisy extraction"""

        raise NotImplementedError("this function need to be updated as it's using old extraction matrix")

        """
        # drop cork to the center
        res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, extraction_map)
        if res != smoothie.RESPONSE_OK:
            return res, cork_is_stuck
        else:
            center_sm_x = math.floor(smoothie.get_smoothie_current_coordinates()["X"] /
                                     config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
            center_sm_y = math.floor(smoothie.get_smoothie_current_coordinates()["Y"] /
                                     config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
            extraction_map[center_sm_y, center_sm_x].setPatternExtraction("Daisy")

        box_x_half, box_y_half = plant_box.get_sizes()
        box_x_half, box_y_half = int(box_x_half / 2 / config.ONE_MM_IN_PX), \
                                 int(box_y_half / 2 / config.ONE_MM_IN_PX)

        for x_shift, y_shift in [[-box_x_half, box_y_half], [0, -box_y_half * 2], [box_x_half * 2, 0],
                                 [0, box_y_half * 2]]:
            # move to the corner
            res = smoothie.custom_move_for(config.XY_F_MAX, X=x_shift, Y=y_shift)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                msg = "Aborting movement to the corner (couldn't reach): " + res
                return msg, False

            # extraction, cork down
            res = smoothie.custom_move_for(F=config.Z_F_EXTRACTION_DOWN, Z=config.EXTRACTION_Z)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                msg = "Couldn't move the extractor down, smoothie error occurred: " + res
                return msg, False

            # extraction, cork up
            res = smoothie.ext_cork_up()
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                msg = "Couldn't move the extractor up, smoothie error occurred: " + res + \
                      "\nemergency exit as I don't want break corkscrew."
                return msg, True
            else:
                sm_x = smoothie.get_smoothie_current_coordinates()["X"]
                sm_y = smoothie.get_smoothie_current_coordinates()["Y"]
                x = math.floor(sm_x / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
                y = math.floor(sm_y / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
                extraction_map[y, x].setPatternExtraction("Daisy", parent=extraction_map[center_sm_y, center_sm_x])
                if config.DEBUG_MATRIX_FILE:
                    ExtractionManager.save_matrix("last_extraction_map.txt", extraction_map, header=True)

        return res, False
        """

    @staticmethod
    def Plantain(smoothie: adapters.SmoothieAdapter, extraction_map: ExtractionMap):
        """This method is used for a plantain extraction"""

        raise NotImplementedError("this function is not implemented yet")

    @staticmethod
    def Dandelion(smoothie: adapters.SmoothieAdapter, extraction_map: ExtractionMap):
        """This method is used for a dandelion extraction"""

        raise NotImplementedError("this function is not implemented yet")

    @staticmethod
    def pattern_plus(smoothie: adapters.SmoothieAdapter,
                     vesc_adapter: adapters.VescAdapterV4,
                     extraction_map: ExtractionMap,
                     data_collector: datacollection.DataCollector,
                     demo_server: utility.DemoPauseServer):
        if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
            print("EXT. PAUSE: Starting pattern plus.")

        sm_cur = smoothie.get_adapter_current_coordinates()
        positions = [
            [sm_cur["X"] - config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"]],
            [sm_cur["X"], sm_cur["Y"] + config.EXTRACTION_PATTERNS_OFFSET_MM],
            [sm_cur["X"] + config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"]],
            [sm_cur["X"], sm_cur["Y"] - config.EXTRACTION_PATTERNS_OFFSET_MM]
        ]

        for sm_x, sm_y in positions:
            # skip this move if it is out of working range
            if sm_x <= config.X_MIN or sm_x >= config.X_MAX or sm_y <= config.Y_MIN or sm_y >= config.Y_MAX:
                continue

            # move to position
            all_ext_xy_start_t = time.time()
            res = smoothie.custom_move_to(X_F=config.X_F_MAX, Y_F=config.Y_F_MAX, X=sm_x, Y=sm_y)
            smoothie.wait_for_all_actions_done()
            data_collector.add_all_ext_xy_t(time.time() - all_ext_xy_start_t)
            if res != smoothie.RESPONSE_OK:
                msg = "Couldn't move corkscrew to the one of plant sides, smoothie error occurred:\n" + res
                return msg, False

            # do extraction
            res, cork_is_stuck = ExtractionMethods.single_center_drop(
                smoothie,
                vesc_adapter,
                extraction_map,
                data_collector,
                demo_server)
            if res != smoothie.RESPONSE_OK:
                return res, cork_is_stuck

        return smoothie.RESPONSE_OK, False

    @staticmethod
    def pattern_x(smoothie: adapters.SmoothieAdapter,
                  vesc_adapter: adapters.VescAdapterV4,
                  extraction_map: ExtractionMap,
                  data_collector: datacollection.DataCollector,
                  demo_server: utility.DemoPauseServer):
        if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
            print("EXT. PAUSE: Starting pattern X.")

        sm_cur = smoothie.get_adapter_current_coordinates()
        positions = [
            [sm_cur["X"] - config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"] - config.EXTRACTION_PATTERNS_OFFSET_MM],
            [sm_cur["X"] - config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"] + config.EXTRACTION_PATTERNS_OFFSET_MM],
            [sm_cur["X"] + config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"] + config.EXTRACTION_PATTERNS_OFFSET_MM],
            [sm_cur["X"] + config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"] - config.EXTRACTION_PATTERNS_OFFSET_MM]
        ]

        for sm_x, sm_y in positions:
            # skip this move if it is out of working range
            if sm_x <= config.X_MIN or sm_x >= config.X_MAX or sm_y <= config.Y_MIN or sm_y >= config.Y_MAX:
                continue

            # move to position
            all_ext_xy_start_t = time.time()
            res = smoothie.custom_move_to(X_F=config.X_F_MAX, Y_F=config.Y_F_MAX, X=sm_x, Y=sm_y)
            smoothie.wait_for_all_actions_done()
            data_collector.add_all_ext_xy_t(time.time() - all_ext_xy_start_t)
            if res != smoothie.RESPONSE_OK:
                msg = "Couldn't move corkscrew to the one of plant sides, smoothie error occurred:\n" + res
                return msg, False

            # do extraction
            res, cork_is_stuck = ExtractionMethods.single_center_drop(
                smoothie,
                vesc_adapter,
                extraction_map,
                data_collector,
                demo_server)
            if res != smoothie.RESPONSE_OK:
                return res, cork_is_stuck

        return smoothie.RESPONSE_OK, False

    @staticmethod
    def pattern_3x3(smoothie: adapters.SmoothieAdapter,
                    vesc_adapter: adapters.VescAdapterV4,
                    extraction_map: ExtractionMap,
                    data_collector: datacollection.DataCollector,
                    demo_server: utility.DemoPauseServer):
        """
        Sequence of extraction positions:
        3--4--5
        2--1--6
        9--8--7
        """

        if config.SET_EXTRACTIONS_ON_DEBUG_PAUSE:
            print("EXT. PAUSE: Starting pattern 3x3.")

        sm_cur = smoothie.get_adapter_current_coordinates()
        positions = [
            [sm_cur["X"], sm_cur["Y"]],  # 1
            [sm_cur["X"] - config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"]],  # 2
            [sm_cur["X"] - config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"] + config.EXTRACTION_PATTERNS_OFFSET_MM],  # 3
            [sm_cur["X"], sm_cur["Y"] + config.EXTRACTION_PATTERNS_OFFSET_MM],  # 4
            [sm_cur["X"] + config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"] + config.EXTRACTION_PATTERNS_OFFSET_MM],  # 5
            [sm_cur["X"] + config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"]],  # 6
            [sm_cur["X"] + config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"] - config.EXTRACTION_PATTERNS_OFFSET_MM],  # 7
            [sm_cur["X"], sm_cur["Y"] - config.EXTRACTION_PATTERNS_OFFSET_MM],  # 8
            [sm_cur["X"] - config.EXTRACTION_PATTERNS_OFFSET_MM, sm_cur["Y"] - config.EXTRACTION_PATTERNS_OFFSET_MM],  # 9
        ]

        for sm_x, sm_y in positions:
            # skip this move if it is out of working range
            if sm_x <= config.X_MIN or sm_x >= config.X_MAX or sm_y <= config.Y_MIN or sm_y >= config.Y_MAX:
                continue

            # move to position
            all_ext_xy_start_t = time.time()
            res = smoothie.custom_move_to(X_F=config.X_F_MAX, Y_F=config.Y_F_MAX, X=sm_x, Y=sm_y)
            smoothie.wait_for_all_actions_done()
            data_collector.add_all_ext_xy_t(time.time() - all_ext_xy_start_t)
            if res != smoothie.RESPONSE_OK:
                msg = "Couldn't move corkscrew to the one of plant sides, smoothie error occurred:\n" + res
                return msg, False

            # do extraction
            res, cork_is_stuck = ExtractionMethods.single_center_drop(
                smoothie,
                vesc_adapter,
                extraction_map,
                data_collector,
                demo_server)
            if res != smoothie.RESPONSE_OK:
                return res, cork_is_stuck

        return smoothie.RESPONSE_OK, False


class PxToMMConverter:
    # TODO: old unoptimized code. It is loading data from disk during conversion calls - need to change this.

    def __init__(self):
        self.__loaded_prediction_model = dict()

    def convert_px_to_mm(self, plant_px_x, plant_px_y):
        """ returns the distance in mm between the center of the plant and the center of the scene """

        norm_x = plant_px_x - config.SCENE_CENTER_X
        norm_y = plant_px_y - config.SCENE_CENTER_Y
        dis_from_center = math.sqrt(norm_x ** 2 + norm_y ** 2)
        return self.__plant_position_prediction(norm_x, norm_y, dis_from_center)[:2]

    # prediction of the distance in mm between the center of the plant and the center of the scene
    def __plant_position_prediction(self, norm_x, norm_y,
                                    dis_from_center):  # train the model and get .sav with prediction.py before

        degree = config.ZONE_THRESHOLD_DEGREE[-1][1]

        for threshold, _degree in config.ZONE_THRESHOLD_DEGREE:
            if dis_from_center <= threshold:
                degree = _degree
                break

        if degree is None:
            raise ValueError("Incorrect data !")

        values = [norm_x, norm_y, dis_from_center]

        # generate the matrix of the right degree
        poly_features = PolynomialFeatures(degree=degree)

        if degree not in self.__loaded_prediction_model.keys():
            filename = 'zone_models/zone_model_' + str(degree) + '.sav'
            self.__loaded_prediction_model[degree] = pickle.load(
                open(filename, 'rb'))  # open the model trained

        data = poly_features.fit_transform([values])  # charge the data into the matrix
        return self.__loaded_prediction_model[degree].predict(data)[0]  # model.predict() comes from scikit


class PxToMmPredictorCP:
    """Predicts distance in millimeters from scene center px to given point px

    Uses raw control points for predictions"""

    def __init__(self, scene_center_x: int, scene_center_y: int, cp_csv_path: str = None, control_points: list = None):
        """Uses given control points list or loads it from given csv file path.

        If there was given both control points list and path, given CP list will be used instead of loading from a file.
        """

        # get the list of control points
        if control_points is not None:
            self.__control_points = control_points
        elif cp_csv_path is not None:
            self.__control_points = self.load_control_points(cp_csv_path)
        else:
            msg = "can't load control points list as given cp_csv_path and control_points both are Nones"
            raise RuntimeError(msg)

        # validate control points
        if not isinstance(self.__control_points, list):
            msg = f"given control_points must be a list, got {type(self.__control_points).__name__} instead"
            raise TypeError(msg)
        for idx, item in enumerate(self.__control_points):
            if not isinstance(item, (list, tuple)):
                msg = f"control_points must contain lists or tuples, got {type(item).__name__} instead"
                raise TypeError(msg)
            if len(item) != 7:
                msg = f"control_points[{str(idx)}] item length is {str(len(item))}, must be 7"
                raise ValueError(msg)

        self.__scene_center_x = scene_center_x
        self.__scene_center_y = scene_center_y

    @staticmethod
    def load_control_points(cp_csv_path: str):
        """Loads and parses control points from a given csv file path.

        Returns list of control points, each CP is list of 7 elements with such a format:
        [X_px: int, Y_px: int, X_diff_px: int, Y_diff_px: int, Distance_px: float, X_mm: float, Y_mm: float]
        """

        with open(cp_csv_path) as csv_file:
            # format is: X_px, Y_px, X_diff_px, Y_diff_px, Distance_px, X_mm, Y_mm
            # px data are ints, distance and mm data are floats
            control_points = csv_file.readlines()[1:]  # cut header off

        # convert each point to a list of numeric
        for i in range(len(control_points)):
            control_points[i] = control_points[i].strip().split(",")
            for j in range(4):
                control_points[i][j] = int(control_points[i][j].strip())
            for j in range(4, 7):
                control_points[i][j] = float(control_points[i][j].strip())

        return control_points

    def predict(self, point_x_px: int, point_y_px: int):
        """Predict (x, y) mm distance from stored scene center for given px (x, y) point.

        Return format is: (x_mm, y_mm, control_point_index)
        """

        # remember sign to apply it to mm values and mirror points coordinates into +x +y quarter
        if point_x_px < self.__scene_center_x:
            point_x_px = self.__scene_center_x + (self.__scene_center_x - point_x_px)
            res_x_mm_sign = -1
        else:
            res_x_mm_sign = 1

        if point_y_px > self.__scene_center_y:
            point_y_px = self.__scene_center_y - (point_y_px - self.__scene_center_y)
            res_y_mm_sign = -1
        else:
            res_y_mm_sign = 1

        res_point_dist = float("inf")
        res_point_idx = 0
        for i in range(len(self.__control_points)):
            cur_dist = math.sqrt(
                (point_x_px - self.__control_points[i][0]) ** 2 + (point_y_px - self.__control_points[i][1]) ** 2
            )
            if cur_dist < res_point_dist:
                res_point_dist = cur_dist
                res_point_idx = i

        # X_mm, Y_mm
        return self.__control_points[res_point_idx][5] * res_x_mm_sign, \
            self.__control_points[res_point_idx][6] * res_y_mm_sign, \
            res_point_idx
