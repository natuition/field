from config import config
import adapters
import detection
import math
import numpy as np
import utility
import datacollection
from matplotlib.patches import Polygon
import time
import pickle
import sklearn


class ExtractionManagerV3:
    """Implements extraction logic and control"""

    def __init__(self,
                 smoothie: adapters.SmoothieAdapter,
                 camera: adapters.CameraAdapterIMX219_170,
                 working_zone_points_cv: np.array,  # not used atm
                 logger_full: utility.Logger,
                 data_collector: datacollection.DataCollector,  # not used atm
                 image_saver: utility.ImageSaver,  # not used atm
                 log_cur_dir,  # not used atm
                 periphery_det: detection.YoloOpenCVDetection,  # not used atm
                 precise_det: detection.YoloOpenCVDetection,
                 camera_positions: list,
                 pdz_distances: dict):

        self.__smoothie = smoothie
        self.__camera = camera
        self.__working_zone_points_cv = working_zone_points_cv
        self.__logger_full = logger_full
        self.__data_collector = data_collector
        self.__image_saver = image_saver
        self.__log_cur_dir = log_cur_dir
        self.__periphery_det = periphery_det
        self.__precise_det = precise_det
        self.__camera_positions = camera_positions
        self.__pdz_polygon = self.__pdz_dist_to_poly(pdz_distances)
        self.__extraction_map = ExtractionMap(config.EXTRACTION_MAP_CELL_SIZE_MM)
        self.__converter = PxToMMConverter()

    @staticmethod
    def __pdz_dist_to_poly(pdz: dict):
        """Converts related to scene center PDZ distances into px coordinates and returns that area as
        matplotlib.patches.Polygon
        """

        return Polygon([
            [config.SCENE_CENTER_X - pdz["left"], config.SCENE_CENTER_Y - pdz["top"]],
            [config.SCENE_CENTER_X + pdz["right"], config.SCENE_CENTER_Y - pdz["top"]],
            [config.SCENE_CENTER_X + pdz["right"], config.SCENE_CENTER_Y + pdz["bot"]],
            [config.SCENE_CENTER_X - pdz["left"], config.SCENE_CENTER_Y + pdz["bot"]]
        ])

    def scan_sectors(self):
        """Detects plants under robot's working zone, optimizes extractions order and returns list of plants as smoothie
        absolute coordinates (coordinates out of working range are skipped).
        """

        smoothie_plants_positions = []

        # loop over camera positions
        for cam_sm_x, cam_sm_y in self.__camera_positions:
            # move cork to camera position
            res = self.__smoothie.custom_move_to(config.XY_F_MAX, X=cam_sm_x, Y=cam_sm_y)
            if res != self.__smoothie.RESPONSE_OK:
                msg = f"Could not move cork to camera position (x={cam_sm_x}, y={cam_sm_y}) - \
                smoothie error occurred:\n" + res
                self.__logger_full.write(msg + "\n")
                self.__smoothie.wait_for_all_actions_done()
                continue
            self.__smoothie.wait_for_all_actions_done()

            # take a photo and look for a plants
            time.sleep(config.DELAY_BEFORE_2ND_SCAN)
            image = self.__camera.get_image()
            # get plants boxes and keep only that are in PDZ
            cur_pos_plant_boxes_pdz = list(filter(
                lambda box: self.is_point_in_poly(
                    box.center_x,
                    box.center_y,
                    self.__pdz_polygon),
                self.__precise_det.detect(image)))

            # convert plants boxes px coordinates into absolute smoothie coordinates
            for plant_box in cur_pos_plant_boxes_pdz:
                # calculate values to move camera over a plant from current position (coords are relative)
                rel_sm_x, rel_sm_y = self.__converter.convert_px_to_mm(plant_box.center_x, plant_box.center_y)

                # convert smoothie relative coordinates to absolute
                cur_sm_pos = self.__smoothie.get_adapter_current_coordinates()
                abs_sm_x, abs_sm_y = cur_sm_pos["X"] + rel_sm_x, cur_sm_pos["Y"] + rel_sm_y

                # skip coordinates that are out of working range
                if not (config.X_MIN < abs_sm_x < config.X_MAX and config.Y_MIN < abs_sm_y < config.Y_MAX):
                    continue

                # add absolute coordinates to the result list
                smoothie_plants_positions.append((abs_sm_x, abs_sm_y))

        return self.optimize_corkscrew_way(smoothie_plants_positions)

    def extract_all_plants(self):
        """Find and extract all plants found in current robot's position
        """

        self.__extraction_map.clear()

        # do sectored scans
        smoothie_positions = self.scan_sectors()
        msg = "Visiting initial " + str(len(smoothie_positions)) + " positions"
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
                # try to move to the plant rescan position, calibrate and try again if failed, exit app otherwise
                movement_messages = [
                    "Failed to move cork to the plant rescan position, smoothie's response:\n",
                    "Failed to move cork to the plant rescan position Y_MIN (after calibration), smoothie's response:\n"
                ]
                for idx, movement_message in enumerate(movement_messages):
                    res = self.__smoothie.custom_move_to(config.XY_F_MAX, X=cur_pos_sm_x, Y=cur_pos_sm_y)
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

                # do rescan using delta seeking if nothing detected, it was 1rst scan and delta seeking is allowed
                if len(cur_pos_plant_boxes_undist) == 0:
                    if scan_is_first:
                        msg = "No plants detected (plant was in working zone before)," \
                              "trying to do delta movement and find this plant"
                        self.__logger_full.write(msg + "\n")
                        if config.VERBOSE:
                            print(msg)

                        if config.ALLOW_DELTA_SEEKING:
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
                                # check if coordinates are in working range
                                if not (config.X_MIN < delta_sm_x < config.X_MAX and
                                        config.Y_MIN < delta_sm_y < config.Y_MAX):
                                    continue

                                # do movement
                                res = self.__smoothie.custom_move_to(config.XY_F_MAX, X=delta_sm_x, Y=delta_sm_y)
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
                                    break
                                else:
                                    msg = "No plants found during delta scan iteration"
                                    self.__logger_full.write(msg + "\n")
                            # go to next plant in PDZ list if delta seeking loop found nothing
                            else:
                                msg = "No plants found during all delta scans"
                                self.__logger_full.write(msg + "\n")
                                break
                    # go to next plant in PDZ list if no plants detected after extraction attempting
                    else:
                        msg = "No plants detected - assuming last extraction was successful; coming to next PDZ item"
                        self.__logger_full.write(msg + "\n")
                        break

                scan_is_first = False

                # convert plant boxes into smoothie absolute coordinates pairs
                smoothie_plants_positions = []
                for plant_box in cur_pos_plant_boxes_undist:
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
                    smoothie_plants_positions.append((abs_sm_x, abs_sm_y))

                # extract these plants
                for ext_sm_x, ext_sm_y in smoothie_plants_positions:
                    extraction_pattern = self.__extraction_map.get_strategy(ext_sm_x, ext_sm_y)
                    if extraction_pattern:
                        res, cork_is_stuck = extraction_pattern(self.__smoothie, self.__extraction_map)
                        if res != self.__smoothie.RESPONSE_OK:
                            msg = "Something gone wrong during extractions, smoothie's response:\n" + res
                            self.__logger_full.write(msg + "\n")
                            if cork_is_stuck:  # danger flag is True if smoothie couldn't pick up the corkscrew
                                msg = "Corkscrew is stuck! Emergency stopping."
                                self.__logger_full.write(msg + "\n")
                                exit(1)
                    else:
                        msg = "Did too many extraction tries at this position, no strategies to try left"
                        self.__logger_full.write(msg + "\n")

        # set camera back to the Y min X max / 2
        res = self.__smoothie.custom_move_to(config.XY_F_MAX,
                                             X=config.X_MAX / 2 / config.XY_COEFFICIENT_TO_MM,
                                             Y=config.Y_MIN)
        self.__smoothie.wait_for_all_actions_done()
        if res != self.__smoothie.RESPONSE_OK:
            msg = "Couldn't set camera back to cruise scan position after extractions, smoothie's response:\n" + res
            self.__logger_full.write(msg + "\n")

    def optimize_corkscrew_way(self, smoothie_coordinates: list):
        """Optimizes order of smoothie points to visit to reduce cork total path length during this list extractions"""

        # TODO: implement TSP optimization algorithm instead
        # reverse list to start from last (closest) positions checked
        return list(reversed(smoothie_coordinates))

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
            ExtractionMethods.pattern_x,
            ExtractionMethods.pattern_plus
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


class ExtractionMethods:
    """Contains methods for different plants extraction strategies. All methods should have similar signatures.
    """

    @staticmethod
    def single_center_drop(smoothie: adapters.SmoothieAdapter, extraction_map: ExtractionMap):
        """Extract a plant with a single corkscrew drop to the center"""

        # extraction, cork down
        res = smoothie.custom_move_for(F=config.Z_F_EXTRACTION_DOWN, Z=config.EXTRACTION_Z)
        smoothie.wait_for_all_actions_done()
        if res != smoothie.RESPONSE_OK:  # or smoothie.checkendstop("Z")==1
            msg = "Couldn't move the extractor down, smoothie error occurred:\n" + res
            return msg, False

        # extraction, cork up
        res = smoothie.ext_cork_up()
        smoothie.wait_for_all_actions_done()
        if res != smoothie.RESPONSE_OK:
            msg = "Couldn't move the extractor up, smoothie error occurred:\n" + res + \
                  "\nemergency exit as I don't want break corkscrew."
            return msg, True

        # make a record about this extraction to extraction matrix
        sm_cur_coords = smoothie.get_smoothie_current_coordinates()
        extraction_map.record_extraction(sm_cur_coords["X"], sm_cur_coords["Y"])

        # save current matrix state to a file
        if config.DEBUG_MATRIX_FILE:
            extraction_map.save_to_file("last_extraction_map.txt")

        return res, False

    @staticmethod
    def five_drops_near_center(smoothie: adapters.SmoothieAdapter, extraction_map: ExtractionMap):
        """Extract a plant with a single corkscrew drop to the center and four drops (distances are defined in config)
        """

        raise NotImplementedError("this function need to be updated as it's using old extraction matrix")

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

    @staticmethod
    def Daisy(smoothie: adapters.SmoothieAdapter, extraction_map: ExtractionMap):
        """This method is used for a daisy extraction"""

        raise NotImplementedError("this function need to be updated as it's using old extraction matrix")

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

    @staticmethod
    def Plantain(smoothie: adapters.SmoothieAdapter, extraction_map: ExtractionMap):
        """This method is used for a plantain extraction"""

        raise NotImplementedError("this function is not implemented yet")

    @staticmethod
    def Dandelion(smoothie: adapters.SmoothieAdapter, extraction_map: ExtractionMap):
        """This method is used for a dandelion extraction"""

        raise NotImplementedError("this function is not implemented yet")

    @staticmethod
    def pattern_plus(smoothie: adapters.SmoothieAdapter, extraction_map: ExtractionMap):

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
            res = smoothie.custom_move_to(config.XY_F_MAX, X=sm_x, Y=sm_y)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                msg = "Couldn't move corkscrew to the one of plant sides, smoothie error occurred:\n" + res
                return msg, False

            # do extraction
            res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, extraction_map)
            if res != smoothie.RESPONSE_OK:
                return res, cork_is_stuck

        return smoothie.RESPONSE_OK, False

    @staticmethod
    def pattern_x(smoothie: adapters.SmoothieAdapter, extraction_map: ExtractionMap):

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
            res = smoothie.custom_move_to(config.XY_F_MAX, X=sm_x, Y=sm_y)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                msg = "Couldn't move corkscrew to the one of plant sides, smoothie error occurred:\n" + res
                return msg, False

            # do extraction
            res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, extraction_map)
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
        return self.__plant_position_prediction(norm_x, norm_y, dis_from_center)

    # prediction of the distance in mm between the center of the plant and the center of the scene
    def __plant_position_prediction(self, norm_x, norm_y,
                                    dis_from_center):  # train the model and get .sav with prediction.py before

        degree = None

        for threshold, _degree in config.ZONE_THRESHOLD_DEGREE.items():
            if dis_from_center <= threshold:
                degree = _degree
                break

        if degree is None:
            raise ValueError("Incorrect data !")

        values = [norm_x, norm_y, dis_from_center]

        # generate the matrix of the right degree
        poly_features = sklearn.preprocessing.PolynomialFeatures(degree=degree)

        if degree not in self.__loaded_prediction_model.keys():
            filename = 'zone_models/zone_model_' + str(degree) + '.sav'
            self.__loaded_prediction_model[degree] = pickle.load(
                open(filename, 'rb'))  # open the model trained

        data = poly_features.fit_transform([values])  # charge the data into the matrix
        return self.__loaded_prediction_model[degree].predict(data)[0]  # model.predict() comes from scikit
