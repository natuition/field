from config import config
import adapters
import detection
import math
import numpy as np
from extraction_manager import ExtractionManager
import utility
import datacollection

DEBUG = False

class ExtractionMethods:
    """
    Contains methods for different plants extraction.
    All methods should have similar signatures.
    """

    @staticmethod
    def single_center_drop(smoothie: adapters.SmoothieAdapter, plant_box: detection.DetectedPlantBox, extraction_map: np.matrix, parent=None):
        """Extract a plant with a single corkscrew drop to the center"""

        # extraction, cork down
        # TODO: calculation -Z depending on box size
        res = smoothie.custom_move_for(F=config.Z_F_EXTRACTION_DOWN, Z=config.EXTRACTION_Z)
        smoothie.wait_for_all_actions_done()
        if res != smoothie.RESPONSE_OK:#or smoothie.checkendstop("Z")==1
            msg = "Couldn't move the extractor down, smoothie error occurred:\n" + res
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
            if parent is None:
                extraction_map[y,x].setRootExtraction()
            else:
                extraction_map[y,x].setPatternExtraction(parent.lastPattern,parent)
            if DEBUG:
                ExtractionManager.save_matrix("last_extraction_map.txt",extraction_map, header=True)

        return res, False

    @staticmethod
    def five_drops_near_center(smoothie: adapters.SmoothieAdapter, plant_box: detection.DetectedPlantBox, extraction_map: np.matrix):
        """Extract a plant with a single corkscrew drop to the center and four drops (distances are defined in config)"""

        # mms to move near plant's center
        x = config.ADDITIONAL_EXTRACTIONS_DISTANCE_X
        y = config.ADDITIONAL_EXTRACTIONS_DISTANCE_Y

        # drop cork to the center
        res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box, extraction_map)
        if res != smoothie.RESPONSE_OK:
            return res, cork_is_stuck
        else:
            sm_x = math.floor(smoothie.get_smoothie_current_coordinates()["X"] / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
            sm_y = math.floor(smoothie.get_smoothie_current_coordinates()["Y"] / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
            extraction_map[sm_y,sm_x].setPatternExtraction("five_drops_near_center")

        for x_shift, y_shift in [[0, y], [-x, -y], [x, -y], [x, y]]:
            # move to the position
            res = smoothie.custom_move_for(config.XY_F_MAX, X=x_shift, Y=y_shift)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                msg = "Aborting movement to the corner (couldn't reach): " + res
                return msg, False

            # extraction, cork down
            # TODO: calculation -Z depending on box size
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
                extraction_map[y,x].setPatternExtraction("five_drops_near_center", parent=extraction_map[sm_y,sm_x])
                if DEBUG:
                    ExtractionManager.save_matrix("last_extraction_map.txt",extraction_map, header=True)

        return res, False

    @staticmethod
    def Daisy(smoothie: adapters.SmoothieAdapter, plant_box: detection.DetectedPlantBox, extraction_map: np.matrix):
        """This method is used for a daisy extraction"""

        # drop cork to the center
        res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box, extraction_map)
        if res != smoothie.RESPONSE_OK:
            return res, cork_is_stuck
        else:
            center_sm_x = math.floor(smoothie.get_smoothie_current_coordinates()["X"] / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
            center_sm_y = math.floor(smoothie.get_smoothie_current_coordinates()["Y"] / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
            extraction_map[center_sm_y,center_sm_x].setPatternExtraction("Daisy")

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
            # TODO: calculation -Z depending on box size
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
                extraction_map[y,x].setPatternExtraction("Daisy", parent=extraction_map[center_sm_y,center_sm_x])
                if DEBUG:
                    ExtractionManager.save_matrix("last_extraction_map.txt",extraction_map, header=True)

        return res, False
    '''
    @staticmethod
    def Plantain(smoothie: adapters.SmoothieAdapter, plant_box: detection.DetectedPlantBox, extraction_map: np.matrix):
        """This method is used for a plantain extraction"""

        pass

    @staticmethod
    def Dandelion(smoothie: adapters.SmoothieAdapter, plant_box: detection.DetectedPlantBox, extraction_map: np.matrix):
        """This method is used for a dandelion extraction"""

        pass
    '''

    @staticmethod
    def pattern_plus(smoothie: adapters.SmoothieAdapter, plant_box: detection.DetectedPlantBox, extraction_map: np.matrix):

        x = math.floor(smoothie.get_smoothie_current_coordinates()["X"] / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
        y = math.floor(smoothie.get_smoothie_current_coordinates()["Y"] / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL

        offset = math.floor(config.OFFSET_FOR_MATRIX_PATTERN_IN_MM / config.MATRIX_ONE_MATRICE_CELL_IN_MM)

        res = smoothie.RESPONSE_OK

        extraction_map[y,x].setPatternExtraction("pattern_plus")

        if ExtractionManager.is_in_matrice(extraction_map,Y=y+offset):
            if extraction_map[y+offset,x] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork top of plant, smoothie error occurred:\n" + res
                    return msg, False

                res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box, extraction_map, parent=extraction_map[y,x])
                if res != smoothie.RESPONSE_OK:
                    return res, cork_is_stuck

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork center of plant, smoothie error occurred:\n" + res
                    return msg, False

        if ExtractionManager.is_in_matrice(extraction_map,Y=y-offset):
            if extraction_map[y-offset,x] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork bottom of plant, smoothie error occurred:\n" + res
                    return msg, False

                res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box, extraction_map, parent=extraction_map[y,x])
                if res != smoothie.RESPONSE_OK:
                    return res, cork_is_stuck

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork center of plant, smoothie error occurred:\n" + res
                    return msg, False

        if ExtractionManager.is_in_matrice(extraction_map,X=x+offset):
            if extraction_map[y,x+offset] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork right of plant, smoothie error occurred:\n" + res
                    return msg, False

                res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box, extraction_map, parent=extraction_map[y,x])
                if res != smoothie.RESPONSE_OK:
                    return res, cork_is_stuck

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork center of plant, smoothie error occurred:\n" + res
                    return msg, False

        if ExtractionManager.is_in_matrice(extraction_map,X=x-offset):
            if extraction_map[y,x-offset] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork left of plant, smoothie error occurred:\n" + res
                    return msg, False

                res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box, extraction_map, parent=extraction_map[y,x])
                if res != smoothie.RESPONSE_OK:
                    return res, cork_is_stuck

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork center of plant, smoothie error occurred:\n" + res
                    return msg, False

        return res, False

    @staticmethod
    def pattern_x(smoothie: adapters.SmoothieAdapter, plant_box: detection.DetectedPlantBox, extraction_map: np.matrix):

        x = math.floor(smoothie.get_smoothie_current_coordinates()["X"] / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
        y = math.floor(smoothie.get_smoothie_current_coordinates()["Y"] / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + config.OFFSET_FOR_MATRIX_BORDER_IN_CELL

        offset = math.floor(config.OFFSET_FOR_MATRIX_PATTERN_IN_MM / config.MATRIX_ONE_MATRICE_CELL_IN_MM)

        res = smoothie.RESPONSE_OK

        extraction_map[y,x].setPatternExtraction("pattern_x")

        if ExtractionManager.is_in_matrice(extraction_map,Y=y+offset,X=x+offset):
            if extraction_map[y+offset,x+offset] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM, X=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork top right of plant, smoothie error occurred:\n" + res
                    return msg, False

                res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box, extraction_map, parent=extraction_map[y,x])
                if res != smoothie.RESPONSE_OK:
                    return res, cork_is_stuck

                res = smoothie.custom_move_for(config.XY_F_MAX, Y=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM, X=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork center of plant, smoothie error occurred:\n" + res
                    return msg, False

        if ExtractionManager.is_in_matrice(extraction_map,X=x+offset,Y=y-offset):
            if extraction_map[y-offset,x+offset] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM, Y=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork bottom right of plant, smoothie error occurred:\n" + res
                    return msg, False

                res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box, extraction_map, parent=extraction_map[y,x])
                if res != smoothie.RESPONSE_OK:
                    return res, cork_is_stuck

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM, Y=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork center plant, smoothie error occurred:\n" + res
                    return msg, False

        if ExtractionManager.is_in_matrice(extraction_map,Y=y-offset,X=x-offset):
            if extraction_map[y-offset,x-offset] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM, X=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork bottom left of plant, smoothie error occurred:\n" + res
                    return msg, False

                res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box, extraction_map, parent=extraction_map[y,x])
                if res != smoothie.RESPONSE_OK:
                    return res, cork_is_stuck

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM, X=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork center plant, smoothie error occurred:\n" + res
                    return msg, False

        if ExtractionManager.is_in_matrice(extraction_map,X=x-offset, Y=y+offset):
            if extraction_map[y+offset,x-offset] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM, Y=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork top left of plant, smoothie error occurred:\n" + res
                    return msg, False

                res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box, extraction_map, parent=extraction_map[y,x])
                if res != smoothie.RESPONSE_OK:
                    return res, cork_is_stuck

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=config.OFFSET_FOR_MATRIX_PATTERN_IN_MM, Y=-config.OFFSET_FOR_MATRIX_PATTERN_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork center plant, smoothie error occurred:\n" + res
                    return msg, False
        
        return res, False
