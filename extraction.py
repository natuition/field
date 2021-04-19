from config import config
import adapters
import detection
import math
import numpy as np
from extraction_manager import ExtractionManager
import utility
import datacollection

class ExtractionMethods:
    """
    Contains methods for different plants extraction.
    All methods should have similar signatures.
    """

    @staticmethod
    def single_center_drop(smoothie: adapters.SmoothieAdapter, plant_box: detection.DetectedPlantBox, extraction_map: np.matrix):
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
            x = math.ceil(sm_x / config.MATRIX_ONE_MATRICE_CELL_IN_MM)
            y = math.ceil(sm_y / config.MATRIX_ONE_MATRICE_CELL_IN_MM)
            extraction_map[y,x] = config.MATRIX_EXTRACTION
            #Only for debug
            np.savetxt("last_extraction_map.txt",extraction_map,fmt='%d')

        return res, False

    @staticmethod
    def five_drops_near_center(smoothie: adapters.SmoothieAdapter, plant_box: detection.DetectedPlantBox, extraction_map: np.matrix):
        """Extract a plant with a single corkscrew drop to the center and four drops (distances are defined in config)"""

        # mms to move near plant's center
        x = config.ADDITIONAL_EXTRACTIONS_DISTANCE_X
        y = config.ADDITIONAL_EXTRACTIONS_DISTANCE_Y

        # drop cork to the center
        res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box)
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
                x = math.ceil(sm_x / config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                y = math.ceil(sm_y / config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                extraction_map[y,x] = config.MATRIX_EXTRACTION
                #Only for debug
                np.savetxt("last_extraction_map.txt",extraction_map,fmt='%d')

        return res, False

    @staticmethod
    def Daisy(smoothie: adapters.SmoothieAdapter, plant_box: detection.DetectedPlantBox, extraction_map: np.matrix):
        """This method is used for a daisy extraction"""

        # drop cork to the center
        res, cork_is_stuck = ExtractionMethods.single_center_drop(smoothie, plant_box, extraction_map)
        if res != smoothie.RESPONSE_OK:
            return res, cork_is_stuck

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
                x = math.ceil(sm_x / config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                y = math.ceil(sm_y / config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                extraction_map[y,x] = config.MATRIX_EXTRACTION
                #Only for debug
                np.savetxt("last_extraction_map.txt",extraction_map,fmt='%d')

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
    def pattern_plus(smoothie: adapters.SmoothieAdapter, box: detection.DetectedPlantBox, 
                     logger_full: utility.Logger, data_collector: datacollection.DataCollector,
                     extraction_map: np.matrix, log_cur_dir, x: int, y: int):

        if ExtractionManager.is_in_matrice(extraction_map,Y=y+1):
            if extraction_map[y+1,x] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

                extract_one_plant(smoothie, box, logger_full, data_collector, extraction_map, log_cur_dir)

                res = smoothie.custom_move_for(config.XY_F_MAX, Y=-config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

        if ExtractionManager.is_in_matrice(extraction_map,Y=y-1):
            if extraction_map[y-1,x] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=-config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

                extract_one_plant(smoothie, box, logger_full, data_collector, extraction_map, log_cur_dir)

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

        if ExtractionManager.is_in_matrice(extraction_map,X=x+1):
            if extraction_map[y,x+1] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

                extract_one_plant(smoothie, box, logger_full, data_collector, extraction_map, log_cur_dir)

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=-config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

        if ExtractionManager.is_in_matrice(extraction_map,X=x-1):
            if extraction_map[y,x-1] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=-config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

                extract_one_plant(smoothie, box, logger_full, data_collector, extraction_map, log_cur_dir)

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False
        
        return True

    @staticmethod
    def pattern_x(smoothie: adapters.SmoothieAdapter, box: detection.DetectedPlantBox, 
                  logger_full: utility.Logger, data_collector: datacollection.DataCollector,
                  extraction_map: np.matrix, log_cur_dir, x: int, y: int):

        if ExtractionManager.is_in_matrice(extraction_map,Y=y+1,X=x+1):
            if extraction_map[y+1,x+1] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=config.MATRIX_ONE_MATRICE_CELL_IN_MM, X=config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

                extract_one_plant(smoothie, box, logger_full, data_collector, extraction_map, log_cur_dir)

                res = smoothie.custom_move_for(config.XY_F_MAX, Y=-config.MATRIX_ONE_MATRICE_CELL_IN_MM, X=-config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

        if ExtractionManager.is_in_matrice(extraction_map,Y=y-1,X=x-1):
            if extraction_map[y-1,x-1] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=-config.MATRIX_ONE_MATRICE_CELL_IN_MM, X=-config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

                extract_one_plant(smoothie, box, logger_full, data_collector, extraction_map, log_cur_dir)

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, Y=config.MATRIX_ONE_MATRICE_CELL_IN_MM, X=config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

        if ExtractionManager.is_in_matrice(extraction_map,X=x+1,Y=y-1):
            if extraction_map[y-1,x+1] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=config.MATRIX_ONE_MATRICE_CELL_IN_MM, Y=-config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

                extract_one_plant(smoothie, box, logger_full, data_collector, extraction_map, log_cur_dir)

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=-config.MATRIX_ONE_MATRICE_CELL_IN_MM, Y=config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

        if ExtractionManager.is_in_matrice(extraction_map,X=x-1, Y=y+1):
            if extraction_map[y+1,x-1] != 0:

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=-config.MATRIX_ONE_MATRICE_CELL_IN_MM, Y=config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False

                extract_one_plant(smoothie, box, logger_full, data_collector, extraction_map, log_cur_dir)

                # move
                res = smoothie.custom_move_for(config.XY_F_MAX, X=config.MATRIX_ONE_MATRICE_CELL_IN_MM, Y=-config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Couldn't move cork around plant, smoothie error occurred:\n" + res
                    logger_full.write(msg + "\n")
                    return False
        
        return True
