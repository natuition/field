import adapters
import detection
import datacollection
import numpy as np
from config import config
import math
import time
from matplotlib.patches import Polygon
import utility
import extraction

DEBUG = False

class ExtractionManager:

    def __init__(self, smoothie: adapters.SmoothieAdapter, camera: adapters.CameraAdapterIMX219_170,
                 working_zone_polygon: Polygon, working_zone_points_cv: np.array,
                 logger_full: utility.Logger, data_collector: datacollection.DataCollector,
                 image_saver: utility.ImageSaver, log_cur_dir, periphery_det: detection.YoloOpenCVDetection, precise_det: detection.YoloOpenCVDetection):

        self.smoothie = smoothie
        self.camera = camera
        self.working_zone_polygon = working_zone_polygon
        self.logger_full = logger_full
        self.data_collector = data_collector
        self.image_saver = image_saver
        self.log_cur_dir = log_cur_dir
        self.undistorted_zone_radius = config.UNDISTORTED_ZONE_RADIUS
        self.working_zone_points_cv = working_zone_points_cv
        self.periphery_det = periphery_det
        self.precise_det = precise_det

        self.offsetMatriceBorder = config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
        self.numberMatriceLines = math.ceil(config.Y_MAX / config.XY_COEFFICIENT_TO_MM / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + self.offsetMatriceBorder*2
        self.numberMatriceColumns = math.ceil(config.X_MAX / config.XY_COEFFICIENT_TO_MM / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + self.offsetMatriceBorder*2
        self.reset_map()

    def reset_map(self):
        self.detection_map = np.array([[DetectionMapCell(j,i) for j in range(self.numberMatriceColumns)] for i in range(self.numberMatriceLines)], DetectionMapCell)  
        self.extraction_map = np.array([[ExtractionMapCell(j,i) for j in range(self.numberMatriceColumns)] for i in range(self.numberMatriceLines)], ExtractionMapCell)
        if DEBUG:
            ExtractionManager.save_matrix("last_detection_map.txt",self.detection_map)    
            ExtractionManager.save_matrix("last_extraction_map.txt",self.extraction_map, header=True) 

    def extraction_control(self, plants_boxes, img_output_dir, vesc_engine, close_to_end, current_working_mode):

        working_mode_slow = 1
        working_mode_switching = 2
        working_mode_fast = 3

        slow_mode_time = -float("inf")
                                       
        if config.AUDIT_MODE:
            dc_start_t = time.time()

            # count detected plant boxes for each type
            plants_count = dict()
            for plant_box in plants_boxes:
                plant_box_name = plant_box.get_name()
                if plant_box_name in plants_count:
                    plants_count[plant_box_name] += 1
                else:
                    plants_count[plant_box_name] = 1

            # save info into data collector
            for plant_label in plants_count:
                self.data_collector.add_detections_data(plant_label, math.ceil((plants_count[plant_label])/config.AUDIT_DIVIDER))

            # flush updates into the audit output file and log measured time
            if len(plants_boxes) > 0:
                self.data_collector.save_detections_data(self.log_cur_dir + config.AUDIT_OUTPUT_FILE)

            dc_t = time.time() - dc_start_t
            msg = "Last scan weeds detected: " + str(len(plants_boxes)) +\
                  ", audit processing tick time: " + str(dc_t)
            self.logger_full.write(msg + "\n")
        else:
            # slow mode
            if current_working_mode == working_mode_slow:
                if DEBUG:
                    print("[Working mode] : slow")
                if ExtractionManager.any_plant_in_zone(plants_boxes, self.working_zone_polygon):
                    vesc_engine.stop_moving()

                    for i in range(1, config.EXTRACTIONS_FULL_CYCLES + 1):

                        time.sleep(config.DELAY_BEFORE_2ND_SCAN)

                        msg = "Extraction cycle " + str(i) + " of " + str(config.EXTRACTIONS_FULL_CYCLES)
                        self.logger_full.write(msg + "\n")
                        if DEBUG:
                            print(msg)

                        start_work_t = time.time()
                        frame = self.camera.get_image()
                        frame_t = time.time()
                        plants_boxes = self.precise_det.detect(frame)
                        pre_det_t = time.time()

                        if config.SAVE_DEBUG_IMAGES:
                            self.image_saver.save_image(frame, img_output_dir, label="(precise view scan 2 M=1)",
                                                   plants_boxes=plants_boxes)

                        msg = "Work frame time: " + str(frame_t - start_work_t) + "\t\tPrec. det. 2 time: " + \
                              str(pre_det_t - frame_t)
                        self.logger_full.write(msg + "\n")

                        if ExtractionManager.any_plant_in_zone(plants_boxes, self.working_zone_polygon):

                            if i == 1 and config.ALLOW_DETECT_AND_EXTRACT_GROUP:

                                for plant_box in plants_boxes:
                                    box_x,box_y = plant_box.get_center_points()

                                    if ExtractionManager.is_point_in_circle(box_x, box_y, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, self.undistorted_zone_radius):
                                        x,y = box_x,box_y

                                        x_center = math.floor(x / config.ONE_MM_IN_PX / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + self.offsetMatriceBorder
                                        y_center = math.floor(y / config.ONE_MM_IN_PX / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + self.offsetMatriceBorder

                                    else:
                                        p_x, p_y, x, y, index = ExtractionManager.get_closest_control_point(box_x, box_y, config.IMAGE_CONTROL_POINTS_MAP)
                                        
                                        x_center = math.floor((config.X_MAX/2/config.XY_COEFFICIENT_TO_MM + x) / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + self.offsetMatriceBorder
                                        y_center = math.floor((config.Y_MAX/config.XY_COEFFICIENT_TO_MM - y) / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + self.offsetMatriceBorder

                                    radiusSize_x = math.floor(plant_box.get_sizes()[0] / config.ONE_MM_IN_PX / 2 / config.MATRIX_ONE_MATRICE_CELL_IN_MM)
                                    radiusSize_y = math.floor(plant_box.get_sizes()[1] / config.ONE_MM_IN_PX / 2 / config.MATRIX_ONE_MATRICE_CELL_IN_MM)

                                    y_min = math.floor(y_center-radiusSize_y)
                                    y_max = math.floor(y_center+radiusSize_y+1)
                                    x_min = math.floor(x_center-radiusSize_x)
                                    x_max = math.floor(x_center+radiusSize_x+1)

                                    self.detection_map[y_center,x_center].setRoot(plant_box.get_name(), plants_boxes.index(plant_box))
                                    for y_leaf in range(y_min,y_max):
                                        for x_leaf in range(x_min,x_max):
                                            if y_leaf != y_center or x_leaf != x_center:
                                                self.detection_map[y_leaf,x_leaf].setLeaf(self.detection_map[y_center,x_center],plant_box.get_name(), plants_boxes.index(plant_box))    

                                if DEBUG:
                                    ExtractionManager.save_matrix("last_detection_map.txt",self.detection_map)

                                self.extract_all_groups(plants_boxes)
                            
                            # set camera to the Y min
                            res = self.smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2 / config.XY_COEFFICIENT_TO_MM, Y=config.Y_MIN)
                            if res != self.smoothie.RESPONSE_OK:
                                msg = "INIT: Failed to move camera to Y min X max/2, smoothie response:\n" + res
                                self.logger_full.write(msg + "\n")
                            self.smoothie.wait_for_all_actions_done() 

                            self.extract_all_plants(frame, plants_boxes, img_output_dir)


                        else:
                            msg = "View scan 2 found no plants in working zone."
                            self.logger_full.write(msg + "\n")
                            break

                    # force step forward to avoid infinite loop after extraction (if NN triggers on extracted plants)
                    msg = "Applying force step forward after extractions cycle(s)"
                    self.logger_full.write(msg + "\n")
                    if DEBUG:
                        print(msg)

                    self.reset_map()

                    vesc_engine.set_moving_time(config.STEP_FORWARD_TIME)
                    vesc_engine.set_rpm(config.STEP_FORWARD_RPM)
                    vesc_engine.start_moving()
                    vesc_engine.wait_for_stop()
                    vesc_engine.set_moving_time(config.VESC_MOVING_TIME)
                    vesc_engine.set_rpm(config.VESC_RPM_SLOW)
                    return

                elif not ExtractionManager.any_plant_in_zone(plants_boxes, self.working_zone_polygon) and \
                        time.time() - slow_mode_time > config.SLOW_MODE_MIN_TIME and \
                        config.SLOW_FAST_MODE:
                    """
                    # set camera to the Y max
                    res = smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2 / config.XY_COEFFICIENT_TO_MM,
                                                  Y=config.Y_MAX / config.XY_COEFFICIENT_TO_MM)
                    if res != smoothie.RESPONSE_OK:
                        msg = "M=" + str(current_working_mode) + ": " + "Failed to move to Y max, smoothie response:\n" + res
                        logger_full.write(msg + "\n")
                    smoothie.wait_for_all_actions_done()
                    """
                    current_working_mode = working_mode_fast
                    if not close_to_end:
                        vesc_engine.apply_rpm(config.VESC_RPM_FAST)
                vesc_engine.start_moving()  

            # switching to fast mode
            elif current_working_mode == working_mode_switching:
                if DEBUG:
                    print("[Working mode] : switching")
                if ExtractionManager.any_plant_in_zone(plants_boxes, self.working_zone_polygon):
                    vesc_engine.stop_moving()
                    """
                    # set camera to the Y min
                    res = smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2 / config.XY_COEFFICIENT_TO_MM,
                                                  Y=config.Y_MIN)
                    if res != smoothie.RESPONSE_OK:
                        msg = "M=" + str(current_working_mode) + ": " + "Failed to move to Y min, smoothie response:\n" + res
                        logger_full.write(msg + "\n")
                    smoothie.wait_for_all_actions_done()
                    """
                    current_working_mode = working_mode_slow
                    slow_mode_time = time.time()
                # elif smoothie.get_smoothie_current_coordinates(False)["Y"] + config.XY_COEFFICIENT_TO_MM * 20 > config.Y_MAX:
                else:
                    current_working_mode = working_mode_fast
                    if not close_to_end:
                        vesc_engine.apply_rpm(config.VESC_RPM_FAST)

            # fast mode
            elif current_working_mode == working_mode_fast:
                if DEBUG:
                    print("[Working mode] : fast")
                if ExtractionManager.any_plant_in_zone(plants_boxes, self.working_zone_polygon):
                    vesc_engine.stop_moving()
                    vesc_engine.apply_rpm(config.FAST_TO_SLOW_RPM)
                    time.sleep(config.FAST_TO_SLOW_TIME)
                    vesc_engine.stop_moving()
                    """
                    # set camera to the Y min
                    res = smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2 / config.XY_COEFFICIENT_TO_MM,
                                                  Y=config.Y_MIN)
                    if res != smoothie.RESPONSE_OK:
                        msg = "M=" + str(current_working_mode) + ": " + "Failed to move to Y min, smoothie response:\n" + res
                        logger_full.write(msg + "\n")
                    smoothie.wait_for_all_actions_done()
                    """
                    current_working_mode = working_mode_slow
                    slow_mode_time = time.time()
                    vesc_engine.set_rpm(config.VESC_RPM_SLOW)
                elif close_to_end:
                    vesc_engine.apply_rpm(config.VESC_RPM_SLOW)
                else:
                    vesc_engine.apply_rpm(config.VESC_RPM_FAST)

    def extract_all_groups(self, plants_boxes: list):
        time.sleep(2)
        groups = dict()
        cpt = 1
        for idx, obj in np.ndenumerate(self.detection_map):
            if obj.getValue() >= config.GROUP_THRESHOLD and not obj.isRegister:
                print("Find group !")
                groups[cpt] = set()
                groups[cpt].add(obj)
                obj.isRegister = True
                parents = list(obj.herParents)

                for parent in parents:
                    if not parent.isRegister:
                        groups[cpt].add(parent)
                        parent.isRegister = True

                        for child in parent.herChildren:
                            if not child.isRegister:
                                groups[cpt].add(child)
                                child.isRegister = True
                                
                                for childParent in child.herParents:
                                    if not childParent.isRegister:
                                        parents.append(childParent)

                cpt += 1

        if groups:

            msg = "Groups are found, let's extract them..."
            self.logger_full.write(msg + "\n")

            if DEBUG:
                print(msg)

            shootList = list()

            for groupNumber,group in groups.items():
                shootCoordinate = set()
                for element in group:
                    shootCoordinate.add((element.x, element.y, element.type, element.value))
                shootList += list(shootCoordinate)

            shootList.sort(key = lambda tup: (tup[1],tup[0])) 

            y_max,x_max = np.shape(self.extraction_map)[0]-2*config.OFFSET_FOR_MATRIX_BORDER_IN_CELL,np.shape(self.extraction_map)[1]-2*config.OFFSET_FOR_MATRIX_BORDER_IN_CELL
            
            shoot_step = math.ceil(config.OFFSET_FOR_MATRIX_PATTERN_IN_MM/config.MATRIX_ONE_MATRICE_CELL_IN_MM)

            last_y, last_x, cpt_x, cpt_y = (0,0,0,0)

            for shoot in shootList:
                x = (shoot[0]-config.OFFSET_FOR_MATRIX_BORDER_IN_CELL+0.5) * config.MATRIX_ONE_MATRICE_CELL_IN_MM
                y = (shoot[1]-config.OFFSET_FOR_MATRIX_BORDER_IN_CELL+0.5) * config.MATRIX_ONE_MATRICE_CELL_IN_MM
                if last_y != y:
                    if cpt_x != 0:
                        cpt_y += 1
                    cpt_x = 0
                last_y,last_x = (y,x)
                if (cpt_x%shoot_step==0 and cpt_y%shoot_step==0) or shoot[3]==1:
                    if x >= config.X_MIN*config.XY_COEFFICIENT_TO_MM and x <= config.X_MAX*config.XY_COEFFICIENT_TO_MM \
                       and y >= config.Y_MIN*config.XY_COEFFICIENT_TO_MM and y <= config.Y_MAX*config.XY_COEFFICIENT_TO_MM:
                        res = self.smoothie.custom_move_to(config.XY_F_MAX, X=x, Y=y)
                        if res != self.smoothie.RESPONSE_OK:
                            msg = "Failed to move cork to the extraction position to the group :\n" + res
                            self.logger_full.write(msg + "\n")
                            exit(1)
                        self.smoothie.wait_for_all_actions_done()
                        if shoot[3]==1: 
                            label = "root_of_" + shoot[2] + "__groups'"
                        else:  
                            label =  "leaf_of_" + shoot[2] + "__groups'"
                        self.extract_one_plant(detection.DetectedPlantBox(0, 0, 0, 0, label, 0, 0, 0, 0), pattern="single_center_drop")
                cpt_x+=1

            msg = "Extraction of groups is finished."
            self.logger_full.write(msg + "\n")
            if DEBUG:
                print(msg)

    def extract_all_plants(self, frame, plant_boxes: list, img_output_dir):
        """Extract all plants found in current position"""

        smoothie = self.smoothie
        camera = self.camera
        detector = self.precise_det
        working_zone_polygon = self.working_zone_polygon
        undistorted_zone_radius = self.undistorted_zone_radius
        working_zone_points_cv = self.working_zone_points_cv
        logger_full = self.logger_full
        data_collector = self.data_collector
        log_cur_dir = self.log_cur_dir
        image_saver = self.image_saver

        msg = "Extracting " + str(len(plant_boxes)) + " plants"
        logger_full.write(msg + "\n")
        if DEBUG:
            print(msg)

        # loop over all detected plants
        for box in plant_boxes:
            # go to the extraction position Y min
            res = smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2 / config.XY_COEFFICIENT_TO_MM, Y=config.Y_MIN)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                msg = "Failed to move cork to the extraction position Y_MIN, smoothie's response:\n" + res
                logger_full.write(msg + "\n")

                msg = "Trying to calibrate cork"
                logger_full.write(msg + "\n")
                res = smoothie.ext_calibrate_cork()
                if res != smoothie.RESPONSE_OK:
                    msg = "Failed to calibrate cork, smoothie's response:\n" + res
                    logger_full.write(msg + "\n")
                    exit(1)

                # go to the extraction position Y min again
                res = smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2 / config.XY_COEFFICIENT_TO_MM,
                                            Y=config.Y_MIN)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    msg = "Failed to move cork to the extraction position Y_MIN (again, after calibration), smoothie's response:\n" + res
                    logger_full.write(msg + "\n")
                    exit(1)

            plant_position_is_precise = not config.ALLOW_PRECISE_RESCAN
            box_x, box_y = box.get_center_points()

            # if plant is in working zone (can be reached by cork)
            if ExtractionManager.is_point_in_poly(box_x, box_y, working_zone_polygon):
                # extraction loop
                for _ in range(config.EXTRACTION_TUNING_MAX_COUNT):
                    if DEBUG:
                        print(f"Turn n°{_}")
                    box_x, box_y = box.get_center_points()

                    # if plant inside undistorted zone
                    if ExtractionManager.is_point_in_circle(box_x, box_y, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, undistorted_zone_radius):
                        msg = "Plant " + str(box) + " is in undistorted zone"
                        logger_full.write(msg + "\n")
                        if DEBUG:
                            print(msg)

                        # use plant box from precise NN for movement calculations
                        if not plant_position_is_precise:
                            time.sleep(config.DELAY_BEFORE_2ND_SCAN)
                            frame = camera.get_image()
                            temp_plant_boxes = detector.detect(frame)

                            # debug image saving
                            if config.SAVE_DEBUG_IMAGES:
                                image_saver.save_image(frame, img_output_dir, label="(increasing precision)",
                                                    plants_boxes=plant_boxes)

                            # check case if no plants detected
                            if len(temp_plant_boxes) == 0:
                                msg = "No plants detected (plant was in undistorted zone before), trying to move on next item"
                                logger_full.write(msg + "\n")
                                if DEBUG:
                                    print(msg)
                                break

                            # get closest box (update current box from main list coordinates after moving closer)
                            box = ExtractionManager.min_plant_box_dist(temp_plant_boxes, config.SCENE_CENTER_X, config.SCENE_CENTER_Y)
                            box_x, box_y = box.get_center_points()

                            # check if box still in undistorted zone
                            if not ExtractionManager.is_point_in_circle(box_x, box_y, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, undistorted_zone_radius):
                                msg = "No plants in undistorted zone (plant was in undistorted zone before), trying to move on next item"
                                logger_full.write(msg + "\n")
                                if DEBUG:
                                    print(msg)
                                continue

                        # calculate values to move camera over a plant
                        sm_x = ExtractionManager.px_to_smoothie_value(box_x, config.SCENE_CENTER_X, config.ONE_MM_IN_PX)
                        sm_y = -ExtractionManager.px_to_smoothie_value(box_y, config.SCENE_CENTER_Y, config.ONE_MM_IN_PX)
                        cam_sm_x = sm_x
                        cam_sm_y = sm_y
                        # swap camera and cork for extraction immediately
                        sm_x += config.CORK_TO_CAMERA_DISTANCE_X
                        sm_y += config.CORK_TO_CAMERA_DISTANCE_Y

                        msg = "box_x:{0} box_y:{1} cam_sm_x:{2} cam_sm_y:{3} sm_x:{4} sm_y:{5} scene_center_x:{6} scene_center_y:{7} one_mm_in_px:{8}".format(
                            box_x, box_y, cam_sm_x, cam_sm_y, sm_x, sm_y, config.SCENE_CENTER_X, config.SCENE_CENTER_Y,
                            config.ONE_MM_IN_PX)
                        logger_full.write(msg + "\n")

                        # move cork over a plant
                        res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                        smoothie.wait_for_all_actions_done()
                        if res != smoothie.RESPONSE_OK:
                            msg = "Couldn't move cork over plant, smoothie error occurred:\n" + res
                            logger_full.write(msg + "\n")
                            break

                        # debug image saving
                        if config.SAVE_DEBUG_IMAGES:
                            time.sleep(config.DELAY_BEFORE_2ND_SCAN)
                            frame = camera.get_image()
                            image_saver.save_image(frame, img_output_dir, label="(before first cork down)")

                        x = math.floor(smoothie.get_smoothie_current_coordinates()["X"] / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + self.offsetMatriceBorder
                        y = math.floor(smoothie.get_smoothie_current_coordinates()["Y"] / config.MATRIX_ONE_MATRICE_CELL_IN_MM) + self.offsetMatriceBorder

                        offset = math.floor(config.OFFSET_FOR_MATRIX_PATTERN_IN_MM / config.MATRIX_ONE_MATRICE_CELL_IN_MM)

                        """
                        self.extract_one_plant(box,pattern="pattern_x")
                        self.extract_one_plant(box,pattern="pattern_plus")
                        """

                        if self.extraction_map[y,x].hasDrop:

                            if self.extraction_map[y,x].isRootExtraction:

                                if self.extraction_map[y,x].lastPattern is None:

                                    self.extract_one_plant(box,pattern="pattern_plus")

                                elif not self.extraction_map[y,x].dictPattern["pattern_x"]:

                                    self.extract_one_plant(box,pattern="pattern_x")

                        else:
                            self.extract_one_plant(box)
                        
                        break

                    # if outside undistorted zone but in working zone
                    else:
                        msg = "Plant is in working zone, trying to get closer"
                        logger_full.write(msg + "\n")
                        if DEBUG:
                            print(msg)

                        # calculate values for move camera closer to a plant
                        control_point = ExtractionManager.get_closest_control_point(box_x, box_y, config.IMAGE_CONTROL_POINTS_MAP)

                        # fixing cork tube view obscuring
                        if config.AVOID_CORK_VIEW_OBSCURING:
                            # compute target point x
                            C_H = box_x - control_point[0]  # may be negative
                            H_x = control_point[0] + C_H
                            target_x = H_x

                            # compute target point y
                            T1_y = control_point[1] - config.UNDISTORTED_ZONE_RADIUS
                            T1_P = box_y - T1_y  # always positive
                            target_y = control_point[1] + T1_P - config.DISTANCE_FROM_UNDIST_BORDER

                            # transfer that to millimeters
                            sm_x = ExtractionManager.px_to_smoothie_value(target_x, control_point[0], config.ONE_MM_IN_PX)
                            sm_y = -ExtractionManager.px_to_smoothie_value(target_y, control_point[1], config.ONE_MM_IN_PX)

                            # move camera closer to a plant (and trying to avoid obscuring)
                            res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                msg = "Couldn't apply cork obscuring, smoothie's response:\n" + res + "\n" + \
                                    "(box_x: " + str(box_x) + " box_y: " + str(box_y) + " target_x: " + str(target_x) + \
                                    " target_y: " + str(target_y) + " cp_x: " + str(control_point[0]) + " cp_y: " + \
                                    str(control_point[1]) + ")"
                                logger_full.write(msg + "\n")

                                sm_x, sm_y = control_point[2], control_point[3]

                                # move camera closer to a plant
                                res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                                smoothie.wait_for_all_actions_done()
                                if res != smoothie.RESPONSE_OK:
                                    msg = "Couldn't move camera closer to plant, smoothie error occurred:\n" + res
                                    logger_full.write(msg + "\n")
                                    break
                        else:
                            sm_x, sm_y = control_point[2], control_point[3]

                            # move camera closer to a plant
                            res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                msg = "Couldn't move camera closer to plant, smoothie error occurred:\n" + res
                                logger_full.write(msg + "\n")
                                break

                        # make new photo and re-detect plants
                        time.sleep(config.DELAY_BEFORE_2ND_SCAN)
                        frame = camera.get_image()
                        temp_plant_boxes = detector.detect(frame)

                        # debug image saving
                        if config.SAVE_DEBUG_IMAGES:
                            image_saver.save_image(frame, img_output_dir, label="(extraction specify)",
                                                plants_boxes=temp_plant_boxes)

                        # check case if no plants detected
                        if len(temp_plant_boxes) == 0:
                            need_to_break = True  # breaks outer current plant extractions loop, forcing to start extraction of the next plant in the visit list

                            msg = "No plants detected (plant was in working zone before), trying to do delta movement and find this plant"
                            logger_full.write(msg + "\n")
                            if DEBUG:
                                print(msg)

                            # try to move for delta values and find the weed (down, up and left, right)
                            # TODO: make some kind of pool with results of prev. movement, and compute next step if prev wasn't successful (now breaks if movement was failed)
                            for sm_x, sm_y in [[0, -config.SEEK_DELTA_DISTANCE],
                                            [-config.SEEK_DELTA_DISTANCE, config.SEEK_DELTA_DISTANCE],
                                            [config.SEEK_DELTA_DISTANCE * 2, 0]]:
                                msg = "Trying to move for delta X" + str(sm_x) + " Y" + str(sm_y)
                                logger_full.write(msg + "\n")

                                res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                                smoothie.wait_for_all_actions_done()
                                if res != smoothie.RESPONSE_OK:
                                    msg = "Couldn't move for delta X" + str(sm_x) + " Y" + str(sm_y) + ", smoothie response:\n" + res
                                    logger_full.write(msg + "\n")
                                    break

                                # make new photo and re-detect plants
                                time.sleep(config.DELAY_BEFORE_2ND_SCAN)
                                frame = camera.get_image()
                                temp_plant_boxes = detector.detect(frame)

                                # debug image saving
                                if config.SAVE_DEBUG_IMAGES:
                                    image_saver.save_image(frame, img_output_dir,
                                                        label="(delta movement X" + str(sm_x) + " Y" + str(sm_y) + ")",
                                                        plants_boxes=temp_plant_boxes)

                                if len(temp_plant_boxes) > 0:
                                    msg = "Found " + str(len(temp_plant_boxes)) + " plants after delta movement, breaking delta movement and searching"
                                    logger_full.write(msg + "\n")
                                    need_to_break = False
                                    break
                                else:
                                    msg = "No plants found during current delta scan"
                                    logger_full.write(msg + "\n")

                            if need_to_break:
                                break

                        # get closest box (update current box from main list coordinates after moving closer)
                        box = ExtractionManager.min_plant_box_dist(temp_plant_boxes, config.SCENE_CENTER_X, config.SCENE_CENTER_Y)
                        plant_position_is_precise = True
                else:
                    msg = "Too much extraction attempts, trying to extract next plant if there is."
                    logger_full.write(msg)
            # if not in working zone
            else:
                msg = "Skipped " + str(box) + " (not in working area)"
                logger_full.write(msg + "\n")

        # set camera back to the Y min
        smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2 / config.XY_COEFFICIENT_TO_MM, Y=config.Y_MIN)
        smoothie.wait_for_all_actions_done()

    def extract_one_plant(self, box: detection.DetectedPlantBox, pattern=None):
        """Extract plant in current position of extraction head"""
        
        if pattern is not None:
            if hasattr(extraction.ExtractionMethods, pattern):
                msg = f"Trying extractions method \"{pattern}\"."
                self.logger_full.write(msg + "\n")
                if DEBUG:
                    print(msg)

                res, cork_is_stuck = getattr(extraction.ExtractionMethods, pattern)(self.smoothie, box, self.extraction_map)

            else:
                msg = f"Extractions method \"{pattern}\" not exist !."
                self.logger_full.write(msg + "\n")
                print(msg)
                return False

        elif hasattr(extraction.ExtractionMethods, box.get_name()):
            # TODO: it's temporary log (1)
            msg = f"Trying extractions method \"{box.get_name()}\"."  # only Daisy implemented, it has 5 drops
            self.logger_full.write(msg + "\n")
            if DEBUG:
                print(msg)

            res, cork_is_stuck = getattr(extraction.ExtractionMethods, box.get_name())(self.smoothie, box, self.extraction_map)

        else:
            # TODO: it's temporary log (2)
            # 5 drops is default, also 1 center drop is possible
            drops = 5 if config.EXTRACTION_DEFAULT_METHOD == "five_drops_near_center" else 1
            msg = f"Trying extractions method \"{config.EXTRACTION_DEFAULT_METHOD}\"."
            self.logger_full.write(msg + "\n")
            if DEBUG:
                print(msg)

            res, cork_is_stuck = getattr(extraction.ExtractionMethods, config.EXTRACTION_DEFAULT_METHOD)(self.smoothie, box, self.extraction_map)

        if res != self.smoothie.RESPONSE_OK:
            self.logger_full.write(res + "\n")
            if cork_is_stuck:  # danger flag is True if smoothie couldn't pick up cork
                msg = "Cork is stuck! Emergency stopping."
                self.logger_full.write(msg + "\n")
                exit(1)
        else:
            self.data_collector.add_extractions_data(box.get_name(), 1)
            self.data_collector.save_extractions_data(self.log_cur_dir + config.STATISTICS_OUTPUT_FILE)
            if DEBUG:
                ExtractionManager.save_matrix("last_extraction_map.txt",self.extraction_map, header=True) 

        return True

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
            if ExtractionManager.is_point_in_poly(box_x, box_y, zone_polygon):
                return True
        return False

    @staticmethod
    def is_point_in_circle(point_x, point_y, circle_center_x, circle_center_y, circle_radius):
        """Returns True if (x,y) point in the circle or on it's border, False otherwise"""

        return math.sqrt((point_x - circle_center_x) ** 2 + (point_y - circle_center_y) ** 2) <= circle_radius

    @staticmethod
    def get_closest_control_point(plant_px_x, plant_px_y, points_map):
        """Returns image control point, which is closest to the given plant center point"""

        index, min_distance = None, float("inf")
        for i in range(len(points_map)):
            cur_distance = math.sqrt((plant_px_x - points_map[i][0]) ** 2 + (plant_px_y - points_map[i][1]) ** 2)
            if cur_distance < min_distance:
                min_distance, index = cur_distance, i
        return points_map[index]

    @staticmethod
    def is_in_matrice(map: np.matrix, X=None, Y=None):
        isGood = True

        if Y is not None:
            if Y.__class__.__name__ in ('list', 'tuple'):
                for element in Y:
                    if element >= np.shape(map)[0] or element<0:
                        isGood = False
            else:
                if Y >= np.shape(map)[0] or Y<0:
                    isGood = False

        if X is not None:
            if X.__class__.__name__ in ('list', 'tuple'):
                for element in X:
                    if element >= np.shape(map)[1] or element<0:
                        isGood = False
            else:
                if X >= np.shape(map)[1] or X<0:
                    isGood = False

        return isGood

    @staticmethod
    def is_point_in_poly(point_x, point_y, polygon: Polygon):
        """Returns True if received polygon object contains received point, False otherwise"""

        return polygon.contains_point([point_x, point_y])

    @staticmethod
    def px_to_smoothie_value(target_px, center_px, one_mm_in_px):
        """Converts the distance given in pixels to the distance in millimeters"""

        # returns wrong sign for x because of different 0 position between smoothie and image
        return (target_px - center_px) / one_mm_in_px

    @staticmethod
    def min_plant_box_dist(boxes: list, current_px_x, current_px_y):
        """Returns plant box, which is closest to the given point coordinates"""

        return min(boxes, key=lambda box: box.get_distance_from(current_px_x, current_px_y))

    @staticmethod
    def save_matrix(file: str, matrix: np.ndarray, fmt = "{0:01}", header=False):
        str_matrix = ""
        for idx, obj in np.ndenumerate(matrix):
            if idx[0] != 0 and idx[1] == 0:
                str_matrix = str_matrix[:-1] + "\n"
            if idx[1] >= config.OFFSET_FOR_MATRIX_BORDER_IN_CELL and idx[1] < np.shape(matrix)[1]-config.OFFSET_FOR_MATRIX_BORDER_IN_CELL \
              and idx[0] >= config.OFFSET_FOR_MATRIX_BORDER_IN_CELL and idx[0] < np.shape(matrix)[0]-config.OFFSET_FOR_MATRIX_BORDER_IN_CELL:
                str_matrix += fmt.format(obj.getValue()) + " "
        str_matrix = str_matrix[:-1] + "\n"
        if (str_matrix[0] == "\n" and not header):
            str_matrix = str_matrix[1:]
        with open(file, "w+") as text_file:
            methods = [method for method in dir(extraction.ExtractionMethods) if (method.startswith('_') is False and method != "single_center_drop")]
            if header:
                text_file.write(f"# Method list : "+"(), ".join(methods)+".\n")
                text_file.flush()
                text_file.write(f"# If you see {config.MATRIX_EXTRACTION_PATTERN} in matrice its {methods[0]}() pattern for exemple.\n")
                text_file.flush()
            text_file.write(str_matrix)
            text_file.flush()

class MapCell:

    def __init__(self, x: int, y: int):
        self.value = 0
        self.x = x
        self.y = y

    def getValue(self):
        return self.value
        
    def __str__(self):
        return "<" + f"{self.__class__.__name__}[{self.x},{self.y}]={self.value}" + ">"

    def __repr__(self):
        return self.__str__()

class DetectionMapCell(MapCell):

    def __init__(self, x: int, y: int):
        super().__init__(x,y)
        self.numberOfLeaf = 0
        self.isRoot = False
        self.isLeaf = False
        self.isRegister = False
        self.type = None
        self.herParents = list()
        self.herChildren = list()
        self.indexInDetectedList = 0

    def setRoot(self, type: str, index: int):
        self.isLeaf = False
        self.isRoot = True
        self.indexInDetectedList = index
        self.type = type
        self.value = config.MATRIX_PLANT_ROOT

    def setLeaf(self, parent, type: str, index: int):
        if not self.isRoot:
            self.isLeaf = True
            self.type = type
            self.indexInDetectedList = index
            self.herParents.append(parent)
            parent.herChildren.append(self)
            self.numberOfLeaf = len(self.herParents)
            self.value = config.MATRIX_PLANT_LEAF + (self.numberOfLeaf-1)

class ExtractionMapCell(MapCell):
    
    def __init__(self, x: int, y: int):
        super().__init__(x,y)
        self.hasDrop = False
        self.isRootExtraction = False
        self.parent = None
        self.lastPattern = None
        self.dictPattern = dict((method, False) for method in dir(extraction.ExtractionMethods) if method.startswith('_') is False)

    def setRootExtraction(self):
        self.hasDrop = True
        self.isRootExtraction = True
        self.value = config.MATRIX_EXTRACTION

    def setPatternExtraction(self, pattern: str, parent = None):
        self.hasDrop = True
        self.dictPattern[pattern] = True
        self.lastPattern = pattern

        if parent is not None:
            self.parent = parent
        if not self.isRootExtraction:
            self.value = config.MATRIX_EXTRACTION_PATTERN + list(self.dictPattern.keys()).index(pattern)
        
