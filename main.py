"""Spiral movement, detection and extraction over given by ABCD points area"""
import threading
import os
import sys
from turtle import speed
import adapters
import navigation
from config import config
import time
import utility
import traceback
import detection
from matplotlib.patches import Polygon
import math
import cv2 as cv
import numpy as np
import stubs
import extraction
import datacollection
import pickle
from notification import NotificationClient
from notification import SyntheseRobot
import posix_ipc
import json
from extraction import ExtractionManagerV3
import glob
import importlib

"""
import SensorProcessing
import socketForRTK
from socketForRTK.Client import Client
"""
"""
if config.RECEIVE_FIELD_FROM_RTK:
    # import robotEN_JET as rtk
    import robotEN_JETSON as rtk
"""
# TODO: temp debug counter
IMAGES_COUNTER = 0


def load_coordinates(file_path):
    positions_list = []
    with open(file_path) as file:
        for line in file:
            if line != "":
                positions_list.append(list(map(float, line.split(" "))))
    return positions_list


def save_gps_coordinates(points: list, file_name: str):
    """
    Saves given list of points using QGIS format
    :param points:
    :param file_name:
    :return:
    """

    with open(file_name, "w") as file:
        for point in points:            
            if isinstance(point[0],list):
                str_point = str(point[0][0]) + " " + str(point[0][1]) + " " + str(point[1]) + "\n"
            else:
                str_point = str(point[0]) + " " + str(point[1]) + "\n"
            file.write(str_point)


def save_gps_coordinates_raw(points: list, file_name: str):
    """
    Saves given list of points as raw text
    :param points:
    :param file_name:
    :return:
    """

    with open(file_name, "w") as file:
        for point in points:
            file.write(str(point) + "\n")


def ask_for_ab_points(gps: adapters.GPSUbloxAdapter):
    """Ask user for moving vector AB points"""

    input("Press enter to save point B")
    point_b = gps.get_fresh_position()
    print("Point B saved.")
    input("Press enter to save point A")
    point_a = gps.get_fresh_position()
    print("Point A saved.")
    return [point_a, point_b]

def save_image(path_to_save, image, counter, session_label, date, sep="_"):
    """
    Assembles image file name and saves received image under this name to specified directory.
    Counter and session label may be passed if was set to None.
    """

    date = sep + date if date else ""
    session_label = sep + session_label if session_label else ""
    counter = sep + str(counter) if counter or counter == 0 else ""
    cv.imwrite(path_to_save + date + session_label + counter + ".jpg", image)


def debug_save_image(img_output_dir, label, frame, plants_boxes, undistorted_zone_radius, poly_zone_points_cv):
    # TODO: temp counter debug
    global IMAGES_COUNTER
    IMAGES_COUNTER += 1

    # TODO: data gathering temporary hardcoded
    if config.ALLOW_GATHERING:
        save_image(config.DATA_GATHERING_DIR, frame, IMAGES_COUNTER, label, utility.get_current_time())

    # debug image saving
    if config.SAVE_DEBUG_IMAGES:
        # draw time on frame
        cur_time = utility.get_current_time()
        left, top = 30, 30
        label_size, base_line = cv.getTextSize(cur_time + " No: " + str(IMAGES_COUNTER), cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, label_size[1])
        frame = cv.rectangle(frame, (left, top - round(1.5 * label_size[1])),
                             (left + round(1.5 * label_size[0]), top + base_line),
                             (0, 0, 255), cv.FILLED)
        frame = cv.putText(frame, cur_time + " No: " + str(IMAGES_COUNTER), (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.75,
                           (0, 0, 0), 2)

        # draw data on frame
        frame = utility.ImageSaver.draw_zone_circle(frame, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, undistorted_zone_radius)
        frame = utility.ImageSaver.draw_zone_poly(frame, poly_zone_points_cv)
        frame = detection.draw_boxes(frame, plants_boxes)
        save_image(img_output_dir, frame, IMAGES_COUNTER, label, cur_time)


def move_to_point_and_extract(coords_from_to: list,
                              gps: adapters.GPSUbloxAdapter,
                              vesc_engine: adapters.VescAdapterV3,
                              smoothie: adapters.SmoothieAdapter,
                              camera: adapters.CameraAdapterIMX219_170,
                              periphery_det: detection.YoloOpenCVDetection,
                              precise_det: detection.YoloOpenCVDetection,
                              logger_full: utility.Logger,
                              logger_table: utility.Logger,
                              report_field_names,
                              trajectory_saver: utility.TrajectorySaver,
                              working_zone_polygon,
                              img_output_dir,
                              nav: navigation.GPSComputing,
                              data_collector: datacollection.DataCollector,
                              log_cur_dir,
                              image_saver: utility.ImageSaver,
                              notification: NotificationClient,
                              extraction_manager_v3: ExtractionManagerV3,
                              ui_msg_queue: posix_ipc.MessageQueue, 
                              SI_speed: float, 
                              wheels_straight: bool, 
                              navigation_prediction: navigation.NavigationPrediction,
                              future_points: list,
                              allow_extractions: bool):
    """
    Moves to the given target point and extracts all weeds on the way.
    :param coords_from_to:
    :param gps:
    :param vesc_engine:
    :param smoothie:
    :param camera:
    :param periphery_det:
    :param precise_det:
    :param logger_full:
    :param logger_table:
    :param report_field_names:
    :param trajectory_saver:
    :param working_zone_polygon:
    :param img_output_dir:
    :param nav:
    :param data_collector:
    :param log_cur_dir:
    :param image_saver:
    :param notification:
    :param extraction_manager_v3:
    :return:
    """

    extract = SI_speed > 0 and allow_extractions

    vesc_speed = SI_speed * config.MULTIPLIER_SI_SPEED_TO_RPM
    speed_fast = config.SI_SPEED_FAST * config.MULTIPLIER_SI_SPEED_TO_RPM
    vesc_speed_fast = speed_fast if SI_speed >= 0 else -speed_fast
    navigation_prediction.set_SI_speed(SI_speed)

    raw_angles_history = []
    detections_period =[]
    navigations_period =[]
    stop_helping_point = nav.get_coordinate(coords_from_to[1], coords_from_to[0], 90, 1000)
    learn_go_straight_index = 0
    learn_go_straight_history = []
    
    last_skipped_point = coords_from_to[0]
    start_Nav_while =True
    last_correct_raw_angle = 0
    point_status ="origin"
    last_corridor_side = 0
    current_corridor_side = 1
    almost_start = 0

    prev_maneuver_time = time.time()
    working_mode_slow = 1
    working_mode_fast = 2
    working_mode_switching = 3
    current_working_mode = working_mode_slow
    last_working_mode = 0
    close_to_end = config.USE_SPEED_LIMIT  # True if robot is close to one of current movement vector points, False otherwise; False if speed limit near points is disabled

    lastNtripRestart = time.time()

    # set camera to the Y min
    res = smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                              Y_F=config.Y_F_MAX,
                                              X=smoothie.smoothie_to_mm((config.X_MAX - config.X_MIN) / 2, "X"),
                                              Y=smoothie.smoothie_to_mm(config.Y_MIN, "Y"))
    if res != smoothie.RESPONSE_OK:
        msg = "INIT: Failed to move camera to Y min, smoothie response:\n" + res
        logger_full.write(msg + "\n")
    smoothie.wait_for_all_actions_done()

    # TODO: maybe should add sleep time as camera currently has delay

    if config.AUDIT_MODE:
        vesc_engine.apply_rpm(vesc_speed, vesc_engine.PROPULSION_KEY)
        vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)

    try:
        notificationQueue = posix_ipc.MessageQueue(config.QUEUE_NAME_UI_NOTIFICATION)
    except:
        notificationQueue = None

    degraded_navigation_mode = False
    
    number_navigation_cycle_without_gps = 0

    point_reading_t = time.time()

    slow_mode_time = time.time()

    # main navigation control loop
    while True:
        # EXTRACTION CONTROL
        start_t = time.time()
        frame = camera.get_image()
        frame_t = time.time()

        per_det_start_t = time.time()
        if extract:
            plants_boxes = periphery_det.detect(frame)
        else:
            plants_boxes = list()
        per_det_end_t = time.time()
        detections_period.append(per_det_end_t - start_t)

        if config.SAVE_DEBUG_IMAGES:
            image_saver.save_image(
                frame,
                img_output_dir,
                label="PE_view_M=" + str(current_working_mode),
                plants_boxes=plants_boxes)
        if config.ALLOW_GATHERING and current_working_mode == working_mode_slow and \
                image_saver.get_counter("gathering") < config.DATA_GATHERING_MAX_IMAGES:
            image_saver.save_image(frame, config.DATA_GATHERING_DIR, plants_boxes=plants_boxes, counter_key="gathering")

        if extract:
            msg = "View frame time: " + str(frame_t - start_t) + "\t\tPeri. det. time: " + \
                  str(per_det_end_t - per_det_start_t)
        else:
            msg = "View frame time: " + str(frame_t - start_t) + "\t\tPeri. det. off time: " + \
                  str(per_det_end_t - per_det_start_t)
        logger_full.write(msg + "\n")

        # MOVEMENT AND ACTIONS MODES
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
                data_collector.add_detections_data(plant_label,
                                                   math.ceil((plants_count[plant_label]) / config.AUDIT_DIVIDER))

            # flush updates into the audit output file and log measured time
            if len(plants_boxes) > 0:
                data_collector.save_all_data(log_cur_dir + config.AUDIT_OUTPUT_FILE)

            dc_t = time.time() - dc_start_t
            msg = "Last scan weeds detected: " + str(len(plants_boxes)) + \
                  ", audit processing tick time: " + str(dc_t)
            logger_full.write(msg + "\n")
        else:
            # slow mode
            if current_working_mode == working_mode_slow:
                if last_working_mode != current_working_mode:
                    last_working_mode = current_working_mode
                    msg = "[Working mode] : slow"
                    if config.LOG_SPEED_MODES:
                        logger_full.write(msg + "\n")
                    if config.PRINT_SPEED_MODES:
                        print(msg)

                if ExtractionManagerV3.any_plant_in_zone(plants_boxes, working_zone_polygon):
                    vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
                    if config.VERBOSE_EXTRACT:
                        msg = "[VERBOSE EXTRACT] Stopping the robot because we have detected plant(s)."
                        logger_full.write_and_flush(msg+"\n")
                    # TODO remove thread init from here!
                    voltage_thread = threading.Thread(target=send_voltage_thread_tf, args=(vesc_engine, ui_msg_queue), daemon=True)
                    voltage_thread.start()
                    data_collector.add_vesc_moving_time_data(
                        vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))

                    # single precise center scan before calling for PDZ scanning and extractions
                    if config.ALLOW_PRECISE_SINGLE_SCAN_BEFORE_PDZ:
                        time.sleep(config.DELAY_BEFORE_2ND_SCAN)
                        frame = camera.get_image()
                        plants_boxes = precise_det.detect(frame)

                        # do PDZ scan and extract all plants if single precise scan got plants in working area
                        if ExtractionManagerV3.any_plant_in_zone(plants_boxes, working_zone_polygon):
                            if config.EXTRACTION_MODE == 1:
                                extraction_manager_v3.extract_all_plants()
                            elif config.EXTRACTION_MODE == 2:
                                extraction_manager_v3.mill_all_plants()
                            slow_mode_time = time.time()
                    else:
                        if config.EXTRACTION_MODE == 1:
                            extraction_manager_v3.extract_all_plants()
                        elif config.EXTRACTION_MODE == 2:
                            extraction_manager_v3.mill_all_plants()
                        slow_mode_time = time.time()

                    if config.VERBOSE_EXTRACT:
                        msg = "[VERBOSE EXTRACT] Extract cycle are finish."
                        logger_full.write_and_flush(msg+"\n")

                    msg = "Applying force step forward after extractions cycle(s)"
                    logger_full.write(msg + "\n")
                    if config.VERBOSE:
                        print(msg)
                    vesc_engine.set_time_to_move(config.STEP_FORWARD_TIME, vesc_engine.PROPULSION_KEY)
                    vesc_engine.set_rpm(config.SI_SPEED_STEP_FORWARD*config.MULTIPLIER_SI_SPEED_TO_RPM,
                                        vesc_engine.PROPULSION_KEY)
                    vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)
                    vesc_engine.wait_for_stop(vesc_engine.PROPULSION_KEY)

                elif config.SLOW_FAST_MODE and time.time() - slow_mode_time > config.SLOW_MODE_MIN_TIME:
                    # move cork to fast mode scan position
                    if config.VERBOSE:
                        msg = "SLOW MODE: moving cork to fast mode position\n"
                        logger_full.write(msg)

                    res = smoothie.custom_separate_xy_move_to(
                        X_F=config.X_F_MAX,
                        Y_F=config.Y_F_MAX,
                        X=smoothie.smoothie_to_mm((config.X_MAX - config.X_MIN) / 2, "X"),
                        Y=smoothie.smoothie_to_mm((config.Y_MAX - config.Y_MIN) * config.SLOW_FAST_MODE_HEAD_FACTOR,
                                                  "Y"))
                    if res != smoothie.RESPONSE_OK:
                        msg = "INIT: Keeping in slow mode as failed to move camera to fast mode scan position, smoothie's response:\n" + res
                        logger_full.write(msg + "\n")
                    else:
                        msg = "Switching from 'slow mode' to 'switching mode'"
                        if config.LOG_SPEED_MODES:
                            logger_full.write(msg + "\n")
                        if config.PRINT_SPEED_MODES:
                            print(msg)
                        current_working_mode = working_mode_switching

                # TODO a bug: will not start moving if config.SLOW_MODE_MIN_TIME == 0 or too low (switch speed applies right after slow mode weeds extractions)
                if not vesc_engine.is_moving(vesc_engine.PROPULSION_KEY):
                    vesc_engine.set_time_to_move(config.VESC_MOVING_TIME, vesc_engine.PROPULSION_KEY)
                    vesc_engine.apply_rpm(vesc_speed, vesc_engine.PROPULSION_KEY)
                    vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)

            # switching (from slow to fast) mode
            elif current_working_mode == working_mode_switching:
                if last_working_mode != current_working_mode:
                    last_working_mode = current_working_mode
                    msg = "[Working mode] : switching to fast"
                    if config.LOG_SPEED_MODES:
                        logger_full.write(msg + "\n")
                    if config.PRINT_SPEED_MODES:
                        print(msg)

                if ExtractionManagerV3.any_plant_in_zone(plants_boxes, working_zone_polygon):
                    vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
                    data_collector.add_vesc_moving_time_data(
                        vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))

                    if config.VERBOSE:
                        msg = "Moving cork to slow mode scan position\n"
                        logger_full.write(msg)

                    # smoothie.wait_for_all_actions_done()
                    res = smoothie.custom_separate_xy_move_to(
                        X_F=config.X_F_MAX,
                        Y_F=config.Y_F_MAX,
                        X=smoothie.smoothie_to_mm((config.X_MAX - config.X_MIN) / 2, "X"),
                        Y=smoothie.smoothie_to_mm(config.Y_MIN, "Y"))
                    if res != smoothie.RESPONSE_OK:
                        msg = "INIT: Failed to move camera to Y min, smoothie response:\n" + res
                        logger_full.write(msg + "\n")
                    smoothie.wait_for_all_actions_done()

                    current_working_mode = working_mode_slow
                    slow_mode_time = time.time()
                    vesc_engine.set_rpm(vesc_speed, vesc_engine.PROPULSION_KEY)
                    continue

                sm_cur_pos = smoothie.get_smoothie_current_coordinates(convert_to_mms=False)
                if abs(sm_cur_pos["X"] - (config.X_MAX - config.X_MIN) / 2) < 0.001 and \
                        abs(sm_cur_pos["Y"] - (config.Y_MAX - config.Y_MIN) * config.SLOW_FAST_MODE_HEAD_FACTOR) < 0.001:
                    msg = "Switching from 'switching mode' to 'fast mode'"
                    if config.LOG_SPEED_MODES:
                        logger_full.write(msg + "\n")
                    if config.PRINT_SPEED_MODES:
                        print(msg)
                    current_working_mode = working_mode_fast

            # fast mode
            elif current_working_mode == working_mode_fast:
                if last_working_mode != current_working_mode:
                    last_working_mode = current_working_mode
                    msg = "[Working mode] : fast"
                    if config.LOG_SPEED_MODES:
                        logger_full.write_and_flush(msg + "\n")
                    if config.PRINT_SPEED_MODES:
                        print(msg)

                if ExtractionManagerV3.any_plant_in_zone(plants_boxes, working_zone_polygon):
                    vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
                    data_collector.add_vesc_moving_time_data(
                        vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))

                    if config.VERBOSE:
                        msg = "Moving cork to slow mode scan position\n"
                        logger_full.write(msg)

                    # smoothie.wait_for_all_actions_done()
                    res = smoothie.custom_separate_xy_move_to(
                        X_F=config.X_F_MAX,
                        Y_F=config.Y_F_MAX,
                        X=smoothie.smoothie_to_mm((config.X_MAX - config.X_MIN) / 2, "X"),
                        Y=smoothie.smoothie_to_mm(config.Y_MIN, "Y"))
                    if res != smoothie.RESPONSE_OK:
                        msg = "INIT: Failed to move camera to Y min, smoothie response:\n" + res
                        logger_full.write(msg + "\n")
                    smoothie.wait_for_all_actions_done()

                    msg = "Switching from 'fast mode' to 'slow mode'"
                    if config.LOG_SPEED_MODES:
                        logger_full.write(msg + "\n")
                    if config.PRINT_SPEED_MODES:
                        print(msg)
                    current_working_mode = working_mode_slow
                    slow_mode_time = time.time()
                    # TODO dont need anymore? as rpm is set at the end of slow mode
                    # vesc_engine.set_rpm(vesc_speed, vesc_engine.PROPULSION_KEY)
                    continue
                elif close_to_end:
                    # TODO: won't execute (speed not applied bug) in case if rpm already was set by set_rpm() method (1)
                    if vesc_engine.get_adapter_rpm(vesc_engine.PROPULSION_KEY) != vesc_speed:
                        msg = f"Applying slow speed {vesc_speed} at 'fast mode' " \
                              f"(was {vesc_engine.get_adapter_rpm(vesc_engine.PROPULSION_KEY)}) " \
                              f"because of close_to_end flag trigger"
                        if config.LOG_SPEED_MODES:
                            logger_full.write(msg + "\n")
                        if config.PRINT_SPEED_MODES:
                            print(msg)
                        vesc_engine.apply_rpm(vesc_speed, vesc_engine.PROPULSION_KEY)
                else:
                    # TODO: won't execute (speed not applied bug) in case if rpm already was set by set_rpm() method (2)
                    if vesc_engine.get_adapter_rpm(vesc_engine.PROPULSION_KEY) != vesc_speed_fast:
                        msg = f"Applying fast speed {vesc_speed_fast} at 'fast mode' " \
                              f"(was {vesc_engine.get_adapter_rpm(vesc_engine.PROPULSION_KEY)})"
                        if config.LOG_SPEED_MODES:
                            logger_full.write(msg + "\n")
                        if config.PRINT_SPEED_MODES:
                            print(msg)
                        vesc_engine.apply_rpm(vesc_speed_fast, vesc_engine.PROPULSION_KEY)

        # NAVIGATION CONTROL
        cur_pos = gps.get_last_position()

        nav_start_t = time.time()

        if start_Nav_while:
            navigation_period =1
        else:
            navigation_period = nav_start_t - prev_maneuver_time 
        
        navigations_period.append(navigation_period)  
        prev_maneuver_time = nav_start_t #time reference to decide the number of detection before resuming gps.get
        #print("tock")

        if start_Nav_while:
            prev_pos = cur_pos
            start_Nav_while=False

        # mu_navigations_period, sigma_navigations_period = utility.mu_sigma(navigations_period)

        navigation_prediction.set_current_lat_long(cur_pos)

        if str(cur_pos) == str(prev_pos):
            if time.time() - point_reading_t > config.GPS_POINT_WAIT_TIME_MAX:
                vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
                data_collector.add_vesc_moving_time_data(
                    vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))
                while True:
                    cur_pos = gps.get_last_position()
                    if str(cur_pos) != str(prev_pos):
                        vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)
                        break
            else:
                continue

        if config.GPS_QUALITY_IGNORE and cur_pos[2] != "4":
            vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
            logger_full.write_and_flush("Stopping the robot for lack of quality gps 4, waiting for it...\n")
            data_collector.add_vesc_moving_time_data(
                vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))
            navigation.NavigationV3.check_reboot_Ntrip("1", 0, logger_full)
            while True:
                cur_pos = gps.get_last_position()
                if cur_pos[2] == "4":
                    vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)
                    logger_full.write_and_flush("The gps has regained quality 4, relaunch of the robot.\n")
                    break

        point_reading_t = time.time()

        trajectory_saver.save_point(cur_pos)
        if ui_msg_queue is not None:
            ui_msg_queue.send(json.dumps({"last_gps": cur_pos}))

        if config.CONTINUOUS_INFORMATION_SENDING and not degraded_navigation_mode:
            notification.set_current_coordinate(cur_pos)

        distance = nav.get_distance(cur_pos, coords_from_to[1])
        
        last_corridor_side = current_corridor_side
        perpendicular, current_corridor_side = nav.get_deviation(coords_from_to[0],coords_from_to[1],cur_pos)

        # check if arrived
        _, side = nav.get_deviation(coords_from_to[1], stop_helping_point, cur_pos)
        # if distance <= config.COURSE_DESTINATION_DIFF:  # old way
        if side != 1:  # TODO: maybe should use both side and distance checking methods at once
            vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
            data_collector.add_vesc_moving_time_data(
                vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))
            # msg = "Arrived (allowed destination distance difference " + str(config.COURSE_DESTINATION_DIFF) + " mm)"
            msg = "Arrived to " + str(coords_from_to[1])  # TODO: service will reload script even if it done his work?
            # print(msg)
            logger_full.write(msg + "\n")
            
            # put the wheel straight
            if wheels_straight:
                response = smoothie.custom_move_to(A_F=config.A_F_MAX, A=0)
                if response != smoothie.RESPONSE_OK:  # TODO: what if response is not ok?
                    msg = "Couldn't turn wheels to center (0), smoothie response:\n" + response
                    print(msg)
                    logger_full.write(msg + "\n")
                else:
                    # save wheels angle
                    with open(config.LAST_ANGLE_WHEELS_FILE, "w+") as wheels_angle_file:
                        wheels_angle_file.write(str(smoothie.get_adapter_current_coordinates()["A"]))
            break

        # TODO check for bug: arrival check applies single speed for all path (while multiple speeds are applied)
        # check if can arrived
        if vesc_engine.get_adapter_rpm(vesc_engine.PROPULSION_KEY) / config.MULTIPLIER_SI_SPEED_TO_RPM * \
                config.MANEUVERS_FREQUENCY > nav.get_distance(cur_pos, coords_from_to[1]):
            vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
            data_collector.add_vesc_moving_time_data(
                vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))
            msg = "Will have arrived before the next point to " + str(coords_from_to[1])
            # print(msg)
            logger_full.write(msg + "\n")
            
            break

        # reduce speed if near the target point
        if config.USE_SPEED_LIMIT:
            distance_from_start = nav.get_distance(coords_from_to[0], cur_pos)
            close_to_end = distance < config.DECREASE_SPEED_TRESHOLD or distance_from_start < config.DECREASE_SPEED_TRESHOLD

            msg = "Distance to B: " + str(distance)
            # print(msg)
            logger_full.write(msg + "\n")

            msg = "Prev: " + str(prev_pos) + " Cur: " + str(cur_pos) + " A: " + str(coords_from_to[0]) \
                + " B: " + str(coords_from_to[1])
            # print(msg)
            logger_full.write(msg + "\n")

            # pass by cur points which are very close to prev point to prevent angle errors when robot is staying
            # (too close points in the same position can produce false huge angles)

        navigation_prediction.run_prediction(coords_from_to, cur_pos)

        # raw_angle_cruise = nav.get_angle(coords_from_to[0], cur_pos, cur_pos, coords_from_to[1])
        #raw_angle_legacy = nav.get_angle(prev_pos, cur_pos, cur_pos, coords_from_to[1])
        raw_angle_centroid = nav.get_angle(prev_pos, cur_pos, coords_from_to[0], coords_from_to[1])
        raw_angle_cruise = - current_corridor_side * math.log(1+perpendicular)

        if nav.get_distance(coords_from_to[0],coords_from_to[1]) < config.CORNER_THRESHOLD and nav.get_distance(coords_from_to[1],future_points[0][0]) < config.CORNER_THRESHOLD:
        #if abs(raw_angle_legacy)>config.LOST_THRESHOLD:
            centroid_factor = config.CENTROID_FACTOR_LOST
            cruise_factor = 1/centroid_factor
        else:
            centroid_factor = config.CENTROID_FACTOR_ORIENTED
            cruise_factor = 1

        raw_angle = raw_angle_centroid*centroid_factor + raw_angle_cruise*cruise_factor

        #raw_angle = butter_lowpass_filter(raw_angle, 0.5, 4, 6)

        if config.LEARN_GO_STRAIGHT:
            if config.MIN_PERPENDICULAR_GO_STRAIGHT >= perpendicular:
                learn_go_straight_index += 1
                learn_go_straight_history.append(raw_angle)
                if len(learn_go_straight_history) >= config.VALUES_LEARN_GO_STRAIGHT:
                    learn_go_straight = sum(learn_go_straight_history)/len(learn_go_straight_history)
                    msg = f"Average angle applied to the wheel for the robot to have found : {learn_go_straight}."
                    logger_full.write_and_flush(msg + "\n")
                    with open(config.LEARN_GO_STRAIGHT_FILE, "w+") as learn_go_straight_file:
                        learn_go_straight_file.write(str(learn_go_straight))
            else:
                learn_go_straight_index = 0

        # NAVIGATION STATE MACHINE
        if nav.get_distance(prev_pos, cur_pos) < config.PREV_CUR_POINT_MIN_DIST:
            raw_angle = last_correct_raw_angle
            #print("The distance covered is low")
            point_status = "skipped"
            
            # register the last position where the robot almost stop 
            # in order to disable the deviation servo for a config.POURSUIT_LIMIT length and then resume in cruise
            last_skipped_point = cur_pos
        else:
            last_correct_raw_angle = raw_angle
            point_status ="correct"

        almost_start = nav.get_distance(last_skipped_point, cur_pos)

        # sum(e)
        if len(raw_angles_history) >= config.WINDOW:
            raw_angles_history.pop(0)
        raw_angles_history.append(raw_angle)
        #print("len(raw_angles_history):",len(raw_angles_history))
        sum_angles = sum(raw_angles_history)
        if sum_angles > config.SUM_ANGLES_HISTORY_MAX:
            msg = "Sum angles " + str(sum_angles) + " is bigger than max allowed value " + \
                str(config.SUM_ANGLES_HISTORY_MAX) + ", setting to " + str(config.SUM_ANGLES_HISTORY_MAX)
            # print(msg)
            logger_full.write(msg + "\n")
            #Get Ready to go down as soon as the angle get negatif
            raw_angles_history[len(raw_angles_history)-1]-= sum_angles - config.SUM_ANGLES_HISTORY_MAX   
            sum_angles = config.SUM_ANGLES_HISTORY_MAX
        elif sum_angles < -config.SUM_ANGLES_HISTORY_MAX:
            msg = "Sum angles " + str(sum_angles) + " is less than min allowed value " + \
                str(-config.SUM_ANGLES_HISTORY_MAX) + ", setting to " + str(-config.SUM_ANGLES_HISTORY_MAX)
            # print(msg)
            logger_full.write(msg + "\n")
            #get Ready to go up as soon as the angle get positive:
            raw_angles_history[len(raw_angles_history)-1]+=  -sum_angles - config.SUM_ANGLES_HISTORY_MAX
            sum_angles = -config.SUM_ANGLES_HISTORY_MAX

        # KP = 0.2*0,55
        # KI = 0.0092*0,91

        KP = getSpeedDependentConfigParam(config.KP, SI_speed, "KP", logger_full)
        KI = getSpeedDependentConfigParam(config.KI, SI_speed, "KI", logger_full)

        angle_kp_ki = raw_angle * KP + sum_angles * KI 

        target_angle_sm = angle_kp_ki * -config.A_ONE_DEGREE_IN_SMOOTHIE  # smoothie -Value == left, Value == right
        #target_angle_sm = 0     #Debug COVID_PLACE
        ad_wheels_pos = smoothie.get_adapter_current_coordinates()["A"]
        # sm_wheels_pos = smoothie.get_smoothie_current_coordinates()["A"]
        sm_wheels_pos = "off"

        # compute order angle (smoothie can't turn for huge values immediately also as cancel movement,
        # so we need to do nav. actions in steps)
        order_angle_sm = target_angle_sm - ad_wheels_pos

        # check for out of update frequency and smoothie execution speed range (for nav wheels)
        if order_angle_sm > config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND * \
                config.A_ONE_DEGREE_IN_SMOOTHIE:
            msg = "Order angle changed from " + str(order_angle_sm) + " to " + str(
                config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND +
                config.A_ONE_DEGREE_IN_SMOOTHIE) + " due to exceeding degrees per tick allowed range."
            # print(msg)
            logger_full.write(msg + "\n")
            order_angle_sm = config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND * \
                            config.A_ONE_DEGREE_IN_SMOOTHIE
        elif order_angle_sm < -(config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND *
                                config.A_ONE_DEGREE_IN_SMOOTHIE):
            msg = "Order angle changed from " + str(order_angle_sm) + " to " + str(-(
                    config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND *
                    config.A_ONE_DEGREE_IN_SMOOTHIE)) + " due to exceeding degrees per tick allowed range."
            # print(msg)
            logger_full.write(msg + "\n")
            order_angle_sm = -(config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND *
                            config.A_ONE_DEGREE_IN_SMOOTHIE)

        # convert to global smoothie coordinates
        order_angle_sm += ad_wheels_pos

        # checking for out of smoothie supported range
        if order_angle_sm > config.A_MAX:
            msg = "Global order angle changed from " + str(order_angle_sm) + " to config.A_MAX = " + \
                    str(config.A_MAX) + " due to exceeding smoothie allowed values range."
            # print(msg)
            logger_full.write(msg + "\n")
            order_angle_sm = config.A_MAX
        elif order_angle_sm < config.A_MIN:
            msg = "Global order angle changed from " + str(order_angle_sm) + " to config.A_MIN = " + \
                    str(config.A_MIN) + " due to exceeding smoothie allowed values range."
            # print(msg)
            logger_full.write(msg + "\n")
            order_angle_sm = config.A_MIN

        if SI_speed>=0:
            response = smoothie.custom_move_to(A_F=config.A_F_MAX, A=order_angle_sm)
        else:
            response = smoothie.custom_move_to(A_F=config.A_F_MAX, A=-order_angle_sm)
        if response != smoothie.RESPONSE_OK:  # TODO: what if response is not ok?
            msg = "Smoothie response is not ok: " + response
            print(msg)
            logger_full.write(msg + "\n")
        else:
            # save wheels angle
            with open(config.LAST_ANGLE_WHEELS_FILE, "w+") as wheels_angle_file:
                wheels_angle_file.write(str(smoothie.get_adapter_current_coordinates()["A"]))

        raw_angle = round(raw_angle, 2)
        angle_kp_ki = round(angle_kp_ki, 2)
        order_angle_sm = round(order_angle_sm, 2)
        sum_angles = round(sum_angles, 2)
        distance = round(distance, 2)
        ad_wheels_pos = round(ad_wheels_pos, 2)
        perpendicular = round(perpendicular, 2)
        # sm_wheels_pos = round(sm_wheels_pos, 2)
        gps_quality = cur_pos[2]
        corridor = ""
        if current_corridor_side==-1:
            corridor = "left"
        elif current_corridor_side==1:
            corridor = "right"
        

        raw_angle_cruise = round(raw_angle_cruise, 2)

        lastNtripRestart = navigation.NavigationV3.check_reboot_Ntrip(gps_quality, lastNtripRestart, logger_full)

        msg = str(gps_quality).ljust(5) + str(raw_angle).ljust(8) + str(angle_kp_ki).ljust(8) + str(
            order_angle_sm).ljust(8) + str(sum_angles).ljust(8) + str(distance).ljust(13) + str(ad_wheels_pos).ljust(
            8) + str(sm_wheels_pos).ljust(9) + point_status.ljust(12)+str(perpendicular).ljust(10)+corridor.ljust(9)+str(centroid_factor).ljust(16)+str(
            cruise_factor).ljust(14)
        print(msg)
        logger_full.write(msg + "\n")

        # TODO possible bug: reading sensors data from vesc 4 times per second
        # load sensors data to csv
        s = ","
        msg = str(gps_quality) + s + str(raw_angle) + s + str(angle_kp_ki) + s + str(order_angle_sm) + s + \
            str(sum_angles) + s + str(distance) + s + str(ad_wheels_pos) + s + str(sm_wheels_pos)
        vesc_data = vesc_engine.get_sensors_data(report_field_names, vesc_engine.PROPULSION_KEY)
        if vesc_data is not None:
            msg += s
            for key in vesc_data:
                msg += str(vesc_data[key]) + s
                if config.CONTINUOUS_INFORMATION_SENDING and key == "input_voltage":
                    notification.set_input_voltage(vesc_data[key])
            msg = msg[:-1]
        logger_table.write(msg + "\n")

        prev_pos = cur_pos

        msg = "Nav calc time: " + str(time.time() - nav_start_t)
        logger_full.write(msg + "\n\n")


def send_voltage_thread_tf(vesc_engine: adapters.VescAdapterV3, ui_msg_queue):
    vesc_data = None
    while vesc_data is None:
        vesc_data = vesc_engine.get_sensors_data(['input_voltage'], vesc_engine.PROPULSION_KEY)
        if vesc_data is not None:
                input_voltage = vesc_data["input_voltage"]
                if ui_msg_queue is not None:
                    ui_msg_queue.send(json.dumps({"input_voltage": input_voltage}))
        else:
            time.sleep(1)


def getSpeedDependentConfigParam(configParam: dict, SI_speed: float, paramName: str, logger_full: utility.Logger):
    if SI_speed in configParam:
        return configParam[SI_speed]
    else:
        msg = f"Speed SI {SI_speed} not present in {paramName}."
        if config.VERBOSE:
            print(msg)
        logger_full.write(msg + "\n")
        exit(1)


def getAuditDependentConfigParam(configParam: dict, paramName: str, logger_full: utility.Logger):
    if config.AUDIT_MODE in configParam:
        return configParam[config.AUDIT_MODE]
    else:
        msg = f"Audit mode {config.AUDIT_MODE} isn't present in {paramName}."
        if config.VERBOSE:
            print(msg)
        logger_full.write(msg + "\n")
        exit(1)


def compute_x1_x2_points(point_a: list, point_b: list, nav: navigation.GPSComputing, logger: utility.Logger):
    """
    Computes p. x1 with config distance from p. A and p. x2 with the same distance from p. B. Distance is loaded from
     config file. Returns None if AB <= that distance (as there's no place for robot maneuvers).

    :param point_a:
    :param point_b:
    :param nav:
    :param logger:
    :return:
    """

    cur_vec_dist = nav.get_distance(point_a, point_b)

    maneuverStartDistance = getAuditDependentConfigParam(config.MANEUVER_START_DISTANCE,"MANEUVER_START_DISTANCE",logger)

    # check if moving vector is too small for maneuvers
    if maneuverStartDistance * 2 >= cur_vec_dist:
        msg = "No place for maneuvers; config start maneuver distance is (that will be multiplied by 2): " + \
              str(maneuverStartDistance) + " current moving vector distance is: " + str(cur_vec_dist) + \
              " Given points are: " + str(point_a) + " " + str(point_b)
        # print(msg)
        logger.write(msg + "\n")
        return None, None

    point_x1 = nav.get_point_on_vector(point_a, point_b, maneuverStartDistance)
    point_x2 = nav.get_point_on_vector(point_a, point_b, cur_vec_dist - maneuverStartDistance)
    return point_x1, point_x2


def compute_x2_spiral(point_a: list, point_b: list, nav: navigation.GPSComputing, logger: utility.Logger):
    """
    Computes p. x2 with distance + spiral interval distance from p. B. Distances are loaded from
     config file. Returns None if AB <= distance * 2 + interval (as there's no place for robot maneuvers).
    :param point_a:
    :param point_b:
    :param nav:
    :param logger:
    :return:
    """

    cur_vec_dist = nav.get_distance(point_a, point_b)
    
    maneuverStartDistance = getAuditDependentConfigParam(config.MANEUVER_START_DISTANCE,"MANEUVER_START_DISTANCE",logger)

    spiralSidesInterval = getAuditDependentConfigParam(config.SPIRAL_SIDES_INTERVAL,"SPIRAL_SIDES_INTERVAL",logger)

    # check if moving vector is too small for maneuvers
    if maneuverStartDistance * 2 + spiralSidesInterval >= cur_vec_dist:
        msg = "No place for maneuvers; Config maneuver distance is (that will be multiplied by 2): " + \
              str(maneuverStartDistance) + " Config spiral interval: " + str(spiralSidesInterval) + \
              " Current moving vector distance is: " + str(cur_vec_dist) + " Given points are: " + str(point_a) + \
              " " + str(point_b)
        # print(msg)
        logger.write(msg + "\n")
        return None
    return nav.get_point_on_vector(point_a, point_b, cur_vec_dist - maneuverStartDistance - spiralSidesInterval)


def compute_x1_x2_int_points(point_a: list, point_b: list, nav: navigation.GPSComputing, logger: utility.Logger):
    """
    Computes spiral interval points x1, x2
    :param point_a:
    :param point_b:
    :param nav:
    :param logger:
    :return:
    """

    cur_vec_dist = nav.get_distance(point_a, point_b)

    spiralSidesInterval = getAuditDependentConfigParam(config.SPIRAL_SIDES_INTERVAL,"SPIRAL_SIDES_INTERVAL",logger)

    # check if moving vector is too small for maneuvers
    if spiralSidesInterval * 2 >= cur_vec_dist:
        msg = "No place for maneuvers; Config spiral interval (that will be multiplied by 2): " + \
              str(spiralSidesInterval) + " Current moving vector distance is: " + str(cur_vec_dist) + \
              " Given points are: " + str(point_a) + " " + str(point_b)
        if config.VERBOSE:
            print(msg)
        logger.write(msg + "\n")
        return None, None

    point_x1_int = nav.get_point_on_vector(point_a, point_b, spiralSidesInterval)
    point_x2_int = nav.get_point_on_vector(point_a, point_b, cur_vec_dist - spiralSidesInterval)
    return point_x1_int, point_x2_int


def add_points_to_path(path: list, *args):
    for point in args:
        if point is None:
            return False
        path.append(point)
    return True


def check_points_for_nones(*args):
    for point in args:
        if point is None:
            return False
    return True

def compute_bezier_points(point_0, point_1, point_2):
    t = np.linspace(0,1,config.NUMBER_OF_BEZIER_POINT)
    coords = list()
    for i in t :
        x = (point_0[0]-2*point_1[0]+point_2[0])*(i**2) + (2*point_1[0]-2*point_0[0])*i + point_0[0]
        y = (point_0[1]-2*point_1[1]+point_2[1])*(i**2) + (2*point_1[1]-2*point_0[1])*i + point_0[1]
        coords.append([x,y])
    return coords
    
def build_bezier_path(abcd_points: list, nav: navigation.GPSComputing, logger: utility.Logger):
    path = []
    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    # get moving points A1 - ... - D2 spiral
    a1, a2 = compute_x1_x2_points(a, b, nav, logger)
    b1, b2 = compute_x1_x2_points(b, c, nav, logger)
    c1, c2 = compute_x1_x2_points(c, d, nav, logger)
    d1, d2 = compute_x1_x2_points(d, a, nav, logger)

    first_turn = compute_bezier_points(a2,b,b1)
    second_turn = compute_bezier_points(b2,c,c1)
    third_turn = compute_bezier_points(c2,d,d1)
    fourth_turn = compute_bezier_points(d2,a,a1)

    for point in first_turn:
        # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
        if not add_points_to_path(path, point):
            return path
    
    for point in second_turn:
        # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
        if not add_points_to_path(path, point):
            return path

    for point in third_turn:
        # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
        if not add_points_to_path(path, point):
            return path
    
    for point in fourth_turn:
        # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
        if not add_points_to_path(path, point):
            return path

    # get A'B'C'D' (prepare next ABCD points)
    b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
    d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

    if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
        return path

    a_new, b_new = compute_x1_x2_int_points(d2_int, b1_int, nav, logger)
    c_new, d_new = compute_x1_x2_int_points(b2_int, d1_int, nav, logger)

    if not check_points_for_nones(a_new, b_new, c_new, d_new):
        return path

    a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int

    # keep reducing sides for spiral
    while True:
        # get A'B'C'D' (prepare next ABCD points)
        b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
        d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

        if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
            return path

        a_new, b_new = compute_x1_x2_int_points(d2_int, b1_int, nav, logger)
        c_new, d_new = compute_x1_x2_int_points(b2_int, d1_int, nav, logger)

        if not check_points_for_nones(a_new, b_new, c_new, d_new):
            return path

        # get moving points A1 - ... - D2 spiral
        a1, a2 = compute_x1_x2_points(d2_int_prev, b, nav, logger)
        b1, b2 = compute_x1_x2_points(b, c, nav, logger)
        c1, c2 = compute_x1_x2_points(c, d, nav, logger)
        d1, d2 = compute_x1_x2_points(d, a, nav, logger)

        for point in [a,b,c,d,a1,b1,c1,d1,a2,b2,c2,d2]:
            if point is None:
                return path

        first_turn = compute_bezier_points(d2,a,a1)
        second_turn = compute_bezier_points(a2,b,b1)
        third_turn = compute_bezier_points(b2,c,c1)
        fourth_turn = compute_bezier_points(c2,d,d1)

        for point in first_turn:
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, point):
                return path
        
        for point in second_turn:
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, point):
                return path

        for point in third_turn:
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, point):
                return path
        
        for point in fourth_turn:
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, point):
                return path

        a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int


def get_rectangle_isosceles_side(turning_radius):
    return (0.5*(turning_radius*((2**0.5)-1))**2)**0.5 #Bezier refer \Nextcloud\3. Engineering\navigation

def corner_finish_rounds(turning_radius : float):
    if config.VERBOSE:
        print("black corridor width at full steering %2.0f"%get_rectangle_isosceles_side(turning_radius)," millimeters")
    return int((get_rectangle_isosceles_side(turning_radius))/config.FIELD_REDUCE_SIZE)+1 #how many corner round due to robot working width

def add_forward_backward_path(abcd_points: list, nav: navigation.GPSComputing, logger: utility.Logger, SI_speed_fwd: float, SI_speed_rev: float, currently_path: list):
    
    if not config.ADD_FORWARD_BACKWARD_TO_END_PATH and not config.FORWARD_BACKWARD_PATH:
        return currently_path
    
    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    fwd = SI_speed_fwd
    rev = SI_speed_rev

    spiralSidesInterval = getAuditDependentConfigParam(config.SPIRAL_SIDES_INTERVAL,"SPIRAL_SIDES_INTERVAL",logger)

    while nav.get_distance(b,c)>spiralSidesInterval:

        if not add_points_to_path(currently_path, [a,rev]):
            return currently_path

        if not add_points_to_path(currently_path, [b,fwd]):
            return currently_path
        
        b = compute_x1_x2(b, c, spiralSidesInterval, nav)[0]
        a = compute_x1_x2(a, d, spiralSidesInterval, nav)[0]

    if not add_points_to_path(currently_path, [a,rev]):
            return currently_path

    if not add_points_to_path(currently_path, [b,fwd]):
        return currently_path
    
    return currently_path


def build_bezier_with_corner_path(abcd_points: list, nav: navigation.GPSComputing, logger: utility.Logger, SI_speed_fwd: float, SI_speed_rev: float):
    path = []
    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    fwd = SI_speed_fwd
    rev = SI_speed_rev

    spiralSidesInterval = getAuditDependentConfigParam(config.SPIRAL_SIDES_INTERVAL,"SPIRAL_SIDES_INTERVAL",logger)

    _break = False

    # get moving points A1 - ... - D2 spiral
    a1, a2 = compute_x1_x2_points(a, b, nav, logger)
    b1, b2 = compute_x1_x2_points(b, c, nav, logger)
    c1, c2 = compute_x1_x2_points(c, d, nav, logger)
    d1, d2 = compute_x1_x2_points(d, a, nav, logger)
    a1_spiral = nav.get_coordinate(a1, a, 90, spiralSidesInterval)
    _ , a_spiral = compute_x1_x2(d,a,spiralSidesInterval,nav)

    if not add_points_to_path(path, [a, fwd]):
        return path

    first_bezier_turn = compute_bezier_points(a2,b,b1)
    second_bezier_turn = compute_bezier_points(b2,c,c1)
    third_bezier_turn = compute_bezier_points(c2,d,d1)
    fourth_bezier_turn = compute_bezier_points(d2,a_spiral,a1_spiral)

    maneuverStartDistance = getAuditDependentConfigParam(config.MANEUVER_START_DISTANCE,"MANEUVER_START_DISTANCE",logger)

    turning_radius = maneuverStartDistance   # minimum turning radius given in millimeter
    
    if config.ADD_CORNER_TO_BEZIER_PATH:
        rnd = 0
        rnds = corner_finish_rounds(turning_radius)

        for rnd in range(rnds+1):             # example a 3meter radius requires 4 corners finish
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            mxt = "corner rnd "+str(rnd)+"/"+str(rnds)
                    
            #the direction is given along with the point in meter per second, signed
            #go to line forward, step back to the turning point "a1"

            if not add_points_to_path(path,[b,fwd,"B "+mxt]):
                return path                                            
            for index in range(0,len(first_bezier_turn)):
                if index == 0:
                    if not add_points_to_path(path, [first_bezier_turn[index],rev]):
                        return path
                else:
                    if not add_points_to_path(path, [first_bezier_turn[index],fwd]):
                        return path
            if not add_points_to_path(path,[b,rev,mxt]):
                return path 


            if not add_points_to_path(path,[c,fwd,"C "+mxt]):
                return path                                            
            for index in range(0,len(second_bezier_turn)):
                if index == 0:
                    if not add_points_to_path(path, [second_bezier_turn[index],rev]):
                        return path
                else:
                    if not add_points_to_path(path, [second_bezier_turn[index],fwd]):
                        return path
            if not add_points_to_path(path, [c,rev,mxt] ):
                return path  


            if not add_points_to_path(path,[d,fwd,"D "+mxt]):
                return path                                            
            for index in range(0,len(third_bezier_turn)):
                if index == 0:
                    if not add_points_to_path(path, [third_bezier_turn[index],rev]):
                        return path
                else:
                    if not add_points_to_path(path, [third_bezier_turn[index],fwd]):
                        return path
            if not add_points_to_path(path, [d,rev,mxt] ):
                return path 

            if not add_points_to_path(path, [a,fwd,"A "+mxt]):
                return path                                            
            for index in range(0,len(fourth_bezier_turn)):
                if index == 0:
                    if not add_points_to_path(path, [fourth_bezier_turn[index],rev]):
                        return path
                else:
                    if not add_points_to_path(path, [fourth_bezier_turn[index],fwd]):
                        return path
            #if not add_points_to_path(path, [a,rev,mxt] ):
            if not add_points_to_path(path, [a_spiral,rev,mxt] ):
                return path 

            # get A'B'C'D' (prepare next ABCD points)
            b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
            d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

            if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
                return path

            a_new, b_new = compute_x1_x2_int_points(d2_int, b1_int, nav, logger)
            c_new, d_new = compute_x1_x2_int_points(b2_int, d1_int, nav, logger)

            if not check_points_for_nones(a_new, b_new, c_new, d_new):
                return path

            a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int 

            # get moving points A1 - ... - D2 spiral
            a1, a2 = compute_x1_x2_points(d2_int_prev, b, nav, logger)
            b1, b2 = compute_x1_x2_points(b, c, nav, logger)
            c1, c2 = compute_x1_x2_points(c, d, nav, logger)
            d1, d2 = compute_x1_x2_points(d, a, nav, logger)

            for point in [a,b,c,d,a1,b1,c1,d1,a2,b2,c2,d2]:
                if point is None:
                    return path

            a1_spiral = nav.get_coordinate(a1, a, 90, spiralSidesInterval)
            _, a_spiral = compute_x1_x2(d,a,spiralSidesInterval,nav)

            for point in [a1_spiral, a_spiral]:
                if point is None:
                    if not _break:
                        _break = True
                        break
            if _break : 
                break

            first_bezier_turn = compute_bezier_points(a2,b,b1)
            second_bezier_turn = compute_bezier_points(b2,c,c1)
            third_bezier_turn = compute_bezier_points(c2,d,d1)
            fourth_bezier_turn = compute_bezier_points(d2,a_spiral,a1_spiral)

    while True:
        # get A'B'C'D' (prepare next ABCD points)
        b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
        d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

        if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
            return path

        d2_int_prev = d2_int

        a_new, b_new = compute_x1_x2_int_points(d2_int, b1_int, nav, logger)
        c_new, d_new = compute_x1_x2_int_points(b2_int, d1_int, nav, logger)

        if not check_points_for_nones(a_new, b_new, c_new, d_new):
            break

        # get moving points A1 - ... - D2 spiral
        a1, a2 = compute_x1_x2_points(d2_int_prev, b, nav, logger)
        b1, b2 = compute_x1_x2_points(b, c, nav, logger)
        c1, c2 = compute_x1_x2_points(c, d, nav, logger)
        d1, d2 = compute_x1_x2_points(d, a, nav, logger)

        for point in [a,b,c,d,a1,b1,c1,d1,a2,b2,c2,d2]:
            if point is None:
                if not _break:
                    _break = True
                    break
        if _break : 
            break

        a1_spiral = nav.get_coordinate(a1, a, 90, spiralSidesInterval)
        _ , a_spiral = compute_x1_x2(d,a,spiralSidesInterval,nav)

        for point in [a1_spiral, a_spiral]:
            if point is None:
                if not _break:
                    _break = True
                    break
        if _break : 
            break
        
        first_bezier_turn = compute_bezier_points(a2,b,b1)
        second_bezier_turn = compute_bezier_points(b2,c,c1)
        third_bezier_turn = compute_bezier_points(c2,d,d1)
        fourth_bezier_turn = compute_bezier_points(d2,a_spiral,a1_spiral)

        for point in first_bezier_turn:
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, [point,fwd]):
                _break = True
                break
        if _break : 
            break
        
        for point in second_bezier_turn:
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, [point,fwd]):
                _break = True
                break
        if _break : 
            break

        for point in third_bezier_turn:
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, [point,fwd]):
                _break = True
                break
        if _break : 
            break
        
        for point in fourth_bezier_turn:
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, [point,fwd]):
                _break = True
                break
        if _break : 
            break

        a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int

    if nav.get_distance(a,b) >= nav.get_distance(b,c):
        return add_forward_backward_path([a,b,c,d], nav, logger, SI_speed_fwd, SI_speed_rev, path)       
    else:
        return add_forward_backward_path([c,b,a,d], nav, logger, SI_speed_rev, SI_speed_fwd, path)

def build_path(abcd_points: list, nav: navigation.GPSComputing, logger: utility.Logger, SI_speed_fwd: float, SI_speed_rev: float):
    path = []
    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    # get moving points A1 - ... - D2 spiral
    a1, a2 = compute_x1_x2_points(a, b, nav, logger)
    b1, b2 = compute_x1_x2_points(b, c, nav, logger)
    c1, c2 = compute_x1_x2_points(c, d, nav, logger)
    d1, d2 = compute_x1_x2_points(d, a, nav, logger)
    d2_spiral = compute_x2_spiral(d, a, nav, logger)

    # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
    if not add_points_to_path(path, [a,SI_speed_fwd], [a1,SI_speed_fwd], [a2,SI_speed_fwd], [b1,SI_speed_fwd], [b2,SI_speed_fwd], [c1,SI_speed_fwd], [c2,SI_speed_fwd], [d1,SI_speed_fwd], [d2_spiral,SI_speed_fwd]):
        return add_forward_backward_path([a, b, c, d], nav, logger, SI_speed_fwd, SI_speed_rev, path)

    # get A'B'C'D' (prepare next ABCD points)
    b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
    d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

    if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
        return add_forward_backward_path([a, b, c, d], nav, logger, SI_speed_fwd, SI_speed_rev, path)

    a_new, b_new = compute_x1_x2_int_points(d2_int, b1_int, nav, logger)
    c_new, d_new = compute_x1_x2_int_points(b2_int, d1_int, nav, logger)

    if not check_points_for_nones(a_new, b_new, c_new, d_new):
        return add_forward_backward_path([a, b, c, d], nav, logger, SI_speed_fwd, SI_speed_rev, path)

    a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int

    # keep reducing sides for spiral
    while True:
        # get A'B'C'D' (prepare next ABCD points)
        b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
        d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

        if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
            break

        a_new, b_new = compute_x1_x2_int_points(d2_int, b1_int, nav, logger)
        c_new, d_new = compute_x1_x2_int_points(b2_int, d1_int, nav, logger)

        if not check_points_for_nones(a_new, b_new, c_new, d_new):
            break

        # get moving points A1 - ... - D2 spiral
        a1, a2 = compute_x1_x2_points(d2_int_prev, b, nav, logger)
        b1, b2 = compute_x1_x2_points(b, c, nav, logger)
        c1, c2 = compute_x1_x2_points(c, d, nav, logger)
        d1, d2 = compute_x1_x2_points(d, a, nav, logger)
        d2_spiral = compute_x2_spiral(d, a, nav, logger)

        # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
        if not add_points_to_path(path, [a1,SI_speed_fwd], [a2,SI_speed_fwd], [b1,SI_speed_fwd], [b2,SI_speed_fwd], [c1,SI_speed_fwd], [c2,SI_speed_fwd], [d1,SI_speed_fwd], [d2_spiral,SI_speed_fwd]):
            break

        a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int

    if nav.get_distance(a,b) >= nav.get_distance(b,c):
        return add_forward_backward_path([a,b,c,d], nav, logger, SI_speed_fwd, SI_speed_rev, path)       
    else:
        return add_forward_backward_path([c,b,a,d], nav, logger, SI_speed_rev, SI_speed_fwd, path)

def compute_x1_x2(point_a, point_b, distance, nav: navigation.GPSComputing):
    """
    Computes and returns two points on given vector [A, B], where vector [A, X1] = distance, vector [X2, B] = distance
    :param point_a:
    :param point_b:
    :param distance:
    :param nav:
    :return:
    """

    ab_dist = nav.get_distance(point_a, point_b)

    if ab_dist < distance:
        raise ValueError("Size of AB vector (" + str(ab_dist) + ") should be greater than a given distance: " +
                         str(distance))

    x1 = nav.get_point_on_vector(point_a, point_b, distance)
    x2 = nav.get_point_on_vector(point_a, point_b, ab_dist - distance)
    return x1, x2


def reduce_field_size(abcd_points: list, reduce_size, nav: navigation.GPSComputing):
    """
    Reduces given ABCD field for given distance from each side, returns new ABCD field
    :param abcd_points:
    :param reduce_size:
    :param nav:
    :return:
    """

    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]
    b1_dist, b2_dist = compute_x1_x2(b, c, reduce_size, nav)
    d1_dist, d2_dist = compute_x1_x2(d, a, reduce_size, nav)
    a_new, b_new = compute_x1_x2(d2_dist, b1_dist, reduce_size, nav)
    c_new, d_new = compute_x1_x2(b2_dist, d1_dist, reduce_size, nav)
    return [a_new, b_new, c_new, d_new]


def emergency_field_defining(vesc_engine: adapters.VescAdapterV3, gps: adapters.GPSUbloxAdapter,
                             nav: navigation.GPSComputing, cur_log_dir, logger_full: utility.Logger):
    msg = "Using emergency field creation..."
    logger_full.write(msg + "\n")
    print(msg)

    starting_point = gps.get_last_position()

    msg = "Moving forward..."
    logger_full.write(msg + "\n")
    print(msg)
    vesc_engine.start_moving(engine_key=vesc_engine.PROPULSION_KEY)
    time.sleep(config.EMERGENCY_MOVING_TIME)
    vesc_engine.stop_moving(engine_key=vesc_engine.PROPULSION_KEY)

    msg = "Getting point A..."
    logger_full.write(msg + "\n")
    print(msg)
    time.sleep(2)
    A = gps.get_last_position()

    msg = "Computing rest points..."
    logger_full.write(msg + "\n")
    print(msg)
    B = nav.get_coordinate(A, starting_point, 180, config.EMERGENCY_FIELD_SIZE)
    C = nav.get_coordinate(B, A, 90, config.EMERGENCY_FIELD_SIZE)
    D = nav.get_coordinate(C, B, 90, config.EMERGENCY_FIELD_SIZE)

    msg = "Saving field.txt file..."
    logger_full.write(msg + "\n")
    print(msg)
    field = [A, B, C, D]
    save_gps_coordinates(field, "field.txt")
    save_gps_coordinates_raw([starting_point, A, B, C, D], cur_log_dir + "emergency_raw_field.txt")
    return field


def send_name_of_file_of_gps_history(ui_msg_queue: posix_ipc.MessageQueue,
                                     gps_file_dir: str,
                                     gps_file_name: str,
                                     logger_full: utility.Logger):
    """Send name of file if it exists using given message queue
    """

    if os.path.isfile(gps_file_dir + gps_file_name):
        ui_msg_queue.send(json.dumps({"last_gps_list_file": gps_file_dir + gps_file_name}))
    else:
        msg = f"Could not find {gps_file_dir}/used_gps_history.txt file to send previous points to the web UI"
        logger_full.write(msg + "\n")
        if config.VERBOSE:
            print(msg)

def main():
    time_start = utility.get_current_time()
    utility.create_directories(config.LOG_ROOT_DIR)

    # choose log dir dependent continuing previous path or not
    if config.CONTINUE_PREVIOUS_PATH:
        last_log_dir = utility.get_last_dir_name(config.LOG_ROOT_DIR)
        if last_log_dir is not None:
            log_cur_dir = config.LOG_ROOT_DIR + last_log_dir + "/"
        else:
            log_cur_dir = config.LOG_ROOT_DIR + time_start + "/"
    else:
        log_cur_dir = config.LOG_ROOT_DIR + time_start + "/"

    utility.create_directories(log_cur_dir, config.DEBUG_IMAGES_PATH, config.DATA_GATHERING_DIR)

    try:
        ui_msg_queue = posix_ipc.MessageQueue(config.QUEUE_NAME_UI_MAIN)
    except KeyboardInterrupt:
        raise KeyboardInterrupt
    except:
        ui_msg_queue = None

    image_saver = utility.ImageSaver()
    if config.ALLOW_GATHERING:
        image_saver.set_counter(len(glob.glob(config.DATA_GATHERING_DIR + "*.jpg")), "gathering")

    notification = NotificationClient(time_start)
    data_collector = datacollection.DataCollector(notification,
                                                  load_from_file=config.CONTINUE_PREVIOUS_PATH,
                                                  file_path=log_cur_dir + config.DATACOLLECTOR_SAVE_FILE,
                                                  ui_msg_queue=ui_msg_queue)
    working_zone_polygon = Polygon(config.WORKING_ZONE_POLY_POINTS)
    nav = navigation.GPSComputing()
    logger_full = utility.Logger(log_cur_dir + "log full.txt", append_file=config.CONTINUE_PREVIOUS_PATH)
    logger_table = utility.Logger(log_cur_dir + "log table.csv", append_file=config.CONTINUE_PREVIOUS_PATH)

    # get smoothie and vesc addresses
    smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
    if "vesc" in smoothie_vesc_addr:
        vesc_address = smoothie_vesc_addr["vesc"]
    else:
        msg = "Couldn't get vesc's USB address!"
        print(msg)
        logger_full.write(msg + "\n")
        notification.setStatus(SyntheseRobot.HS)
        exit(1)
    if config.SMOOTHIE_BACKEND == 1:
        smoothie_address = config.SMOOTHIE_HOST
    else:
        if "smoothie" in smoothie_vesc_addr:
            smoothie_address = smoothie_vesc_addr["smoothie"]
        else:
            msg = "Couldn't get smoothie's USB address!"
            print(msg)
            logger_full.write(msg + "\n")
            notification.setStatus(SyntheseRobot.HS)
            exit(1)

    # load yolo networks
    print("Loading periphery detector...")
    if config.PERIPHERY_WRAPPER == 1:
        periphery_detector = detection.YoloTRTDetector(
            config.PERIPHERY_MODEL_PATH,
            config.PERIPHERY_CLASSES_FILE,
            config.PERIPHERY_CONFIDENCE_THRESHOLD,
            config.PERIPHERY_NMS_THRESHOLD)
    elif config.PERIPHERY_WRAPPER == 2:
        periphery_detector = detection.YoloOpenCVDetection(config.PERIPHERY_CLASSES_FILE, config.PERIPHERY_CONFIG_FILE,
                                                           config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_INPUT_SIZE,
                                                           config.PERIPHERY_CONFIDENCE_THRESHOLD,
                                                           config.PERIPHERY_NMS_THRESHOLD, config.PERIPHERY_DNN_BACKEND,
                                                           config.PERIPHERY_DNN_TARGET)
    else:
        msg = "Wrong config.PERIPHERY_WRAPPER = " + str(config.PERIPHERY_WRAPPER) + " code. Exiting."
        logger_full.write(msg + "\n")
        notification.setStatus(SyntheseRobot.HS)
        exit(1)

    print("Loading precise detector...")
    if config.PRECISE_WRAPPER == 1:
        precise_detector = detection.YoloTRTDetector(
            config.PERIPHERY_MODEL_PATH,
            config.PERIPHERY_CLASSES_FILE,
            config.PERIPHERY_CONFIDENCE_THRESHOLD,
            config.PERIPHERY_NMS_THRESHOLD)
        if config.CONTINUOUS_INFORMATION_SENDING:
            notification.set_treated_plant(precise_detector.get_classes_names())
    elif config.PRECISE_WRAPPER == 2:
        precise_detector = detection.YoloOpenCVDetection(config.PRECISE_CLASSES_FILE, config.PRECISE_CONFIG_FILE,
                                                         config.PRECISE_WEIGHTS_FILE, config.PRECISE_INPUT_SIZE,
                                                         config.PRECISE_CONFIDENCE_THRESHOLD,
                                                         config.PRECISE_NMS_THRESHOLD, config.PRECISE_DNN_BACKEND,
                                                         config.PRECISE_DNN_TARGET)
        if config.CONTINUOUS_INFORMATION_SENDING:
            notification.set_treated_plant(detection.classes)
    else:
        msg = "Wrong config.PRECISE_WRAPPER = " + str(config.PRECISE_WRAPPER) + " code. Exiting."
        logger_full.write(msg + "\n")
        notification.setStatus(SyntheseRobot.HS)
        exit(1)

    # load and send trajectory to the UI if continuing work
    if config.CONTINUE_PREVIOUS_PATH:
        if ui_msg_queue is not None:
            send_name_of_file_of_gps_history(ui_msg_queue, log_cur_dir, "used_gps_history.txt", logger_full)
        else:
            msg = "GPS message queue connection is not established (None), canceling gps sending to UI"
            logger_full.write(msg + "\n")
            if config.VERBOSE:
                print(msg)

    # sensors picking
    report_field_names = ['temp_fet_filtered', 'temp_motor_filtered', 'avg_motor_current',
                          'avg_input_current', 'rpm', 'input_voltage']

    try:
        msg = "Initializing..."
        print(msg)
        logger_full.write(msg + "\n")

        vesc_speed = config.SI_SPEED_FWD*config.MULTIPLIER_SI_SPEED_TO_RPM

        # stubs.GPSStub(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP) as gps, \
        # utility.MemoryManager(config.DATA_GATHERING_DIR, config.FILES_TO_KEEP_COUNT) as memory_manager, \
        with \
            utility.TrajectorySaver(log_cur_dir + "used_gps_history.txt",
                                    config.CONTINUE_PREVIOUS_PATH) as trajectory_saver, \
            adapters.VescAdapterV3(vesc_address, config.VESC_BAUDRATE, config.VESC_ALIVE_FREQ, config.VESC_CHECK_FREQ,
                                   config.VESC_STOPPER_CHECK_FREQ) as vesc_engine, \
            adapters.SmoothieAdapter(smoothie_address) as smoothie, \
            adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP) as gps, \
            adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                             config.CROP_H_TO, config.CV_ROTATE_CODE,
                                             config.ISP_DIGITAL_GAIN_RANGE_FROM,
                                             config.ISP_DIGITAL_GAIN_RANGE_TO,
                                             config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                             config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                             config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                             config.CAMERA_H, config.CAMERA_FRAMERATE,
                                             config.CAMERA_FLIP_METHOD) as camera, \
            ExtractionManagerV3(smoothie, camera, logger_full, data_collector, image_saver,
                                log_cur_dir, periphery_detector, precise_detector,
                                config.CAMERA_POSITIONS, config.PDZ_DISTANCES, vesc_engine) as extraction_manager_v3, \
            navigation.NavigationPrediction(logger_full=logger_full, nav=nav, log_cur_dir=log_cur_dir) as navigation_prediction :            

            # load previous path
            if config.CONTINUE_PREVIOUS_PATH:
                # TODO: create path manager

                msg = "Loading previous path points"
                logger_full.write(msg + "\n")

                if config.CONTINUOUS_INFORMATION_SENDING:
                    notification.set_field(load_coordinates(config.INPUT_GPS_FIELD_FILE))

                # TODO: check if files exist and handle damaged/incorrect data cases
                with open(config.PREVIOUS_PATH_POINTS_FILE, "rb") as path_points_file:
                    path_points = pickle.load(path_points_file)
                with open(config.PREVIOUS_PATH_INDEX_FILE, "r") as path_index_file:
                    str_index = path_index_file.readline()
                    if str_index == "":
                        msg = "Path start index file " + config.PREVIOUS_PATH_INDEX_FILE + " is empty!"
                        print(msg)
                        logger_full.write(msg + "\n")
                        notification.setStatus(SyntheseRobot.HS)
                        exit(1)
                    path_start_index = int(str_index)  # TODO check if possible to convert

                # check if index is ok
                if path_start_index == -1:
                    msg = "Previous path is already passed"
                    print(msg)
                    logger_full.write(msg + "\n")
                    notification.stop()
                    exit(0)
                elif path_start_index >= len(path_points) or path_start_index < 1:
                    msg = "Path start index " + str(path_start_index) + " is out of path points list range (loaded " + \
                        str(len(path_points)) + " points) (start index can't be zero as 1rst point is starting point)"
                    print(msg)
                    logger_full.write(msg + "\n")
                    notification.setStatus(SyntheseRobot.HS)
                    exit(1)

            # load field points and generate new path
            else:
                if config.USE_EMERGENCY_FIELD_GENERATION:
                    field_gps_coords = emergency_field_defining(vesc_engine, gps, nav, log_cur_dir, logger_full)
                else:
                    msg = "Loading " + config.INPUT_GPS_FIELD_FILE
                    logger_full.write(msg + "\n")

                    field_gps_coords = load_coordinates(config.INPUT_GPS_FIELD_FILE)  # [A, B, C, D]

                # check field corner points count
                if len(field_gps_coords) == 4:
                    # TODO: save field in debug

                    field_gps_coords = reduce_field_size(field_gps_coords, config.FIELD_REDUCE_SIZE, nav)
                    print("field_gps_coords : ",field_gps_coords)
                    # TODO: save reduced field in debug

                    # generate path points
                    path_start_index = 1
                    if config.TRADITIONAL_PATH:
                        path_points = build_path(field_gps_coords, nav, logger_full, config.SI_SPEED_FWD, config.SI_SPEED_REV)
                    if config.BEZIER_PATH:
                        print(field_gps_coords)
                        path_points = build_bezier_with_corner_path(field_gps_coords, nav, logger_full, config.SI_SPEED_FWD, config.SI_SPEED_REV)
                    if config.FORWARD_BACKWARD_PATH:
                        a,b,c,d = field_gps_coords[0], field_gps_coords[1], field_gps_coords[2], field_gps_coords[3]
                        if nav.get_distance(a,b) >= nav.get_distance(b,c):
                            path_points = add_forward_backward_path([a,b,c,d], nav, logger_full, config.SI_SPEED_FWD, config.SI_SPEED_REV, [])
                        else:
                            path_points = add_forward_backward_path([d,a,b,c], nav, logger_full, config.SI_SPEED_FWD, config.SI_SPEED_REV, [])
                    
                    msg = "Generated " + str(len(path_points)) + " points."
                    logger_full.write(msg + "\n")

                elif len(field_gps_coords) == 2: 
                    path_start_index = 1
                    path_points = field_gps_coords

                else:
                    msg = "Expected 4 or 2 gps corner points, got " + str(len(field_gps_coords)) + "\nField:\n" + str(
                        field_gps_coords)
                    print(msg)
                    logger_full.write(msg + "\n")
                    notification.setStatus(SyntheseRobot.HS)
                    exit(1)
                # TODO: save field in debug

                if config.CONTINUOUS_INFORMATION_SENDING:
                    notification.set_field(field_gps_coords)

                # save path points and point to start from index
                with open(config.PREVIOUS_PATH_POINTS_FILE, "wb") as path_points_file:
                    pickle.dump(path_points, path_points_file)
                with open(config.PREVIOUS_PATH_INDEX_FILE, "w") as path_index_file:
                    path_index_file.write(str(path_start_index))

            if len(path_points) > 0:
                save_gps_coordinates(path_points, log_cur_dir + "current_path_points.txt")
                msg = "Current path points are successfully saved."
                print(msg)
                logger_full.write(msg + "\n")
            else:
                msg = "List of path points is empty, saving canceled."
                print(msg)
                logger_full.write(msg + "\n")

            if len(path_points) < 2:
                msg = "Expected at least 2 points in path, got " + str(len(path_points)) + \
                      " instead (1st point is starting point)."
                print(msg)
                logger_full.write(msg + "\n")
                notification.setStatus(SyntheseRobot.HS)
                exit(1)

            # set smoothie's A axis to 0 (nav turn wheels)
            response = smoothie.set_current_coordinates(A=0)
            if response != smoothie.RESPONSE_OK:
                msg = "Failed to set A=0 on smoothie (turning wheels init position), response message:\n" + response
                print(msg)
                logger_full.write(msg + "\n")

            """
            # ask permission to start moving
            msg = "Initializing done. Press enter to start moving."
            input(msg)
            logger_full.write(msg + "\n")
            """

            msg = 'GpsQ|Raw ang|Res ang|Ord ang|Sum ang|Distance    |Adapter|Smoothie|PointStatus|deviation|side dev|centroid factor|cruise factor'
            print(msg)
            logger_full.write(msg + "\n")
            msg = 'GpsQ,Raw ang,Res ang,Ord ang,Sum ang,Distance,Adapter,Smoothie,'
            for field_name in report_field_names:
                msg += field_name + ","
            msg = msg[:-1]
            logger_table.write(msg + "\n")

            # path points visiting loop
            with open(config.PREVIOUS_PATH_INDEX_FILE, "r+") as path_index_file:
                next_calibration_time = time.time() + config.CORK_CALIBRATION_MIN_TIME
                
                try:
                    start_position = utility.average_point(gps,trajectory_saver,nav)
                except:
                    pass

                if ui_msg_queue is not None:
                    try:
                        ui_msg_queue.send(json.dumps({"start": True}))
                    except:
                        pass
                
                last_direction_of_travel = None #1 -> moving forward #-1 -> moving backward

                if config.NAVIGATION_TEST_MODE:
                    path_end_index = sys.maxsize
                else:
                    path_end_index = len(path_points)

                for i in range(path_start_index, path_end_index):
                    
                    if config.NAVIGATION_TEST_MODE:
                        dist_here_point_a = nav.get_distance(start_position, config.POINT_A[0])
                        dist_here_point_b = nav.get_distance(start_position, config.POINT_B[0])

                        if dist_here_point_a>dist_here_point_b:
                            from_to = [config.POINT_B[0], config.POINT_A[0]]
                            speed = config.POINT_A[1]
                        else:
                            from_to = [config.POINT_A[0], config.POINT_B[0]]
                            speed = config.POINT_B[1]

                        display_instruction_path = from_to[0:2]
                            
                    else:
                        from_to = [path_points[i - 1][0], path_points[i][0]] 
                        speed = path_points[i][1]

                        i_inf = i-config.DELTA_DISPLAY_INSTRUCTION_PATH if i>=config.DELTA_DISPLAY_INSTRUCTION_PATH else 0
                        i_sup = i+config.DELTA_DISPLAY_INSTRUCTION_PATH if i+config.DELTA_DISPLAY_INSTRUCTION_PATH<path_end_index else path_end_index-1
                        display_instruction_path = [elem[0] for elem in path_points[i_inf:i_sup]]

                    if ui_msg_queue is not None and config.DISPLAY_INSTRUCTION_PATH:
                        ui_msg_queue.send(json.dumps({"display_instruction_path": display_instruction_path}))

                    if last_direction_of_travel is None:
                        last_direction_of_travel = (speed>=0) if 1 else -1 #1 -> moving forward #-1 -> moving backward
                    
                    direction_of_travel = (speed>=0) if 1 else -1 #1 -> moving forward #-1 -> moving backward

                    if direction_of_travel != last_direction_of_travel:
                        vesc_engine.set_rpm(speed * config.MULTIPLIER_SI_SPEED_TO_RPM, vesc_engine.PROPULSION_KEY)

                    if config.WHEELS_STRAIGHT_CHANGE_DIRECTION_OF_TRAVEL and direction_of_travel != last_direction_of_travel:
                            vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
                            response = smoothie.custom_move_to(A_F=config.A_F_MAX, A=0)
                            if response != smoothie.RESPONSE_OK: 
                                msg = "Smoothie response is not ok: " + response
                                print(msg)
                                logger_full.write(msg + "\n")
                            smoothie.wait_for_all_actions_done()

                    i_inf = i + 1 if i + 1 < path_end_index else path_end_index
                    i_sup = i + 1 + config.FUTURE_NUMBER_OF_POINTS \
                        if i + config.FUTURE_NUMBER_OF_POINTS < path_end_index else path_end_index

                    move_to_point_and_extract(
                        from_to,
                        gps,
                        vesc_engine,
                        smoothie,
                        camera,
                        periphery_detector,
                        precise_detector,
                        logger_full,
                        logger_table,
                        report_field_names,
                        trajectory_saver,
                        working_zone_polygon,
                        config.DEBUG_IMAGES_PATH,
                        nav,
                        data_collector,
                        log_cur_dir,
                        image_saver,
                        notification,
                        extraction_manager_v3,
                        ui_msg_queue,
                        speed,
                        False,
                        navigation_prediction,
                        path_points[i_inf:i_sup],
                        not i == path_start_index
                        if config.FIRST_POINT_NO_EXTRACTIONS and config.CONTINUE_PREVIOUS_PATH else True)

                    if config.NAVIGATION_TEST_MODE:
                        response = smoothie.custom_move_to(A_F=config.A_F_MAX, A=0)
                        if response != smoothie.RESPONSE_OK:  # TODO: what if response is not ok?
                            msg = "Couldn't turn wheels before other navigation test, smoothie response:\n" + response
                            print(msg)
                            logger_full.write(msg + "\n")
                        else:
                            with open(config.LAST_ANGLE_WHEELS_FILE, "w+") as wheels_angle_file:
                                wheels_angle_file.write(str(smoothie.get_adapter_current_coordinates()["A"]))
                        test_continue = input("Press enter to continue the test, type anything to exit.")
                        if test_continue != "":
                            notification.stop()
                            break
                        try:
                            start_position = utility.average_point(gps,trajectory_saver,nav)
                        except:
                            pass
                        if ui_msg_queue is not None:
                            ui_msg_queue.send(json.dumps({"clear_path": True}))
                        #reload config if kp or ki change
                        importlib.reload(config)
                    
                    last_direction_of_travel = (speed>=0) if 1 else -1 #1 -> moving forward #-1 -> moving backward

                    # save path progress (index of next point to move)
                    path_index_file.seek(0)
                    path_index_file.write(str(i + 1))
                    path_index_file.flush()

                    """
                    msg = "Starting memory cleaning"
                    logger_full.write(msg + "\n")
                    cleaning_start_t = time.time()
                    memory_manager.start_clean_manual_blocking()
                    cleaning_end_t = time.time()
                    msg = "Cleaning elapsed time: " + str(cleaning_end_t - cleaning_start_t)
                    logger_full.write(msg + "\n")
                    """

                    # calibration
                    if time.time() > next_calibration_time:
                        msg = "Calibrating cork after reaching path point. Current config.CORK_CALIBRATION_MIN_TIME is "\
                              + str(config.CORK_CALIBRATION_MIN_TIME)
                        logger_full.write(msg + "\n")

                        next_calibration_time = time.time() + config.CORK_CALIBRATION_MIN_TIME
                        smoothie.ext_calibrate_cork()

                # mark path as passed (set next point index to -1)
                path_index_file.seek(0)
                path_index_file.write(str(-1))
                path_index_file.flush()

            msg = "Path is successfully passed."
            print(msg)
            logger_full.write(msg + "\n")
            notification.stop()
    except KeyboardInterrupt:
        msg = "Stopped by a keyboard interrupt (Ctrl + C)\n" + traceback.format_exc()
        print(msg)
        logger_full.write(msg + "\n")
        notification.stop()
        if ui_msg_queue is not None:
            ui_msg_queue.close()
    except:
        msg = "Exception occurred:\n" + traceback.format_exc()
        print(msg)
        logger_full.write(msg + "\n")
        notification.setStatus(SyntheseRobot.HS)
        if ui_msg_queue is not None:
            ui_msg_queue.close()
    finally:
        # put the wheel straight
        # vesc z axis calibration in case it's used for z axis control instead of smoothie
        # (to prevent cork breaking when smoothie calibrates)
        smoothie_safe_calibration = True
        if config.EXTRACTION_CONTROLLER == 2:
            with adapters.VescAdapterV3(
                    vesc_address,
                    config.VESC_BAUDRATE,
                    config.VESC_ALIVE_FREQ,
                    config.VESC_CHECK_FREQ,
                    config.VESC_STOPPER_CHECK_FREQ) as vesc_engine:
                # Z-5 fix (move cork little down to stop touching stopper)
                vesc_engine.set_rpm(config.VESC_EXTRACTION_CALIBRATION_Z5_FIX_RPM, vesc_engine.EXTRACTION_KEY)
                vesc_engine.set_time_to_move(
                    config.VESC_EXTRACTION_CALIBRATION_Z5_FIX_TIME,
                    vesc_engine.EXTRACTION_KEY)
                vesc_engine.start_moving(vesc_engine.EXTRACTION_KEY)
                vesc_engine.wait_for_stop(vesc_engine.EXTRACTION_KEY)

                # calibration
                vesc_engine.set_rpm(config.VESC_EXTRACTION_CALIBRATION_RPM, vesc_engine.EXTRACTION_KEY)
                vesc_engine.set_time_to_move(
                    config.VESC_EXTRACTION_CALIBRATION_MAX_TIME,
                    vesc_engine.EXTRACTION_KEY)
                vesc_engine.start_moving(vesc_engine.EXTRACTION_KEY)
                res = vesc_engine.wait_for_stopper_hit(
                    vesc_engine.EXTRACTION_KEY,
                    config.VESC_EXTRACTION_CALIBRATION_MAX_TIME)
                vesc_engine.stop_moving(vesc_engine.EXTRACTION_KEY)
                if not res:
                    smoothie_safe_calibration = False
                    print(
                        "Stopped vesc EXTRACTION engine calibration due timeout (stopper signal wasn't received)\n",
                        "WHEELS POSITION WILL NOT BE SAVED PROPERLY!", sep="")
        if smoothie_safe_calibration:
            with adapters.SmoothieAdapter(smoothie_address) as smoothie:
                # put the wheel straight
                msg = "Put the wheel straight"
                logger_full.write(msg + "\n")
                if config.VERBOSE:
                    print(msg)
                with open(config.LAST_ANGLE_WHEELS_FILE, "r+") as wheels_angle_file:
                    angle = float(wheels_angle_file.read())
                    smoothie.set_current_coordinates(A=angle)
                    response = smoothie.custom_move_to(A_F=config.A_F_MAX, A=0)
                    if response != smoothie.RESPONSE_OK:  # TODO: what if response is not ok?
                        msg = "Couldn't turn wheels before shutdown, smoothie response:\n" + response
                        print(msg)
                        logger_full.write(msg + "\n")
                    else:
                        wheels_angle_file.seek(0)
                        wheels_angle_file.write(str(smoothie.get_adapter_current_coordinates()["A"]))

        # save adapter points history
        try:
            # TODO: reduce history positions to 1 to save RAM
            # TODO: gps adapter is blocking now if has no points
            adapter_points_history = gps.get_last_positions_list()
            if len(adapter_points_history) > 0:
                save_gps_coordinates(adapter_points_history, log_cur_dir + "adapter_gps_history.txt")
            else:
                msg = "adapter_gps_history list has 0 elements!"
                print(msg)
                logger_full.write(msg + "\n")
        except:
            pass

        # save session statistics TODO: its temporary
        msg = "Saving statistics..."
        logger_full.write(msg + "\n")
        print(msg)
        try:
            data_collector.save_all_data(log_cur_dir + config.STATISTICS_OUTPUT_FILE)
            data_collector.dump_to_file(log_cur_dir + config.DATACOLLECTOR_SAVE_FILE)
        except:
            msg = "Failed:\n" + traceback.format_exc()
            logger_full.write(msg + "\n")
            print(msg)
            pass

        # close log and hardware connections
        msg = "Closing loggers..."
        logger_full.write(msg + "\n")
        print(msg)
        logger_full.close()
        logger_table.close()

        try:
            posix_ipc.unlink_message_queue(config.QUEUE_NAME_UI_MAIN)
        except:
            pass

        if detection.YoloDarknetDetector.webStream is not None:
            detection.YoloDarknetDetector.webStream.terminate()
            detection.YoloDarknetDetector.webStream.join()

        print("Safe disable is done.")


if __name__ == '__main__':
    main()
