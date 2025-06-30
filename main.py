"""Spiral movement, detection and extraction over given by ABCD points area

BE SURE TO IMPORT ANY NATUITION MODULES AFTER AND ONLY AFTER(!) CONFIG.PY LOADING!
This is required to prevent modules loading corrupted config before main resolves this problem.
"""

import threading
import os
import sys
from turtle import speed
import time
import traceback
from matplotlib.patches import Polygon
import math
import cv2 as cv
import numpy as np
import pickle
import posix_ipc
import json
import glob
import importlib
import subprocess

import safe_import_of_config
safe_import_of_config.make_import()
from config import config

import adapters
import navigation
import utility
import detection
import stubs
import extraction
import datacollection
from extraction import ExtractionManagerV3
from shared_class.robot_synthesis import RobotSynthesis
from notification import NotificationClient
import connectors

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


def save_gps_coordinates(points: list, file_name: str):
    """
    Saves given list of points using QGIS format
    :param points:
    :param file_name:
    :return:
    """

    with open(file_name, "w") as file:
        for point in points:
            if isinstance(point[0], list):
                str_point = str(point[0][0]) + " " + \
                    str(point[0][1]) + " " + str(point[1]) + "\n"
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
        save_image(config.DATA_GATHERING_DIR, frame, IMAGES_COUNTER,
                   label, utility.get_current_time())

    # debug image saving
    if config.SAVE_DEBUG_IMAGES:
        # draw time on frame
        cur_time = utility.get_current_time()
        left, top = 30, 30
        label_size, base_line = cv.getTextSize(
            cur_time + " No: " + str(IMAGES_COUNTER), cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, label_size[1])
        frame = cv.rectangle(frame, (left, top - round(1.5 * label_size[1])),
                             (left + round(1.5 *
                              label_size[0]), top + base_line),
                             (0, 0, 255), cv.FILLED)
        frame = cv.putText(frame, cur_time + " No: " + str(IMAGES_COUNTER), (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.75,
                           (0, 0, 0), 2)

        # draw data on frame
        frame = utility.ImageSaver.draw_zone_circle(
            frame, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, undistorted_zone_radius)
        frame = utility.ImageSaver.draw_zone_poly(frame, poly_zone_points_cv)
        frame = detection.draw_boxes(frame, plants_boxes)
        save_image(img_output_dir, frame, IMAGES_COUNTER, label, cur_time)


def move_to_point_and_extract(coords_from_to: list,
                              gps: adapters.GPSUbloxAdapter,
                              vesc_engine: adapters.VescAdapterV4,
                              smoothie: adapters.SmoothieAdapter,
                              camera: adapters.CameraAdapterIMX219_170,
                              periphery_det: detection.YoloOpenCVDetection,
                              precise_det: detection.YoloOpenCVDetection,
                              logger_full: utility.Logger,
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
                              allow_extractions: bool,
                              x_scan_poly: list,
                              cur_field):
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
    :param cur_field: None or list of 4 ABCD points which are describing current field robot is working on.
    :return:
    """

    if config.ALLOW_FIELD_LEAVING_PROTECTION and cur_field is not None and len(cur_field) > 2:
        enable_field_leaving_protection = True
    else:
        enable_field_leaving_protection = False
        if config.ALLOW_FIELD_LEAVING_PROTECTION:
            if cur_field is None:
                msg = f"WARNING: robot field leaving protection WILL NOT WORK as given field is None"
                print(msg)
                logger_full.write(msg)
            elif len(cur_field) < 3:
                msg = f"WARNING: robot field leaving protection WILL NOT WORK as given field contains " \
                      f"{len(cur_field)} points (required ar least 3 points)"
                print(msg)
                logger_full.write(msg)

    extract = SI_speed > 0 and allow_extractions

    vesc_speed = SI_speed * config.MULTIPLIER_SI_SPEED_TO_RPM
    speed_fast = config.SI_SPEED_FAST * config.MULTIPLIER_SI_SPEED_TO_RPM
    vesc_speed_fast = speed_fast if SI_speed >= 0 else -speed_fast
    navigation_prediction.set_SI_speed(SI_speed)

    raw_angles_history = []
    detections_period = []
    navigations_period = []
    stop_helping_point = nav.get_coordinate(
        coords_from_to[1], coords_from_to[0], 90, 1000)
    learn_go_straight_index = 0
    learn_go_straight_history = []

    last_skipped_point = coords_from_to[0]
    start_Nav_while = True
    last_correct_raw_angle = 0
    point_status = "origin"
    last_corridor_side = 0
    current_corridor_side = 1
    almost_start = 0

    prev_maneuver_time = time.time()
    working_mode_slow = 1
    working_mode_fast = 2
    working_mode_switching = 3
    current_working_mode = working_mode_slow
    last_working_mode = 0
    # True if robot is close to one of current movement vector points, False otherwise; False if speed limit near points is disabled
    close_to_end = config.USE_SPEED_LIMIT
    bumper_is_pressed = None

    # message queue sending temporary performance tracker
    if config.QUEUE_TRACK_PERFORMANCE:
        ui_msg_queue_perf = {
            "max_time": 0,
            "min_time": float("inf"),
            "total_time": 0,
            "total_sends": 0,
            "timeouts_exceeded": 0
        }

    # x movements during periphery scans
    x_scan_cur_idx = 0
    x_scan_idx_increasing = True

    # set camera to the Y min
    res = smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                              Y_F=config.Y_F_MAX,
                                              X=smoothie.smoothie_to_mm(
                                                  (config.X_MAX - config.X_MIN) / 2, "X"),
                                              Y=smoothie.smoothie_to_mm(config.Y_MIN, "Y"))
    if res != smoothie.RESPONSE_OK:
        msg = "INIT: Failed to move camera to Y min, smoothie response:\n" + res
        logger_full.write(msg + "\n")
    smoothie.wait_for_all_actions_done()

    # TODO: maybe should add sleep time as camera currently has delay

    if config.AUDIT_MODE:
        vesc_engine.set_target_rpm(vesc_speed, vesc_engine.PROPULSION_KEY)
        vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)

    try:
        notificationQueue = posix_ipc.MessageQueue(
            config.QUEUE_NAME_UI_NOTIFICATION)
    except KeyboardInterrupt:
        raise KeyboardInterrupt
    except:
        notificationQueue = None

    degraded_navigation_mode = False

    number_navigation_cycle_without_gps = 0

    point_reading_t = last_send_gps_time = slow_mode_time = time.time()

    have_time_for_inference = True
    predictor_next_gps_expected_ts = float("inf")

    # main navigation control loop
    while True:
        # gps point reading time predictor
        if have_time_for_inference and config.ALLOW_GPS_TIME_PREDICTIONS_LIMITING_INFERENCE:
            if time.time() + config.INFERENCE_MAX_TICK_TIME > predictor_next_gps_expected_ts:
                have_time_for_inference = False

        if have_time_for_inference:
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
                image_saver.save_image(frame, config.DATA_GATHERING_DIR,
                                       plants_boxes=plants_boxes, counter_key="gathering")

            if extract:
                msg = "View frame time: " + str(frame_t - start_t) + "\t\tPeri. det. time: " + \
                      str(per_det_end_t - per_det_start_t)
            else:
                msg = "View frame time: " + str(frame_t - start_t) + "\t\tPeri. det. (extractions are off) time: " + \
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
                    data_collector.save_all_data(
                        log_cur_dir + config.AUDIT_OUTPUT_FILE)

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

                    if ExtractionManagerV3.any_plant_in_zone(
                            plants_boxes,
                            x_scan_poly[x_scan_cur_idx] if config.ALLOW_X_MOVEMENT_DURING_SCANS else working_zone_polygon):
                        vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
                        if config.VERBOSE_EXTRACT:
                            msg = "[VERBOSE EXTRACT] Stopping the robot because we have detected plant(s)."
                            logger_full.write_and_flush(msg+"\n")
                        data_collector.add_vesc_moving_time_data(
                            vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))
                        # TODO this 0 rpm "movement" is to prevent robot movement during extractions, need to add this in future to rest speed modes too
                        vesc_engine.set_time_to_move(config.VESC_MOVING_TIME, vesc_engine.PROPULSION_KEY)
                        vesc_engine.set_target_rpm(0, vesc_engine.PROPULSION_KEY)
                        vesc_engine.set_current_rpm(0, vesc_engine.PROPULSION_KEY)
                        vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)
                        

                        # single precise center scan before calling for PDZ scanning and extractions
                        if config.ALLOW_PRECISE_SINGLE_SCAN_BEFORE_PDZ and not config.ALLOW_X_MOVEMENT_DURING_SCANS:
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

                        vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)

                        msg = "Applying force step forward after extractions cycle(s)"
                        logger_full.write(msg + "\n")
                        if config.VERBOSE:
                            print(msg)
                        vesc_engine.set_time_to_move(config.STEP_FORWARD_TIME, vesc_engine.PROPULSION_KEY)
                        vesc_engine.set_target_rpm(
                            config.SI_SPEED_STEP_FORWARD * config.MULTIPLIER_SI_SPEED_TO_RPM,
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
                            X=smoothie.smoothie_to_mm(
                                (config.X_MAX - config.X_MIN) / 2, "X"),
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
                        vesc_engine.set_target_rpm(vesc_speed, vesc_engine.PROPULSION_KEY)
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

                    if ExtractionManagerV3.any_plant_in_zone(
                            plants_boxes,
                            x_scan_poly[x_scan_cur_idx] if config.ALLOW_X_MOVEMENT_DURING_SCANS else working_zone_polygon):
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
                            X=smoothie.smoothie_to_mm(
                                (config.X_MAX - config.X_MIN) / 2, "X"),
                            Y=smoothie.smoothie_to_mm(config.Y_MIN, "Y"))
                        if res != smoothie.RESPONSE_OK:
                            msg = "INIT: Failed to move camera to Y min, smoothie response:\n" + res
                            logger_full.write(msg + "\n")
                        smoothie.wait_for_all_actions_done()

                        current_working_mode = working_mode_slow
                        slow_mode_time = time.time()
                        vesc_engine.set_target_rpm(
                            vesc_speed, vesc_engine.PROPULSION_KEY)
                        continue

                    sm_cur_pos = smoothie.get_smoothie_current_coordinates(
                        convert_to_mms=False)
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

                    if ExtractionManagerV3.any_plant_in_zone(
                            plants_boxes,
                            x_scan_poly[x_scan_cur_idx] if config.ALLOW_X_MOVEMENT_DURING_SCANS else working_zone_polygon):
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
                            X=smoothie.smoothie_to_mm(
                                (config.X_MAX - config.X_MIN) / 2, "X"),
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
                        cur_vesc_rpm = vesc_engine.get_current_rpm(
                            vesc_engine.PROPULSION_KEY)
                        if cur_vesc_rpm != vesc_speed:
                            msg = f"Applying slow speed {vesc_speed} at 'fast mode' " \
                                  f"(was {cur_vesc_rpm}) " \
                                  f"because of close_to_end flag trigger"
                            if config.LOG_SPEED_MODES:
                                logger_full.write(msg + "\n")
                            if config.PRINT_SPEED_MODES:
                                print(msg)
                            vesc_engine.set_target_rpm(
                                vesc_speed, vesc_engine.PROPULSION_KEY)
                            vesc_engine.set_current_rpm(
                                vesc_speed, vesc_engine.PROPULSION_KEY)
                    else:
                        cur_vesc_rpm = vesc_engine.get_current_rpm(
                            vesc_engine.PROPULSION_KEY)
                        if cur_vesc_rpm != vesc_speed_fast:
                            msg = f"Applying fast speed {vesc_speed_fast} at 'fast mode' (was {cur_vesc_rpm})"
                            if config.LOG_SPEED_MODES:
                                logger_full.write(msg + "\n")
                            if config.PRINT_SPEED_MODES:
                                print(msg)
                            vesc_engine.set_target_rpm(
                                vesc_speed_fast, vesc_engine.PROPULSION_KEY)
                            vesc_engine.set_current_rpm(
                                vesc_speed_fast, vesc_engine.PROPULSION_KEY)

        # NAVIGATION CONTROL
        cur_pos_obj = gps.get_last_position_v2()
        cur_pos = cur_pos_obj.as_old_list

        nav_start_t = time.time()

        if start_Nav_while:
            navigation_period = 1
        else:
            navigation_period = nav_start_t - prev_maneuver_time

        navigations_period.append(navigation_period)
        # time reference to decide the number of detection before resuming gps.get
        prev_maneuver_time = nav_start_t
        # print("tock")

        if start_Nav_while:
            prev_pos_obj = cur_pos_obj
            prev_pos = prev_pos_obj.as_old_list
            start_Nav_while = False

        # mu_navigations_period, sigma_navigations_period = utility.mu_sigma(navigations_period)

        navigation_prediction.set_current_lat_long(cur_pos)

        # skip same points (non-blocking reading returns old point if new point isn't available yet)
        if math.isclose(cur_pos_obj.creation_ts, prev_pos_obj.creation_ts):
            # stop robot if there's no new points for a while
            if time.time() - point_reading_t > config.GPS_POINT_TIME_BEFORE_STOP:
                vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
                msg = f"Stopping the robot due to exceeding time 'GPS_POINT_TIME_BEFORE_STOP=" \
                      f"{config.GPS_POINT_TIME_BEFORE_STOP}' limit without new gps points from adapter"
                logger_full.write_and_flush(msg + "\n")
                data_collector.add_vesc_moving_time_data(
                    vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))

                gps_reconnect_ts = time.time()

                while True:
                    cur_pos_obj = gps.get_last_position_v2()
                    cur_pos = cur_pos_obj.as_old_list

                    if math.isclose(cur_pos_obj.creation_ts, prev_pos_obj.creation_ts):
                        # reconnect gps adapter to ublox if there's no gps points for a while
                        if time.time() - gps_reconnect_ts > config.GPS_POINT_TIME_BEFORE_RECONNECT:
                            gps.reconnect()
                            gps_reconnect_ts = time.time()
                            msg = "Called GPS adapter to reconnect to ublox due to waiting too much for a new GPS " \
                                  "point (new points filter)"
                            if config.VERBOSE:
                                print(msg)
                            logger_full.write_and_flush(msg + "\n")
                    else:
                        msg = "New GPS point received, continuing movement"
                        logger_full.write_and_flush(msg + "\n")
                        vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)
                        break
            else:
                continue

        # gps points reading time predictor
        predictor_next_gps_expected_ts = cur_pos_obj.receiving_ts + config.GPS_POINT_WAIT_TIME_MAX
        have_time_for_inference = True

        # points filter by quality flag
        if cur_pos[2] != "4" and config.ALLOW_GPS_BAD_QUALITY_NTRIP_RESTART:
            # restart ntrip if enough time passed since the last ntrip restart
            navigation.NavigationV3.restart_ntrip_service(logger_full)

            # stop robot due to bad point quality if allowed
            if config.ALLOW_GPS_BAD_QUALITY_STOP:
                vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
                logger_full.write_and_flush(
                    "Stopping the robot for lack of quality gps 4, waiting for it...\n")
                data_collector.add_vesc_moving_time_data(
                    vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))

                prev_bad_quality_pos_obj = cur_pos_obj
                gps_reconnect_ts = time.time()

                while True:
                    cur_pos_obj = gps.get_last_position_v2()
                    cur_pos = cur_pos_obj.as_old_list

                    # check if it's a new point
                    if math.isclose(cur_pos_obj.creation_ts, prev_bad_quality_pos_obj.creation_ts):
                        # reconnect gps adapter to ublox if there's no gps points for a while
                        if time.time() - gps_reconnect_ts > config.GPS_POINT_TIME_BEFORE_RECONNECT:
                            gps.reconnect()
                            gps_reconnect_ts = time.time()
                            msg = "Called GPS adapter to reconnect to ublox due to waiting too much for a new " \
                                  "GPS point (quality filter)"
                            if config.VERBOSE:
                                print(msg)
                            logger_full.write_and_flush(msg + "\n")
                        continue
                    else:
                        prev_bad_quality_pos_obj = cur_pos_obj

                    # check if it's a good quality point
                    if cur_pos[2] != "4":
                        # restart ntrip if enough time passed since the last ntrip restart
                        navigation.NavigationV3.restart_ntrip_service(
                            logger_full)
                    else:
                        msg = "The gps has regained quality 4, starting movement"
                        if config.VERBOSE:
                            print(msg)
                        logger_full.write_and_flush(msg + "\n")
                        vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)
                        break

        # points filter by distance
        prev_cur_distance = nav.get_distance(prev_pos, cur_pos)
        if config.ALLOW_GPS_PREV_CUR_DIST_STOP and prev_cur_distance > config.PREV_CUR_POINT_MAX_DIST:
            vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
            msg = f"Stopping the robot due to GPS points filter by distance (assuming current position point " \
                  f"{str(cur_pos)} is wrong as distance between current position and prev. position {str(prev_pos)}" \
                  f" is bigger than config.PREV_CUR_POINT_MAX_DIST={str(config.PREV_CUR_POINT_MAX_DIST)})"
            logger_full.write_and_flush(msg + "\n")
            data_collector.add_vesc_moving_time_data(
                vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))

            prev_bad_quality_pos_obj = cur_pos_obj
            gps_reconnect_ts = distance_wait_start_ts = time.time()

            while True:
                if time.time() - distance_wait_start_ts > config.GPS_DIST_WAIT_TIME_MAX:
                    msg = f"Stopping waiting for good prev-cur distance due to timeout, using current point " \
                          f"{cur_pos} and starting moving again"
                    if config.VERBOSE:
                        print(msg)
                    logger_full.write_and_flush(msg + "\n")
                    vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)
                    break

                cur_pos_obj = gps.get_last_position_v2()
                cur_pos = cur_pos_obj.as_old_list

                # check if it's a new point
                if math.isclose(cur_pos_obj.creation_ts, prev_bad_quality_pos_obj.creation_ts):
                    # reconnect gps adapter to ublox if there's no gps points for a while
                    if time.time() - gps_reconnect_ts > config.GPS_POINT_TIME_BEFORE_RECONNECT:
                        gps.reconnect()
                        gps_reconnect_ts = time.time()
                        msg = "Called GPS adapter to reconnect to ublox due to waiting too much for a new " \
                              "GPS point (distance filter)"
                        if config.VERBOSE:
                            print(msg)
                        logger_full.write_and_flush(msg + "\n")
                    continue
                else:
                    prev_bad_quality_pos_obj = cur_pos_obj

                # check if it's a good quality point or ignore point quality if bad quality stop is not allowed
                if cur_pos[2] != "4" and config.ALLOW_GPS_BAD_QUALITY_NTRIP_RESTART:
                    # restart ntrip if enough time passed since the last ntrip restart
                    navigation.NavigationV3.restart_ntrip_service(logger_full)
                    continue

                # check if distance became ok
                prev_cur_distance = nav.get_distance(prev_pos, cur_pos)
                if prev_cur_distance <= config.PREV_CUR_POINT_MAX_DIST:
                    msg = f"Starting moving again after GPS points filter by distance as distance become OK " \
                          f"({str(prev_cur_distance)})"
                    logger_full.write_and_flush(msg + "\n")
                    vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)
                    break

        point_reading_t = time.time()

        trajectory_saver.save_point(cur_pos)
        if ui_msg_queue is not None and time.time()-last_send_gps_time >= 1:
            try:
                ui_msg_queue_send_ts = time.time()
                ui_msg_queue.send(json.dumps(
                    {"last_gps": cur_pos}), timeout=config.QUEUE_WAIT_TIME_MAX)
                last_send_gps_time = time.time()

                if config.QUEUE_TRACK_PERFORMANCE:
                    ui_msg_queue_send_et = last_send_gps_time - ui_msg_queue_send_ts
                    if ui_msg_queue_send_et < ui_msg_queue_perf["min_time"]:
                        ui_msg_queue_perf["min_time"] = ui_msg_queue_send_et
                    if ui_msg_queue_send_et > ui_msg_queue_perf["max_time"]:
                        ui_msg_queue_perf["max_time"] = ui_msg_queue_send_et
                    ui_msg_queue_perf["total_time"] += ui_msg_queue_send_et
                    ui_msg_queue_perf["total_sends"] += 1
            except posix_ipc.BusyError:
                msg = f"Current position wasn't sent to ui_msg_queue likely due to sending timeout " \
                      f"(max wait time: config.QUEUE_WAIT_TIME_MAX={config.QUEUE_WAIT_TIME_MAX}"
                logger_full.write(msg + "\n")

                if config.QUEUE_TRACK_PERFORMANCE:
                    ui_msg_queue_perf["timeouts_exceeded"] += 1

        if config.CONTINUOUS_INFORMATION_SENDING and not degraded_navigation_mode:
            notification.set_current_coordinate(cur_pos)

        distance = nav.get_distance(cur_pos, coords_from_to[1])

        last_corridor_side = current_corridor_side
        perpendicular, current_corridor_side = nav.get_deviation(
            coords_from_to[0], coords_from_to[1], cur_pos)

        # stop the robot if it has left the field
        if enable_field_leaving_protection:
            for pt_idx in range(len(cur_field)):
                last_point = pt_idx + 1 == len(cur_field)

                if last_point:
                    deviation, side = nav.get_deviation(cur_field[pt_idx], cur_field[0], cur_pos)
                else:
                    deviation, side = nav.get_deviation(cur_field[pt_idx], cur_field[pt_idx + 1], cur_pos)

                if side == -1 and deviation > config.LEAVING_PROTECTION_DISTANCE_MAX:
                    vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
                    data_collector.add_vesc_moving_time_data(
                        vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))
                    msg = f"Robot is stopped due to leaving the field. Cur pos: '{str(cur_pos)}'; " \
                          f"Field comparison vector - P1: '{str(cur_field[pt_idx])}', " \
                          f"P2: '{str(cur_field[0] if last_point else cur_field[pt_idx + 1])}'"
                    print(msg)
                    logger_full.write_and_flush(msg + "\n")
                    notification.set_robot_state_and_wait_send(RobotSynthesis.ANTI_THEFT)
                    raise Exception("LEAVING_FIELD")

        # check if arrived
        _, side = nav.get_deviation(
            coords_from_to[1], stop_helping_point, cur_pos)
        # if distance <= config.COURSE_DESTINATION_DIFF:  # old way
        if side != 1:  # TODO: maybe should use both side and distance checking methods at once
            vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
            data_collector.add_vesc_moving_time_data(
                vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))
            # msg = "Arrived (allowed destination distance difference " + str(config.COURSE_DESTINATION_DIFF) + " mm)"
            # TODO: service will reload script even if it done his work?
            msg = "Arrived to " + str(coords_from_to[1])
            # print(msg)
            logger_full.write(msg + "\n")

            # put the wheel straight
            if wheels_straight:
                response = smoothie.custom_move_to(A_F=config.A_F_MAX, A=0)
                if response != smoothie.RESPONSE_OK:  # TODO: what if response is not ok?
                    msg = "Couldn't turn wheels to center (0), smoothie response:\n" + \
                        response
                    print(msg)
                    logger_full.write(msg + "\n")
                else:
                    # save wheels angle
                    with open(config.LAST_ANGLE_WHEELS_FILE, "w+") as wheels_angle_file:
                        wheels_angle_file.write(
                            str(smoothie.get_adapter_current_coordinates()["A"]))
            break

        # TODO check for bug: arrival check applies single speed for all path (while multiple speeds are applied)
        # check if can arrived
        if vesc_engine.get_current_rpm(vesc_engine.PROPULSION_KEY) / config.MULTIPLIER_SI_SPEED_TO_RPM * \
                config.MANEUVERS_FREQUENCY > nav.get_distance(cur_pos, coords_from_to[1]):
            vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
            data_collector.add_vesc_moving_time_data(
                vesc_engine.get_last_movement_time(vesc_engine.PROPULSION_KEY))
            msg = "Will have arrived before the next point to " + \
                str(coords_from_to[1])
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
        # raw_angle_legacy = nav.get_angle(prev_pos, cur_pos, cur_pos, coords_from_to[1])
        raw_angle_centroid = nav.get_angle(
            prev_pos, cur_pos, coords_from_to[0], coords_from_to[1])
        raw_angle_cruise = - current_corridor_side * math.log(1+perpendicular)

        if nav.get_distance(coords_from_to[0], coords_from_to[1]) < config.CORNER_THRESHOLD and nav.get_distance(coords_from_to[1], future_points[0][0]) < config.CORNER_THRESHOLD:
            # if abs(raw_angle_legacy)>config.LOST_THRESHOLD:
            centroid_factor = config.CENTROID_FACTOR_LOST
            cruise_factor = 1/centroid_factor
        else:
            centroid_factor = config.CENTROID_FACTOR_ORIENTED
            cruise_factor = 1

        raw_angle = raw_angle_centroid*centroid_factor + raw_angle_cruise*cruise_factor

        # raw_angle = butter_lowpass_filter(raw_angle, 0.5, 4, 6)

        if config.LEARN_GO_STRAIGHT:
            if config.MIN_PERPENDICULAR_GO_STRAIGHT >= perpendicular:
                learn_go_straight_index += 1
                learn_go_straight_history.append(raw_angle)
                if len(learn_go_straight_history) >= config.VALUES_LEARN_GO_STRAIGHT:
                    learn_go_straight = sum(
                        learn_go_straight_history)/len(learn_go_straight_history)
                    msg = f"Average angle applied to the wheel for the robot to have found : {learn_go_straight}."
                    logger_full.write_and_flush(msg + "\n")
                    # TODO opening and closing file 4 times per second
                    with open(config.LEARN_GO_STRAIGHT_FILE, "w+") as learn_go_straight_file:
                        learn_go_straight_file.write(str(learn_go_straight))
            else:
                learn_go_straight_index = 0

        # NAVIGATION STATE MACHINE
        if prev_cur_distance < config.PREV_CUR_POINT_MIN_DIST:
            raw_angle = last_correct_raw_angle
            # print("The distance covered is low")
            point_status = "skipped"

            # register the last position where the robot almost stop
            # in order to disable the deviation servo for a config.POURSUIT_LIMIT length and then resume in cruise
            last_skipped_point = cur_pos
        else:
            last_correct_raw_angle = raw_angle
            point_status = "correct"

        almost_start = nav.get_distance(last_skipped_point, cur_pos)

        # sum(e)
        if len(raw_angles_history) >= config.WINDOW:
            raw_angles_history.pop(0)
        raw_angles_history.append(raw_angle)
        # print("len(raw_angles_history):",len(raw_angles_history))
        sum_angles = sum(raw_angles_history)
        if sum_angles > config.SUM_ANGLES_HISTORY_MAX:
            msg = "Sum angles " + str(sum_angles) + " is bigger than max allowed value " + \
                str(config.SUM_ANGLES_HISTORY_MAX) + ", setting to " + \
                str(config.SUM_ANGLES_HISTORY_MAX)
            # print(msg)
            logger_full.write(msg + "\n")
            # Get Ready to go down as soon as the angle get negatif
            raw_angles_history[len(raw_angles_history) -
                               1] -= sum_angles - config.SUM_ANGLES_HISTORY_MAX
            sum_angles = config.SUM_ANGLES_HISTORY_MAX
        elif sum_angles < -config.SUM_ANGLES_HISTORY_MAX:
            msg = "Sum angles " + str(sum_angles) + " is less than min allowed value " + \
                str(-config.SUM_ANGLES_HISTORY_MAX) + ", setting to " + \
                str(-config.SUM_ANGLES_HISTORY_MAX)
            # print(msg)
            logger_full.write(msg + "\n")
            # get Ready to go up as soon as the angle get positive:
            raw_angles_history[len(raw_angles_history)-1] += - \
                sum_angles - config.SUM_ANGLES_HISTORY_MAX
            sum_angles = -config.SUM_ANGLES_HISTORY_MAX

        # KP = 0.2*0,55
        # KI = 0.0092*0,91

        KP = getSpeedDependentConfigParam(
            config.KP, SI_speed, "KP", logger_full)
        KI = getSpeedDependentConfigParam(
            config.KI, SI_speed, "KI", logger_full)

        angle_kp_ki = raw_angle * KP + sum_angles * KI

        # smoothie -Value == left, Value == right
        target_angle_sm = angle_kp_ki * -config.A_ONE_DEGREE_IN_SMOOTHIE
        # target_angle_sm = 0     #Debug COVID_PLACE
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
                str(config.A_MAX) + \
                " due to exceeding smoothie allowed values range."
            # print(msg)
            logger_full.write(msg + "\n")
            order_angle_sm = config.A_MAX
        elif order_angle_sm < config.A_MIN:
            msg = "Global order angle changed from " + str(order_angle_sm) + " to config.A_MIN = " + \
                str(config.A_MIN) + \
                " due to exceeding smoothie allowed values range."
            # print(msg)
            logger_full.write(msg + "\n")
            order_angle_sm = config.A_MIN

        # cork x movement during periphery scans control
        if config.ALLOW_X_MOVEMENT_DURING_SCANS:
            if x_scan_idx_increasing:
                x_scan_cur_idx += 1
                if x_scan_cur_idx >= len(config.X_MOVEMENT_CAMERA_POSITIONS):
                    x_scan_idx_increasing = False
                    x_scan_cur_idx -= 2
            else:
                x_scan_cur_idx -= 1
                if x_scan_cur_idx < 0:
                    x_scan_idx_increasing = True
                    x_scan_cur_idx += 2
        # TODO do we check SI_speed earlier and do proper calculations and angle validations if here we'll get here a negative order angle instead of positive?
        response = smoothie.custom_move_to(
            A_F=config.A_F_MAX,
            A=order_angle_sm if SI_speed >= 0 else -order_angle_sm,
            X_F=config.X_MOVEMENT_CAMERA_X_F[x_scan_cur_idx] if config.ALLOW_X_MOVEMENT_DURING_SCANS else None,
            X=config.X_MOVEMENT_CAMERA_POSITIONS[x_scan_cur_idx] if config.ALLOW_X_MOVEMENT_DURING_SCANS else None
        )

        if response != smoothie.RESPONSE_OK:
            msg = "Couldn't turn wheels! Smoothie response:\n" + response
            print(msg)
            logger_full.write(msg + "\n")
        else:
            # TODO opening and closing file too often (likely 4 times per second)
            # save wheels angle
            with open(config.LAST_ANGLE_WHEELS_FILE, "w+") as wheels_angle_file:
                wheels_angle_file.write(
                    str(smoothie.get_adapter_current_coordinates()["A"]))

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
        if current_corridor_side == -1:
            corridor = "left"
        elif current_corridor_side == 1:
            corridor = "right"

        raw_angle_cruise = round(raw_angle_cruise, 2)

        msg = str(gps_quality).ljust(5) + \
            str(raw_angle).ljust(8) + \
            str(angle_kp_ki).ljust(8) + \
            str(order_angle_sm).ljust(8) + \
            str(sum_angles).ljust(8) + \
            str(distance).ljust(13) + \
            str(ad_wheels_pos).ljust(8) + \
            str(sm_wheels_pos).ljust(9) + \
            point_status.ljust(12) + \
            str(perpendicular).ljust(10) + \
            corridor.ljust(9) + \
            str(centroid_factor).ljust(16) + \
            str(cruise_factor).ljust(14)
        print(msg)
        logger_full.write(msg + "\n")

        # TODO vesc sensors are being asked 4 times per second
        # send voltage and track bumper state
        vesc_data = vesc_engine.get_sensors_data(
            report_field_names, vesc_engine.PROPULSION_KEY)
        if vesc_data is not None and "input_voltage" in vesc_data:
            if bumper_is_pressed is None:
                bumper_is_pressed = not vesc_data["input_voltage"] > config.VESC_BUMBER_UNTRIGGER_VOLTAGE
                if bumper_is_pressed:
                    msg = f"Bumper is pressed initially before starting moving to point. " \
                          f"({vesc_data['input_voltage']}V)"
                    logger_full.write(msg + "\n")
            elif not bumper_is_pressed and vesc_data["input_voltage"] < config.VESC_BUMBER_TRIGGER_VOLTAGE:
                bumper_is_pressed = True
                msg = f"Bumper was pressed. ({vesc_data['input_voltage']}V)"
                logger_full.write(msg + "\n")
            elif bumper_is_pressed and vesc_data["input_voltage"] > config.VESC_BUMBER_UNTRIGGER_VOLTAGE:
                bumper_is_pressed = False
                msg = f"Bumper was unpressed. ({vesc_data['input_voltage']}V)"
                logger_full.write(msg + "\n")

            if config.CONTINUOUS_INFORMATION_SENDING:
                notification.set_input_voltage(vesc_data["input_voltage"])

        prev_pos_obj = cur_pos_obj
        prev_pos = prev_pos_obj.as_old_list

        msg = "Nav calc time: " + str(time.time() - nav_start_t)
        logger_full.write(msg + "\n\n")

    if config.QUEUE_TRACK_PERFORMANCE:
        ui_msg_queue_perf["avg_time"] = ui_msg_queue_perf["total_time"] / \
            ui_msg_queue_perf["total_sends"]
        msg = f"Position sending performance report: {ui_msg_queue_perf}"
        if config.VERBOSE:
            print(msg)
        logger_full.write(msg + "\n")


def send_voltage_thread_tf(voltage_thread_alive, vesc_engine: adapters.VescAdapterV4, logger: utility.Logger, ui_msg_queue):
    """
    Thread function to monitor VESC input voltage and handle bumping events.
    This function continuously checks the VESC input voltage and emits in the main message queue.
    If the voltage drops below a certain threshold, it emits "Bumper" to the main message queue.
    If the voltage returns to normal, it emits "Reseting" to the main message queue and attempts to reset the VESC using a lifeline.
    """

    vesc_data = None
    isBumped = False
    nowReset = True
    while voltage_thread_alive():
        if vesc_engine is not None:
            try:
                vesc_data = vesc_engine.get_sensors_data(["input_voltage"], vesc_engine.PROPULSION_KEY)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except Exception as e:
                print(f"[Send voltage thread] -> Exception while getting VESC data: {e}")
                break

        if vesc_data is not None:
            if voltage_thread_alive():
                vesc_voltage = vesc_data.get("input_voltage", None)
                if vesc_voltage is not None:
                    if vesc_voltage < 12.0:
                        if isBumped:
                            msg = f"[Send voltage thread] -> Bumped, vesc voltage is {vesc_voltage}V."
                            logger.write_and_flush(msg + "\n")
                            print(msg)
                        isBumped = True
                        if ui_msg_queue is not None:
                            ui_msg_queue.send(json.dumps({"input_voltage": "Bumper"}))

                    elif isBumped and vesc_voltage >= 12.0:
                        msg = f"[Send voltage thread] -> Unbumped, vesc voltage is {vesc_voltage}V, resetting VESC with lifeline."
                        logger.write_and_flush(msg + "\n")
                        print(msg)
                        isBumped = False
                        nowReset = True
                        if ui_msg_queue is not None:
                            ui_msg_queue.send(json.dumps({"input_voltage": "Reseting"}))
                        utility.life_line_reset()
                        time.sleep(5)  # Usefull to not send a get_sensors_data request to early
                    else:
                        if nowReset:
                            msg = f"[Send voltage thread] -> VESC voltage is {vesc_voltage}V, no bump detected."
                            logger.write_and_flush(msg + "\n")
                            print(msg)
                            nowReset = False
                        if ui_msg_queue is not None:
                            ui_msg_queue.send(json.dumps({"input_voltage": vesc_voltage}))
        time.sleep(0.3)


def getSpeedDependentConfigParam(configParam: dict, SI_speed: float, paramName: str, logger_full: utility.Logger):
    if SI_speed in configParam:
        return configParam[SI_speed]
    else:
        msg = f"Speed SI {SI_speed} not present in {paramName}."
        if config.VERBOSE:
            print(msg)
        logger_full.write(msg + "\n")
        exit()


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

    # check if moving vector is too small for maneuvers
    if config.MANEUVER_START_DISTANCE * 2 >= cur_vec_dist:
        msg = "No place for maneuvers; config start maneuver distance is (that will be multiplied by 2): " + \
              str(config.MANEUVER_START_DISTANCE) + " current moving vector distance is: " + str(cur_vec_dist) + \
              " Given points are: " + str(point_a) + " " + str(point_b)
        # print(msg)
        logger.write(msg + "\n")
        return None, None

    point_x1 = nav.get_point_on_vector(
        point_a, point_b, config.MANEUVER_START_DISTANCE)
    point_x2 = nav.get_point_on_vector(
        point_a, point_b, cur_vec_dist - config.MANEUVER_START_DISTANCE)
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

    # check if moving vector is too small for maneuvers
    if config.MANEUVER_START_DISTANCE * 2 + config.SPIRAL_SIDES_INTERVAL >= cur_vec_dist:
        msg = "No place for maneuvers; Config maneuver distance is (that will be multiplied by 2): " + \
              str(config.MANEUVER_START_DISTANCE) + " Config spiral interval: " + str(config.SPIRAL_SIDES_INTERVAL) + \
              " Current moving vector distance is: " + str(cur_vec_dist) + " Given points are: " + str(point_a) + \
              " " + str(point_b)
        # print(msg)
        logger.write(msg + "\n")
        return None
    return nav.get_point_on_vector(
        point_a,
        point_b,
        cur_vec_dist - config.MANEUVER_START_DISTANCE - config.SPIRAL_SIDES_INTERVAL)


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

    # check if moving vector is too small for maneuvers
    if config.SPIRAL_SIDES_INTERVAL * 2 >= cur_vec_dist:
        msg = "No place for maneuvers; Config spiral interval (that will be multiplied by 2): " + \
              str(config.SPIRAL_SIDES_INTERVAL) + " Current moving vector distance is: " + str(cur_vec_dist) + \
              " Given points are: " + str(point_a) + " " + str(point_b)
        if config.VERBOSE:
            print(msg)
        logger.write(msg + "\n")
        return None, None

    point_x1_int = nav.get_point_on_vector(
        point_a, point_b, config.SPIRAL_SIDES_INTERVAL)
    point_x2_int = nav.get_point_on_vector(
        point_a, point_b, cur_vec_dist - config.SPIRAL_SIDES_INTERVAL)
    return point_x1_int, point_x2_int


def add_points_to_path(path: list, *args):
    """Tries to add given points into given path.

    Returns True if all points are added successfully
    Returns False if one of given points is None

    If point is None - previous not None points will be added, further points addition will is canceled and False is
    returned"""

    for point in args:
        if point is None:
            return False
        if len(point) > 1:
            if point[0] is None:
                return False
        path.append(point)
    return True


def check_points_for_nones(*args):
    """Checks if any of given points is None.

    Returns True if all given points are not Nones.
    Returns False if any of given points is None."""

    for point in args:
        if point is None:
            return False
    return True


def compute_bezier_points(point_0, point_1, point_2):
    t = np.linspace(0, 1, config.NUMBER_OF_BEZIER_POINT)
    coords = list()
    for i in t:
        x = (point_0[0] - 2 * point_1[0] + point_2[0]) * (i ** 2) + \
            (2 * point_1[0] - 2 * point_0[0]) * i + point_0[0]
        y = (point_0[1] - 2 * point_1[1] + point_2[1]) * (i ** 2) + \
            (2 * point_1[1] - 2 * point_0[1]) * i + point_0[1]
        coords.append([x, y])
    return coords


def get_rectangle_isosceles_side(turning_radius):
    # Bezier refer \Nextcloud\3. Engineering\navigation
    return (0.5*(turning_radius*((2**0.5)-1))**2)**0.5


def corner_finish_rounds(turning_radius: float):
    if config.VERBOSE:
        print("black corridor width at full steering %2.0f" %
              get_rectangle_isosceles_side(turning_radius), " millimeters")
    # how many corner round due to robot working width
    return int((get_rectangle_isosceles_side(turning_radius))/config.FIELD_REDUCE_SIZE)+1


def add_forward_backward_path(abcd_points: list, nav: navigation.GPSComputing, logger: utility.Logger, SI_speed_fwd: float, SI_speed_rev: float, currently_path: list):
    raise NotImplementedError(
        "an obsolete code, use build_forward_backward_path() instead")

    if not config.ADD_FORWARD_BACKWARD_TO_END_PATH and not config.FORWARD_BACKWARD_PATH:
        return currently_path

    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    fwd = SI_speed_fwd
    rev = SI_speed_rev

    while nav.get_distance(b, c) > config.SPIRAL_SIDES_INTERVAL:

        if not add_points_to_path(currently_path, [b, fwd]):
            return currently_path

        if not add_points_to_path(currently_path, [a, rev]):
            return currently_path

        a = compute_x1_x2(a, d, config.SPIRAL_SIDES_INTERVAL, nav)[0]
        b = compute_x1_x2(b, c, config.SPIRAL_SIDES_INTERVAL, nav)[0]

    if not add_points_to_path(currently_path, [b, fwd]):
        return currently_path

    if not add_points_to_path(currently_path, [a, rev]):
        return currently_path

    return currently_path


def build_forward_backward_path(abcd_points: list,
                                nav: navigation.GPSComputing,
                                logger: utility.Logger,
                                SI_speed_fwd: float,
                                SI_speed_rev: float,
                                path: list = None):
    """Builds zigzag (forward-backward) path to fill given ABCD field.
    Can process 4 non 90 degrees corners fields.

    Will append zigzag points into the existing path if it is not None, otherwise creates a path from scratch.
    Returns python list of gps [[latitude, longitude], speed] points."""

    if type(abcd_points) != list:
        msg = f"Given ABCD path must be a list, got {type(abcd_points).__name__} instead"
        raise TypeError(msg)

    if len(abcd_points) != 4:
        msg = f"Expected 4 ABCD points as input field, got {str(len(abcd_points))} points instead"
        raise ValueError(msg)

    for point_name, point in zip("ABCD", abcd_points):
        if type(point) != list:
            msg = f"Point {point_name} of given ABCD field must be a list, got {type(point).__name__} instead"
            raise TypeError(msg)
        if len(point) < 2:
            msg = f"Point {point_name} of given ABCD field must contain >=2 items, found {str(len(point))} instead"
            raise ValueError(msg)

    if path is None:
        path = []
    elif type(path) != list:
        msg = f"Given ABCD path must be a list type, got {type(path).__name__} instead"
        raise TypeError(msg)

    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    # separate stop-flags and BC & AD length control allows correct processing 4 corner non 90 degrees fields
    bc_dist_ok = ad_dist_ok = True

    while bc_dist_ok or ad_dist_ok:
        if not add_points_to_path(path, [b, SI_speed_fwd]):
            msg = f"Failed to add point B={str(b)} to path. This expected never to happen."
            raise RuntimeError(msg)

        if not add_points_to_path(path, [a, SI_speed_rev]):
            msg = f"Failed to add point A={str(a)} to path. This expected never to happen."
            raise RuntimeError(msg)

        if nav.get_distance(b, c) >= config.SPIRAL_SIDES_INTERVAL:
            b = nav.get_point_on_vector(b, c, config.SPIRAL_SIDES_INTERVAL)
        else:
            bc_dist_ok = False

        if nav.get_distance(a, d) >= config.SPIRAL_SIDES_INTERVAL:
            a = nav.get_point_on_vector(a, d, config.SPIRAL_SIDES_INTERVAL)
        else:
            ad_dist_ok = False

    return path


def build_bezier_with_corner_path(abcd_points: list, nav: navigation.GPSComputing, logger: utility.Logger, SI_speed_fwd: float, SI_speed_rev: float):
    raise NotImplementedError(
        "an obsolete code, use build_bezier_path() instead")

    path = []
    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    fwd = SI_speed_fwd
    rev = SI_speed_rev

    _break = False

    # get moving points A1 - ... - D2 spiral
    a1, a2 = compute_x1_x2_points(a, b, nav, logger)
    b1, b2 = compute_x1_x2_points(b, c, nav, logger)
    c1, c2 = compute_x1_x2_points(c, d, nav, logger)
    d1, d2 = compute_x1_x2_points(d, a, nav, logger)
    a1_spiral = nav.get_coordinate(a1, a, 90, config.SPIRAL_SIDES_INTERVAL)
    _, a_spiral = compute_x1_x2(d, a, config.SPIRAL_SIDES_INTERVAL, nav)

    if not add_points_to_path(path, [a, fwd]):
        raise RuntimeError("Failed to add original point A into generated path. "
                           "This could happen if input field's point A is None.")

    first_bezier_turn = compute_bezier_points(a2, b, b1)
    second_bezier_turn = compute_bezier_points(b2, c, c1)
    third_bezier_turn = compute_bezier_points(c2, d, d1)
    fourth_bezier_turn = compute_bezier_points(d2, a_spiral, a1_spiral)

    # minimum turning radius given in millimeter
    turning_radius = config.MANEUVER_START_DISTANCE

    if config.ADD_CORNER_TO_BEZIER_PATH:
        rnd = 0
        rnds = corner_finish_rounds(turning_radius)

        # example a 3meter radius requires 4 corners finish
        for rnd in range(rnds+1):
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            mxt = "corner rnd "+str(rnd)+"/"+str(rnds)

            # the direction is given along with the point in meter per second, signed
            # go to line forward, step back to the turning point "a1"

            if not add_points_to_path(path, [b, fwd, "B "+mxt]):
                return path
            for index in range(0, len(first_bezier_turn)):
                if index == 0:
                    if not add_points_to_path(path, [first_bezier_turn[index], rev]):
                        return path
                else:
                    if not add_points_to_path(path, [first_bezier_turn[index], fwd]):
                        return path
            if not add_points_to_path(path, [b, rev, mxt]):
                return path

            if not add_points_to_path(path, [c, fwd, "C "+mxt]):
                return path
            for index in range(0, len(second_bezier_turn)):
                if index == 0:
                    if not add_points_to_path(path, [second_bezier_turn[index], rev]):
                        return path
                else:
                    if not add_points_to_path(path, [second_bezier_turn[index], fwd]):
                        return path
            if not add_points_to_path(path, [c, rev, mxt]):
                return path

            if not add_points_to_path(path, [d, fwd, "D "+mxt]):
                return path
            for index in range(0, len(third_bezier_turn)):
                if index == 0:
                    if not add_points_to_path(path, [third_bezier_turn[index], rev]):
                        return path
                else:
                    if not add_points_to_path(path, [third_bezier_turn[index], fwd]):
                        return path
            if not add_points_to_path(path, [d, rev, mxt]):
                return path

            if not add_points_to_path(path, [a, fwd, "A "+mxt]):
                return path
            for index in range(0, len(fourth_bezier_turn)):
                if index == 0:
                    if not add_points_to_path(path, [fourth_bezier_turn[index], rev]):
                        return path
                else:
                    if not add_points_to_path(path, [fourth_bezier_turn[index], fwd]):
                        return path
            # if not add_points_to_path(path, [a,rev,mxt] ):
            if not add_points_to_path(path, [a_spiral, rev, mxt]):
                return path

            # get A'B'C'D' (prepare next ABCD points)
            b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
            d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

            if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
                return path

            a_new, b_new = compute_x1_x2_int_points(
                d2_int, b1_int, nav, logger)
            c_new, d_new = compute_x1_x2_int_points(
                b2_int, d1_int, nav, logger)

            if not check_points_for_nones(a_new, b_new, c_new, d_new):
                return path

            a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int

            # get moving points A1 - ... - D2 spiral
            a1, a2 = compute_x1_x2_points(d2_int_prev, b, nav, logger)
            b1, b2 = compute_x1_x2_points(b, c, nav, logger)
            c1, c2 = compute_x1_x2_points(c, d, nav, logger)
            d1, d2 = compute_x1_x2_points(d, a, nav, logger)

            for point in [a, b, c, d, a1, b1, c1, d1, a2, b2, c2, d2]:
                if point is None:
                    return path

            a1_spiral = nav.get_coordinate(
                a1, a, 90, config.SPIRAL_SIDES_INTERVAL)
            _, a_spiral = compute_x1_x2(
                d, a, config.SPIRAL_SIDES_INTERVAL, nav)

            for point in [a1_spiral, a_spiral]:
                if point is None:
                    if not _break:
                        _break = True
                        break
            if _break:
                break

            first_bezier_turn = compute_bezier_points(a2, b, b1)
            second_bezier_turn = compute_bezier_points(b2, c, c1)
            third_bezier_turn = compute_bezier_points(c2, d, d1)
            fourth_bezier_turn = compute_bezier_points(d2, a_spiral, a1_spiral)

    while True:
        # get A'B'C'D' (prepare next ABCD points)
        b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
        d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

        if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
            raise RuntimeError("Some of intermediate points [B1 B2 D1 D2] for next spiral generation are None. "
                               "This may happen if current distance between points is too small for robot maneuvers.")

        d2_int_prev = d2_int

        a_new, b_new = compute_x1_x2_int_points(d2_int, b1_int, nav, logger)
        c_new, d_new = compute_x1_x2_int_points(b2_int, d1_int, nav, logger)

        if not check_points_for_nones(a_new, b_new, c_new, d_new):
            raise RuntimeError("Some of next iteration field points [A_new B_new C_new D_new] are None. "
                               "This may happen if current distance between points is too small for robot maneuvers.")

        # get moving points A1 - ... - D2 spiral
        a1, a2 = compute_x1_x2_points(d2_int_prev, b, nav, logger)
        b1, b2 = compute_x1_x2_points(b, c, nav, logger)
        c1, c2 = compute_x1_x2_points(c, d, nav, logger)
        d1, d2 = compute_x1_x2_points(d, a, nav, logger)

        if None in [a, b, c, d, a1, b1, c1, d1, a2, b2, c2, d2]:
            if nav.get_distance(a, b) >= nav.get_distance(b, c):
                a = compute_x1_x2(a, d, config.SPIRAL_SIDES_INTERVAL, nav)[0]
                b = compute_x1_x2(b, c, config.SPIRAL_SIDES_INTERVAL, nav)[0]
                return add_forward_backward_path([a, b, c, d], nav, logger, SI_speed_fwd, SI_speed_rev, path)
            else:
                raise RuntimeError("Some of [A A1 A2 B B1 B2 C C1 C2 D D1 D2] points are None AND AB < BC. "
                                   "Old code, not sure why author raises an exception for such condition.")

        a1_spiral = nav.get_coordinate(a1, a, 90, config.SPIRAL_SIDES_INTERVAL)
        _, a_spiral = compute_x1_x2(d, a, config.SPIRAL_SIDES_INTERVAL, nav)

        if None in [a1_spiral, a_spiral]:
            raise RuntimeError(
                "One of [A_spiral A1_spiral] points are None. This case actually should never happen.")

        first_bezier_turn = compute_bezier_points(a2, b, b1)
        second_bezier_turn = compute_bezier_points(b2, c, c1)
        third_bezier_turn = compute_bezier_points(c2, d, d1)
        fourth_bezier_turn = compute_bezier_points(d2, a_spiral, a1_spiral)

        if nav.get_distance(a, b) >= nav.get_distance(b, c):

            if None in first_bezier_turn+second_bezier_turn:
                return add_forward_backward_path([a, b, c, d], nav, logger, SI_speed_fwd, SI_speed_rev, path)

            first_bezier_turn_with_speed = [[point, fwd]
                                            for point in first_bezier_turn]
            second_bezier_turn_with_speed = [
                [point, fwd] for point in second_bezier_turn]
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, *(first_bezier_turn_with_speed+second_bezier_turn_with_speed)):
                raise Exception(
                    "Error during generate path (build_bezier_with_corner_path:01) !")

            if None in third_bezier_turn+fourth_bezier_turn:
                return add_forward_backward_path([c, d, a, b], nav, logger, SI_speed_fwd, SI_speed_rev, path)

            third_bezier_turn_with_speed = [[point, fwd]
                                            for point in third_bezier_turn]
            fourth_bezier_turn_with_speed = [
                [point, fwd] for point in fourth_bezier_turn]
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, *(third_bezier_turn_with_speed+fourth_bezier_turn_with_speed)):
                raise Exception(
                    "Error during generate path (build_bezier_with_corner_path:02) !")

        else:

            first_bezier_turn_with_speed = [[point, fwd]
                                            for point in first_bezier_turn]
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, *(first_bezier_turn_with_speed)):
                raise Exception(
                    "Error during generate path (build_bezier_with_corner_path:03) !")

            if None in second_bezier_turn+third_bezier_turn:
                return add_forward_backward_path([b, c, d, a], nav, logger, SI_speed_fwd, SI_speed_rev, path)
            second_bezier_turn_with_speed = [
                [point, fwd] for point in second_bezier_turn]
            third_bezier_turn_with_speed = [[point, fwd]
                                            for point in third_bezier_turn]
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            if not add_points_to_path(path, *(second_bezier_turn_with_speed+third_bezier_turn_with_speed)):
                raise Exception(
                    "Error during generate path (build_bezier_with_corner_path:04) !")

            _, next_a2 = compute_x1_x2_points(d2_int, b_new, nav, logger)
            next_b1, _ = compute_x1_x2_points(b_new, c_new, nav, logger)

            if None in fourth_bezier_turn or None in [next_a2, b_new, next_b1]:
                return add_forward_backward_path([d, a, b, c], nav, logger, SI_speed_fwd, SI_speed_rev, path)
            # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
            fourth_bezier_turn_with_speed = [
                [point, fwd] for point in fourth_bezier_turn]
            if not add_points_to_path(path, *(fourth_bezier_turn_with_speed)):
                raise Exception(
                    "Error during generate path (build_bezier_with_corner_path:05 !")

        a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int


def build_bezier_path(abcd_points: list,
                      nav: navigation.GPSComputing,
                      logger: utility.Logger,
                      SI_speed_fwd: float,
                      SI_speed_rev: float):
    """Builds spiral path to fill given ABCD field.

    Fills field's missing center with zigzag (forward-backward) movement if config.ADD_FORWARD_BACKWARD_TO_END_PATH
    is set to True.
    Returns python list of gps [[latitude, longitude], speed] points."""

    if config.ADD_CORNER_TO_BEZIER_PATH:
        raise NotImplementedError(
            "config.ADD_CORNER_TO_BEZIER_PATH feature is not ready in new path builder yet")

    if type(abcd_points) != list:
        msg = f"Given ABCD path must be a list, got {type(abcd_points).__name__} instead"
        raise TypeError(msg)

    if len(abcd_points) != 4:
        msg = f"Expected 4 ABCD points as input field, got {str(len(abcd_points))} points instead"
        raise ValueError(msg)

    for point_name, point in zip("ABCD", abcd_points):
        if type(point) != list:
            msg = f"Point {point_name} of given ABCD field must be a list, got {type(point).__name__} instead"
            raise TypeError(msg)
        if len(point) < 2:
            msg = f"Point {point_name} of given ABCD field must contain >=2 items, found {str(len(point))} instead"
            raise ValueError(msg)

    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]
    path = []
    center_fill_start_point = 0  # 0 is unidentified, 1 is A, 2 is D

    if not add_points_to_path(path, [a, SI_speed_fwd]):
        raise RuntimeError(
            "Failed to add point A (the once of input field description points) into generated path")

    while True:
        # get moving points A1 - ... - D2 spiral
        a1, a2 = compute_x1_x2_points(a, b, nav, logger)
        b1, b2 = compute_x1_x2_points(b, c, nav, logger)
        c1, c2 = compute_x1_x2_points(c, d, nav, logger)
        d1, _ = compute_x1_x2_points(d, a, nav, logger)
        if not check_points_for_nones(a1, a2, b1, b2, c1, c2, d1):
            center_fill_start_point = 1
            break

        b_corner_bezier = compute_bezier_points(a2, b, b1)
        if not add_points_to_path(path, *map(lambda gps_point: [gps_point, SI_speed_fwd], b_corner_bezier)):
            raise RuntimeError(
                "Failed to add B corner's bezier curve to path. This expected never to happen.")

        c_corner_bezier = compute_bezier_points(b2, c, c1)
        if not add_points_to_path(path, *map(lambda gps_point: [gps_point, SI_speed_fwd], c_corner_bezier)):
            raise RuntimeError(
                "Failed to add C corner's bezier curve to path. This expected never to happen.")

        d_corner_bezier = compute_bezier_points(c2, d, d1)
        if not add_points_to_path(path, *map(lambda gps_point: [gps_point, SI_speed_fwd], d_corner_bezier)):
            raise RuntimeError(
                "Failed to add D corner's bezier curve to path. This expected never to happen.")

        # check before computing d2 and A corner bezier curve (see d2 computing comments below for details)
        if nav.get_distance(d, a) <= config.MANEUVER_START_DISTANCE * 2 + config.SPIRAL_SIDES_INTERVAL \
                or nav.get_distance(a, b) <= config.MANEUVER_START_DISTANCE:
            center_fill_start_point = 2
            break

        # d2 isn't as other x2 points as d2 distance from A is spiral_sides_interval + start_turn_distance
        # instead of just start_turn_distance, so DA acceptable length computing is different (+spiral side interval)
        d2 = nav.get_point_on_vector(
            a, d, config.SPIRAL_SIDES_INTERVAL + config.MANEUVER_START_DISTANCE)
        a_spiral = nav.get_point_on_vector(a, d, config.SPIRAL_SIDES_INTERVAL)
        # a1_spiral point is inside the initial field, corner of D-A_spiral-A1_spiral = 90 degrees
        a1_spiral = nav.get_coordinate(
            a_spiral, d, 90, config.MANEUVER_START_DISTANCE)

        a_corner_bezier = compute_bezier_points(d2, a_spiral, a1_spiral)
        if not add_points_to_path(path, *map(lambda gps_point: [gps_point, SI_speed_fwd], a_corner_bezier)):
            raise RuntimeError(
                "Failed to add A corner's bezier curve to path. This expected never to happen.")

        # get A'B'C'D' (intermediate points used to compute new ABCD points for next iteration)
        # (int points are requiring given vector length >= spiral_sides_interval * 2
        # it is very small value and can be exceeded only if robot can turn almost inplace)
        b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
        d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)
        if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
            msg = "Some of intermediate points [B1_int B2_int D1_int D2_int] for next spiral generation are None. " \
                  "This could happen if spiral shift value is higher than robot's maneuverability. " \
                  "Check config.MANEUVER_START_DISTANCE and config.SPIRAL_SIDES_INTERVAL for wrong values."
            raise RuntimeError(msg)

        a_new, b_new = compute_x1_x2_int_points(d2_int, b1_int, nav, logger)
        c_new, d_new = compute_x1_x2_int_points(b2_int, d1_int, nav, logger)
        if not check_points_for_nones(a_new, b_new, c_new, d_new):
            msg = "Some of points [A_new B_new C_new D_new] for next spiral generation iteration are None. " \
                  "This could happen if spiral shift value is higher than robot's maneuverability. " \
                  "Check config.MANEUVER_START_DISTANCE and config.SPIRAL_SIDES_INTERVAL for wrong values."
            raise RuntimeError(msg)

        a, b, c, d = a_new, b_new, c_new, d_new

    if config.ADD_FORWARD_BACKWARD_TO_END_PATH:
        if center_fill_start_point == 0:
            msg = "Asked to fill field's center during path building, but filling start position point flag was not " \
                  "changed from it's initial value."
            raise RuntimeError(msg)
        elif center_fill_start_point == 1:  # when robot is going to stop spiral movement at point A'n
            path = build_forward_backward_path(
                [a, b, c, d],
                nav,
                logger,
                SI_speed_fwd,
                SI_speed_rev,
                path)
        elif center_fill_start_point == 2:  # when robot is going to stop spiral movement at point D'n
            path = build_forward_backward_path(
                [d, a, b, c],
                nav,
                logger,
                SI_speed_fwd,
                SI_speed_rev,
                path)
        else:
            msg = "Asked to fill field's center during path building, but filling start position point flag value " \
                  "is not supported."
            raise NotImplementedError(msg)

    return path


def build_path(abcd_points: list, nav: navigation.GPSComputing, logger: utility.Logger, SI_speed_fwd: float, SI_speed_rev: float):
    raise NotImplementedError(
        "an obsolete code, use more advanced path builders like bezier or zigzag")

    path = []
    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    # get moving points A1 - ... - D2 spiral
    a1, a2 = compute_x1_x2_points(a, b, nav, logger)
    b1, b2 = compute_x1_x2_points(b, c, nav, logger)
    c1, c2 = compute_x1_x2_points(c, d, nav, logger)
    d1, d2 = compute_x1_x2_points(d, a, nav, logger)
    d2_spiral = compute_x2_spiral(d, a, nav, logger)

    # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
    if not add_points_to_path(path, [a, SI_speed_fwd], [a1, SI_speed_fwd], [a2, SI_speed_fwd], [b1, SI_speed_fwd], [b2, SI_speed_fwd], [c1, SI_speed_fwd], [c2, SI_speed_fwd], [d1, SI_speed_fwd], [d2_spiral, SI_speed_fwd]):
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
        if not add_points_to_path(path, [a1, SI_speed_fwd], [a2, SI_speed_fwd], [b1, SI_speed_fwd], [b2, SI_speed_fwd], [c1, SI_speed_fwd], [c2, SI_speed_fwd], [d1, SI_speed_fwd], [d2_spiral, SI_speed_fwd]):
            break

        a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int

    if nav.get_distance(a, b) >= nav.get_distance(b, c):
        return add_forward_backward_path([a, b, c, d], nav, logger, SI_speed_fwd, SI_speed_rev, path)
    else:
        return add_forward_backward_path([c, b, a, d], nav, logger, SI_speed_rev, SI_speed_fwd, path)


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


def emergency_field_defining(vesc_engine: adapters.VescAdapterV4, gps: adapters.GPSUbloxAdapter,
                             nav: navigation.GPSComputing, cur_log_dir, logger_full: utility.Logger):
    msg = "Using emergency field creation..."
    logger_full.write(msg + "\n")
    print(msg)

    starting_point = gps.get_last_position()

    msg = "Moving forward..."
    logger_full.write(msg + "\n")
    print(msg)
    vesc_engine.set_time_to_move(float("inf"), vesc_engine.PROPULSION_KEY)
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
    save_gps_coordinates_raw(
        [starting_point, A, B, C, D], cur_log_dir + "emergency_raw_field.txt")
    return field


def send_name_of_file_of_gps_history(ui_msg_queue: posix_ipc.MessageQueue,
                                     gps_file_dir: str,
                                     gps_file_name: str,
                                     logger_full: utility.Logger):
    """Send name of file if it exists using given message queue
    """

    if os.path.isfile(gps_file_dir + gps_file_name):
        ui_msg_queue.send(json.dumps(
            {"last_gps_list_file": gps_file_dir + gps_file_name}))
    else:
        msg = f"Could not find {gps_file_dir}/used_gps_history.txt file to send previous points to the web UI"
        logger_full.write(msg + "\n")
        if config.VERBOSE:
            print(msg)


def get_bezier_indexes(path_points: list):
    """ONLY for bezier path with NO backward-forward at corners. Presence of zigzags at field center is ok.

    Returns list of bezier points indexes."""

    if len(path_points) < 2:
        raise ValueError(f"path_points must contain at least 2 points, got {len(path_points)} instead")

    last_non_zigzag_idx = 0
    for i in range(len(path_points)):
        if math.isclose(path_points[i][1], 0):
            raise ValueError(f"POINT {i} HAS 0 SPEED!")
        elif path_points[i][1] > 0:
            last_non_zigzag_idx = i
        else:
            break

    a_non_bezier_indexes = [0]  # A points
    b_non_bezier_indexes = [1]  # B points
    while last_non_zigzag_idx > b_non_bezier_indexes[-1]:
        a_non_bezier_indexes.append(a_non_bezier_indexes[-1] + config.NUMBER_OF_BEZIER_POINT)
        b_non_bezier_indexes.append(b_non_bezier_indexes[-1] + config.NUMBER_OF_BEZIER_POINT)

    non_bezier_indexes = a_non_bezier_indexes + b_non_bezier_indexes
    bezier_indexes = []
    for i in range(last_non_zigzag_idx + 1):
        if i not in non_bezier_indexes:
            bezier_indexes.append(i)

    return bezier_indexes


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

    utility.create_directories(
        log_cur_dir, config.DEBUG_IMAGES_PATH, config.DATA_GATHERING_DIR)

    try:
        if config.QUEUE_MESSAGES_MAX is not None:
            ui_msg_queue = posix_ipc.MessageQueue(
                config.QUEUE_NAME_UI_MAIN, max_messages=config.QUEUE_MESSAGES_MAX)
        else:
            ui_msg_queue = posix_ipc.MessageQueue(config.QUEUE_NAME_UI_MAIN)
    except KeyboardInterrupt:
        raise KeyboardInterrupt
    except:
        ui_msg_queue = None

    image_saver = utility.ImageSaver()
    if config.ALLOW_GATHERING:
        image_saver.set_counter(
            len(glob.glob(config.DATA_GATHERING_DIR + "*.jpg")), "gathering")

    notification = NotificationClient(time_start)
    notification.set_robot_state(RobotSynthesis.WORKING)
    data_collector = datacollection.DataCollector(
        log_cur_dir + config.STATISTICS_DB_FILE_NAME,
        notification,
        load_from_file=config.CONTINUE_PREVIOUS_PATH,
        file_path=log_cur_dir + config.DATACOLLECTOR_SAVE_FILE,
        ui_msg_queue=ui_msg_queue,
        dump_at_receiving=True)
    working_zone_polygon = Polygon(config.WORKING_ZONE_POLY_POINTS)
    nav = navigation.GPSComputing()
    logger_full = utility.Logger(
        log_cur_dir + "log full.txt", append_file=config.CONTINUE_PREVIOUS_PATH)

    # X axis movement during periphery scans config settings validation
    if config.ALLOW_X_MOVEMENT_DURING_SCANS:
        if len(config.X_MOVEMENT_CAMERA_POSITIONS) != len(config.X_MOVEMENT_CAMERA_X_F) != \
                len(config.X_MOVEMENT_IMAGE_ZONES) or len(config.X_MOVEMENT_CAMERA_POSITIONS) in [0, 1]:
            msg = "Disabling X axis movement during scans as lengths of positions/forces/image areas are not equal " \
                  "or there's less than 2 elements in list of positions/forces/image areas!"
            logger_full.write(msg + "\n")
            print(msg)
            config.ALLOW_X_MOVEMENT_DURING_SCANS = False
    if config.ALLOW_X_MOVEMENT_DURING_SCANS:
        x_scan_poly = ExtractionManagerV3.pdz_dist_to_poly(
            config.X_MOVEMENT_IMAGE_ZONES)
    else:
        x_scan_poly = []

    # get smoothie and vesc addresses
    smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
    if "vesc" in smoothie_vesc_addr:
        vesc_address = smoothie_vesc_addr["vesc"]
    else:
        msg = "Couldn't get vesc's USB address!"
        print(msg)
        logger_full.write(msg + "\n")
        notification.set_robot_state(RobotSynthesis.HS)
        exit()
    if config.SMOOTHIE_BACKEND == 1:
        smoothie_address = config.SMOOTHIE_HOST
    else:
        if "smoothie" in smoothie_vesc_addr:
            smoothie_address = smoothie_vesc_addr["smoothie"]
        else:
            msg = "Couldn't get smoothie's USB address!"
            print(msg)
            logger_full.write(msg + "\n")
            notification.set_robot_state(RobotSynthesis.HS)
            exit()

    # load yolo networks
    if config.NN_MODELS_COUNT < 1:
        msg = f"Key 'config.NN_MODELS_COUNT' has 0 or negative value which is wrong as need at least 1 model for work"
        print(msg)
        logger_full.write(msg + "\n")
        exit()

    # load periphery NN
    msg = "Loading periphery detector..."
    print(msg)
    logger_full.write(msg + "\n")
    if config.PERIPHERY_WRAPPER == 1:
        periphery_detector = detection.YoloTRTDetector(
            config.PERIPHERY_MODEL_PATH,
            config.PERIPHERY_CLASSES_FILE,
            config.PERIPHERY_CONFIDENCE_THRESHOLD,
            config.PERIPHERY_NMS_THRESHOLD,
            config.PERIPHERY_INPUT_SIZE)
    elif config.PERIPHERY_WRAPPER == 2:
        periphery_detector = detection.YoloOpenCVDetection(
            config.PERIPHERY_CLASSES_FILE,
            config.PERIPHERY_CONFIG_FILE,
            config.PERIPHERY_WEIGHTS_FILE,
            config.PERIPHERY_INPUT_SIZE,
            config.PERIPHERY_CONFIDENCE_THRESHOLD,
            config.PERIPHERY_NMS_THRESHOLD,
            config.PERIPHERY_DNN_BACKEND,
            config.PERIPHERY_DNN_TARGET)
    else:
        msg = "Wrong config.PERIPHERY_WRAPPER = " + \
            str(config.PERIPHERY_WRAPPER) + " code. Exiting."
        logger_full.write(msg + "\n")
        notification.set_robot_state(RobotSynthesis.HS)
        exit()

    # load precise NN
    if config.NN_MODELS_COUNT > 1:
        msg = "Loading precise detector..."
        print(msg)
        logger_full.write(msg + "\n")
        if config.PRECISE_WRAPPER == 1:
            precise_detector = detection.YoloTRTDetector(
                config.PRECISE_MODEL_PATH,
                config.PRECISE_CLASSES_FILE,
                config.PRECISE_CONFIDENCE_THRESHOLD,
                config.PRECISE_NMS_THRESHOLD,
                config.PRECISE_INPUT_SIZE)
        elif config.PRECISE_WRAPPER == 2:
            precise_detector = detection.YoloOpenCVDetection(
                config.PRECISE_CLASSES_FILE,
                config.PRECISE_CONFIG_FILE,
                config.PRECISE_WEIGHTS_FILE,
                config.PRECISE_INPUT_SIZE,
                config.PRECISE_CONFIDENCE_THRESHOLD,
                config.PRECISE_NMS_THRESHOLD,
                config.PRECISE_DNN_BACKEND,
                config.PRECISE_DNN_TARGET)
        else:
            msg = "Wrong config.PRECISE_WRAPPER = " + \
                str(config.PRECISE_WRAPPER) + " code. Exiting."
            logger_full.write(msg + "\n")
            notification.set_robot_state(RobotSynthesis.HS)
            exit()
    else:
        msg = "Using periphery detector as precise."
        print(msg)
        logger_full.write(msg + "\n")
        precise_detector = periphery_detector

    if config.CONTINUOUS_INFORMATION_SENDING:
        treated_plants = set()
        treated_plants.update(periphery_detector.get_classes_names())
        treated_plants.update(precise_detector.get_classes_names())
        notification.set_treated_weed_types(treated_plants)

    # load and send trajectory to the UI if continuing work
    if config.CONTINUE_PREVIOUS_PATH:
        if ui_msg_queue is not None:
            send_name_of_file_of_gps_history(
                ui_msg_queue, log_cur_dir, "used_gps_history.txt", logger_full)
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
            adapters.VescAdapterV4(vesc_address, config.VESC_BAUDRATE, config.VESC_ALIVE_FREQ, config.VESC_CHECK_FREQ,
                                   config.VESC_STOPPER_CHECK_FREQ, logger_full) as vesc_engine, \
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
            navigation.NavigationPrediction(
                logger_full=logger_full,
                nav=nav,
                log_cur_dir=log_cur_dir) as navigation_prediction:


            send_voltage_thread_alive = {"value": True}
            send_voltage_thread = threading.Thread(
                target=send_voltage_thread_tf,
                args=(lambda: send_voltage_thread_alive["value"],
                      vesc_engine,
                      logger_full, 
                      ui_msg_queue),
                daemon=True)
            send_voltage_thread.start()

            # try to load field ABCD points
            field_gps_coords = None
            field_name = None
            if config.USE_EMERGENCY_FIELD_GENERATION and not config.CONTINUE_PREVIOUS_PATH:
                field_gps_coords = emergency_field_defining(vesc_engine, gps, nav, log_cur_dir, logger_full)
            else:
                # check if shortcut exists
                if os.path.isfile(config.INPUT_GPS_FIELD_FILE):
                    # check if shortcut target file exists
                    # old way: shortcut_target_path = subprocess.check_output(['readlink', '-f', config.INPUT_GPS_FIELD_FILE]).decode("utf-8").strip()
                    shortcut_target_path = os.path.realpath(config.INPUT_GPS_FIELD_FILE)
                    field_name = (shortcut_target_path.split("/")[-1]).split(".")[0]
                    if os.path.isfile(shortcut_target_path):
                        msg = f"Loading '{config.INPUT_GPS_FIELD_FILE}' field file"
                        logger_full.write(msg + "\n")

                        try:
                            field_gps_coords = utility.load_coordinates(config.INPUT_GPS_FIELD_FILE)  # [A, B, C, D]
                        except ValueError:
                            msg = f"Failed to load field '{shortcut_target_path}' due " \
                                  f"to ValueError (file is likely corrupted)"
                            print(msg)
                            logger_full.write(msg + "\n")

                        msg = f"Loaded field: {str(field_gps_coords)}"
                        print(msg)
                        logger_full.write(msg + "\n")
                    else:
                        msg = f"Couldn't find '{os.path.realpath(config.INPUT_GPS_FIELD_FILE)}' target file with" \
                              f"field points"
                        print(msg)
                        logger_full.write(msg + "\n")
                else:
                    msg = f"Couldn't find '{config.INPUT_GPS_FIELD_FILE}' shortcut file with field points"
                    print(msg)
                    logger_full.write(msg + "\n")

            # set field to notification
            if config.CONTINUOUS_INFORMATION_SENDING:
                if field_gps_coords is None:
                    msg = f"Sending field to notification is aborted as field is None"
                    print(msg)
                    logger_full.write(msg)
                elif field_name is None:
                    msg = f"Sending field to notification is aborted as field name is None"
                    print(msg)
                    logger_full.write(msg)
                elif len(field_gps_coords) < 1:
                    msg = f"Loaded '{shortcut_target_path}' field contains 0" \
                          f"points. Sending to notification is aborted."
                    print(msg)
                    logger_full.write(msg + "\n")
                else:
                    notification.set_field(field_gps_coords.copy(), field_name)

            # continue previous path case
            loading_previous_path_failed = False
            loading_previous_index_failed = False
            if config.CONTINUE_PREVIOUS_PATH:
                # TODO: maybe make a path manager and put field and path loading and checking etc there

                msg = "Loading previous path points"
                logger_full.write(msg + "\n")

                if not os.path.isfile(config.PREVIOUS_PATH_POINTS_FILE):
                    loading_previous_path_failed = True
                    msg = f"Couldn't find '{config.PREVIOUS_PATH_POINTS_FILE}' file with previous path points. " \
                          f"Trying to generate path points of current field from scratch."
                    print(msg)
                    logger_full.write(msg + "\n")
                else:
                    # data validation is done later as it needed both to loaded and new generated path
                    with open(config.PREVIOUS_PATH_POINTS_FILE, "rb") as path_points_file:
                        path_points = pickle.load(path_points_file)

                    if not os.path.isfile(config.PREVIOUS_PATH_INDEX_FILE):
                        loading_previous_index_failed = True
                        msg = f"Couldn't find '{config.PREVIOUS_PATH_INDEX_FILE}' file with point index to continue."
                        print(msg)
                        logger_full.write(msg + "\n")
                    else:
                        with open(config.PREVIOUS_PATH_INDEX_FILE, "r+") as path_index_file:
                            str_index = path_index_file.readline().strip()
                        try:
                            path_start_index = int(str_index)
                        except ValueError:
                            loading_previous_index_failed = True
                            msg = f"Couldn't convert path point index '{str_index}' into int."
                            print(msg)
                            logger_full.write(msg + "\n")

                    if path_start_index == -1:
                        msg = "Previous path is already passed"
                        print(msg)
                        logger_full.write_and_flush(msg + "\n")
                        notification.close()
                        exit()
                    elif path_start_index >= len(path_points) or path_start_index < 1:
                        loading_previous_index_failed = True
                        msg = f"Path start index {path_start_index} is out of path points list range (loaded " \
                              f"{len(path_points)} points)"
                        print(msg)
                        logger_full.write(msg + "\n")

                    if loading_previous_index_failed:
                        msg = "Creating new path index storage file, and going to start this field from 1rst point " \
                              "due to index loading troubles (see log above for details)"
                        print(msg)
                        logger_full.write(msg + "\n")
                        path_start_index = 1
                        with open(config.PREVIOUS_PATH_INDEX_FILE, "w") as path_index_file:
                            path_index_file.write(str(path_start_index))

            # load field points and generate new path or continue previous path errors case
            if not config.CONTINUE_PREVIOUS_PATH or loading_previous_path_failed:
                if field_gps_coords is None:
                    msg = f"Exiting main as building path without field points is impossible"
                    print(msg)
                    logger_full.write_and_flush(msg)
                    notification.set_robot_state(RobotSynthesis.HS)
                    exit()

                # check field corner points count
                if len(field_gps_coords) == 4:
                    field_gps_coords = reduce_field_size(
                        field_gps_coords, config.FIELD_REDUCE_SIZE, nav)

                    msg = "Reduced field: " + str(field_gps_coords)
                    print(msg)
                    logger_full.write(msg + "\n")

                    # generate path points
                    path_start_index = 1
                    if config.TRADITIONAL_PATH:
                        path_points = build_path(
                            field_gps_coords,
                            nav,
                            logger_full,
                            config.SI_SPEED_FWD,
                            config.SI_SPEED_REV)
                    elif config.BEZIER_PATH:
                        path_points = build_bezier_path(
                            field_gps_coords,
                            nav,
                            logger_full,
                            config.SI_SPEED_FWD,
                            config.SI_SPEED_REV)
                    elif config.FORWARD_BACKWARD_PATH:
                        path_points = build_forward_backward_path(
                            field_gps_coords,
                            nav,
                            logger_full,
                            config.SI_SPEED_FWD,
                            config.SI_SPEED_REV)

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
                    notification.set_robot_state(RobotSynthesis.HS)
                    exit()

                # save path points and point to start from index
                with open(config.PREVIOUS_PATH_POINTS_FILE, "wb") as path_points_file:
                    pickle.dump(path_points, path_points_file)
                with open(config.PREVIOUS_PATH_INDEX_FILE, "w") as path_index_file:
                    path_index_file.write(str(path_start_index))

            if len(path_points) > 0:
                save_gps_coordinates(
                    path_points, log_cur_dir + "current_path_points.txt")
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
                notification.set_robot_state(RobotSynthesis.HS)
                exit()

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

            msg = 'GpsQ|Raw ang|Res ang|Ord ang|Sum ang|Distance    |Adapter|Smoothie|PointStatus|deviation|side dev|' \
                  'centroid factor|cruise factor'
            print(msg)
            logger_full.write(msg + "\n")

            # path points visiting loop
            with open(
                    config.PREVIOUS_PATH_INDEX_FILE,
                    "r+" if os.path.isfile(config.PREVIOUS_PATH_INDEX_FILE) else "w") as path_index_file:
                # TODO: temp. wheels mechanics hotfix. please don't repeat things I did here they are not good.
                if config.ENABLE_ADDITIONAL_WHEELS_TURN:
                    if config.TRADITIONAL_PATH:
                        msg = f"wheels additional turn is enabled (cur. config.ENABLE_ADDITIONAL_WHEELS_TURN=True), " \
                              f"and it's not compatible with traditional path (cur. config.TRADITIONAL_PATH=True)"
                        raise RuntimeError(msg)
                    if not config.BEZIER_PATH:
                        msg = f"wheels additional turn is enabled, and it's compatible only with bezier path " \
                              f"(cur. config.BEZIER_PATH=False)"
                        raise RuntimeError(msg)
                    if config.FORWARD_BACKWARD_PATH:
                        msg = f"wheels additional turn is enabled, and it's not compatible with forward-backward " \
                              f"path building option (cur. config.FORWARD_BACKWARD_PATH=True)"
                        raise RuntimeError(msg)
                    if config.ADD_CORNER_TO_BEZIER_PATH:
                        msg = f"wheels additional turn is enabled, and it's not compatible with " \
                              f"ADD_CORNER_TO_BEZIER_PATH building path option " \
                              f"(cur. config.ADD_CORNER_TO_BEZIER_PATH=True)"
                        raise RuntimeError(msg)

                    smoothie_tel_conn = None
                    try:
                        bezier_points_indexes = get_bezier_indexes(path_points)
                        smoothie_tel_conn = connectors.SmoothieV11TelnetConnector(config.SMOOTHIE_TELNET_HOST)
                        smoothie_tel_conn.write("G91")
                        res = smoothie_tel_conn.read_some()
                        if res != smoothie.RESPONSE_OK:
                            msg = f"Couldn't set smoothie telnet connection to relative mode:\n{res}\n" \
                                  f"Telnet connection usage will be disabled."
                            print(msg)
                            logger_full.write(msg + "\n")
                            smoothie_tel_conn = None
                    except KeyboardInterrupt:
                        raise KeyboardInterrupt
                    except:
                        msg = f"Wheels additional turn preparations are failed:\n" \
                              f"{traceback.format_exc()}\n" \
                              f"Telnet connection usage will be disabled."
                        print(msg)
                        logger_full.write(msg + "\n")
                        smoothie_tel_conn = None

                next_calibration_time = time.time() + config.CORK_CALIBRATION_MIN_TIME

                try:
                    start_position = utility.average_point(
                        gps, trajectory_saver, nav)
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except:
                    pass

                if ui_msg_queue is not None:
                    try:
                        ui_msg_queue.send(json.dumps({"start": True}))
                    except KeyboardInterrupt:
                        raise KeyboardInterrupt
                    except:
                        pass

                last_direction_of_travel = None  # 1 -> moving forward #-1 -> moving backward

                if config.NAVIGATION_TEST_MODE:
                    path_end_index = sys.maxsize
                else:
                    path_end_index = len(path_points)

                # applies improved approaching to path during continuing previous field job
                if config.CONTINUE_PREVIOUS_PATH and config.USE_SMOOTH_APPROACHING_TO_FIELD:
                    smooth_preparation_ok = True

                    if config.TRADITIONAL_PATH:
                        smooth_preparation_ok = False
                        msg = "USING RUDE TRAJECTORY APPROACHING as traditional path is not supported by smooth " \
                              "approaching feature."
                        logger_full.write(msg + "\n")
                    elif config.BEZIER_PATH:
                        traj_path_start_index = path_start_index
                        cur_pos = gps.get_fresh_position()

                        # speed 0 is abnormal TODO what robot should do in this case?
                        if math.isclose(path_points[traj_path_start_index][1], 0):
                            smooth_preparation_ok = False
                            msg = f"USING RUDE TRAJECTORY APPROACHING as point (path_points[{traj_path_start_index}])" \
                                  f" speed is 0 (look at generated path points, speed 0 is not ok)."
                            logger_full.write(msg + "\n")
                        # zigzag (speed < 0)
                        elif path_points[traj_path_start_index][1] < 0:
                            # skip negative speed points until positive speed or path end
                            log_traj_start_idx = traj_path_start_index
                            traj_path_start_index += 1
                            while traj_path_start_index < len(path_points):
                                if path_points[traj_path_start_index][1] > 0 and not math.isclose(
                                        path_points[traj_path_start_index][1], 0):
                                    break
                                traj_path_start_index += 1
                            else:
                                smooth_preparation_ok = False
                                msg = f"USING RUDE TRAJECTORY APPROACHING as couldn't find any points " \
                                      f"(from {log_traj_start_idx} to {traj_path_start_index}) with positive speed " \
                                      f"to continue zigzag."
                                logger_full.write(msg + "\n")
                        # spiral or zigzag (speed > 0)
                        else:
                            # prev point speed 0 is abnormal TODO what robot should do in this case?
                            if math.isclose(path_points[traj_path_start_index - 1][1], 0):
                                smooth_preparation_ok = False
                                msg = f"USING RUDE TRAJECTORY APPROACHING as point (path_points[" \
                                      f"{traj_path_start_index - 1}]) speed is 0 (look at generated path points, " \
                                      f"speed 0 is not ok)."
                                logger_full.write(msg + "\n")
                            # spiral
                            elif path_points[traj_path_start_index - 1][1] > 0:
                                # generate non bezier indexes
                                a_non_bezier_indexes = [0]  # A points
                                b_non_bezier_indexes = [1]  # B points
                                while traj_path_start_index > b_non_bezier_indexes[-1]:
                                    a_non_bezier_indexes.append(
                                        a_non_bezier_indexes[-1] + config.NUMBER_OF_BEZIER_POINT)
                                    b_non_bezier_indexes.append(
                                        b_non_bezier_indexes[-1] + config.NUMBER_OF_BEZIER_POINT)

                                # look for index of point which will be used to get the robot to trajectory
                                # ABAP correct angle: -180 to -90 or 170 to 180
                                # BAAP correct angle: -10 to 90 (using this)
                                for point_a_idx, point_b_idx in zip(reversed(a_non_bezier_indexes),
                                                                    reversed(b_non_bezier_indexes)):
                                    angle = nav.get_angle(
                                        path_points[point_b_idx][0],
                                        path_points[point_a_idx][0],
                                        path_points[point_a_idx][0],
                                        cur_pos)
                                    if -10 <= angle <= 90:
                                        traj_path_start_index = point_a_idx
                                        break
                                else:
                                    smooth_preparation_ok = False
                                    msg = "USING RUDE TRAJECTORY APPROACHING as couldn't find any previous point " \
                                          "with a satisfactory angle to get on trajectory."
                                    logger_full.write(msg + "\n")
                            # zigzag (prev point speed is negative)
                            else:
                                # skip negative speed points until positive speed or path end
                                traj_path_start_index += 1
                                while traj_path_start_index < len(path_points):
                                    if path_points[traj_path_start_index][1] > 0 and not math.isclose(
                                            path_points[traj_path_start_index][1], 0):
                                        break
                                    traj_path_start_index += 1
                                else:
                                    smooth_preparation_ok = False
                                    msg = "USING RUDE TRAJECTORY APPROACHING as couldn't find any further points " \
                                          "with positive speed to continue zigzag job."
                                    logger_full.write(msg + "\n")

                        # check if robot tries to skip or revert too many path points
                        if abs(traj_path_start_index - path_start_index) > config.SMOOTH_APPROACHING_MAX_POINTS:
                            smooth_preparation_ok = False
                            msg = f"Robot wants to visit too many previous points or skip too many job points; " \
                                  f"traj_start_idx={traj_path_start_index}; path_start_idx={path_start_index}, exiting."
                            logger_full.write(msg + "\n")

                        # approach trajectory smoothly if preparation was successful
                        if smooth_preparation_ok:
                            # for future points (nav predictor)
                            i_inf = traj_path_start_index + 1 if traj_path_start_index + 1 < path_end_index \
                                else path_end_index
                            i_sup = traj_path_start_index + 1 + config.FUTURE_NUMBER_OF_POINTS \
                                if traj_path_start_index + config.FUTURE_NUMBER_OF_POINTS < path_end_index \
                                else path_end_index

                            # if smooth approach target point is or further than job start point
                            # (move to this point and start job)
                            move_to_point_and_extract(
                                [cur_pos, path_points[traj_path_start_index][0]],
                                gps,
                                vesc_engine,
                                smoothie,
                                camera,
                                periphery_detector,
                                precise_detector,
                                logger_full,
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
                                path_points[traj_path_start_index][1],
                                False,
                                navigation_prediction,
                                path_points[i_inf:i_sup],
                                not config.FIRST_POINT_NO_EXTRACTIONS,
                                x_scan_poly,
                                field_gps_coords
                            )
                            if traj_path_start_index >= path_start_index:
                                path_start_index = traj_path_start_index + 1
                            else:
                                for i in range(traj_path_start_index + 1, path_start_index - 1):
                                    i_inf = i + 1 if i + 1 < path_end_index else path_end_index
                                    i_sup = i + 1 + config.FUTURE_NUMBER_OF_POINTS \
                                        if i + config.FUTURE_NUMBER_OF_POINTS < path_end_index else path_end_index

                                    move_to_point_and_extract(
                                        [path_points[i - 1][0], path_points[i][0]],
                                        gps,
                                        vesc_engine,
                                        smoothie,
                                        camera,
                                        periphery_detector,
                                        precise_detector,
                                        logger_full,
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
                                        path_points[i][1],
                                        False,
                                        navigation_prediction,
                                        path_points[i_inf:i_sup],
                                        not config.FIRST_POINT_NO_EXTRACTIONS,
                                        x_scan_poly,
                                        field_gps_coords
                                    )
                    elif config.FORWARD_BACKWARD_PATH:
                        msg = "USING RUDE TRAJECTORY APPROACHING as forward-backward path is not supported yet by " \
                              "smooth approaching feature."
                        logger_full.write(msg + "\n")
                    else:
                        msg = "USING RUDE TRAJECTORY APPROACHING as all known/supported path generation modes are " \
                              "disabled, probably config path generation settings are corrupted"
                        logger_full.write(msg + "\n")

                # move through path points
                for i in range(path_start_index, path_end_index):

                    if config.NAVIGATION_TEST_MODE:
                        dist_here_point_a = nav.get_distance(
                            start_position, config.POINT_A[0])
                        dist_here_point_b = nav.get_distance(
                            start_position, config.POINT_B[0])

                        if dist_here_point_a > dist_here_point_b:
                            from_to = [config.POINT_B[0], config.POINT_A[0]]
                            speed = config.POINT_A[1]
                        else:
                            from_to = [config.POINT_A[0], config.POINT_B[0]]
                            speed = config.POINT_B[1]

                        display_instruction_path = from_to[0:2]

                    else:
                        from_to = [path_points[i - 1][0], path_points[i][0]]
                        speed = path_points[i][1]

                        i_inf = i-config.DELTA_DISPLAY_INSTRUCTION_PATH if i >= config.DELTA_DISPLAY_INSTRUCTION_PATH else 0
                        i_sup = i+config.DELTA_DISPLAY_INSTRUCTION_PATH if i + \
                            config.DELTA_DISPLAY_INSTRUCTION_PATH < path_end_index else path_end_index-1
                        display_instruction_path = [elem[0]
                                                    for elem in path_points[i_inf:i_sup]]

                    if ui_msg_queue is not None and config.DISPLAY_INSTRUCTION_PATH:
                        ui_msg_queue.send(json.dumps(
                            {"display_instruction_path": display_instruction_path}))

                    if last_direction_of_travel is None:
                        # 1 -> moving forward #-1 -> moving backward
                        last_direction_of_travel = (speed >= 0) if 1 else -1

                    # 1 -> moving forward #-1 -> moving backward
                    direction_of_travel = (speed >= 0) if 1 else -1

                    if direction_of_travel != last_direction_of_travel:
                        vesc_engine.set_target_rpm(
                            speed * config.MULTIPLIER_SI_SPEED_TO_RPM, vesc_engine.PROPULSION_KEY)

                    if config.WHEELS_STRAIGHT_CHANGE_DIRECTION_OF_TRAVEL and direction_of_travel != last_direction_of_travel:
                        vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY)
                        response = smoothie.custom_move_to(
                            A_F=config.A_F_MAX, A=0)
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
                        if config.FIRST_POINT_NO_EXTRACTIONS and config.CONTINUE_PREVIOUS_PATH and
                        not config.USE_SMOOTH_APPROACHING_TO_FIELD else True,
                        x_scan_poly,
                        field_gps_coords
                    )

                    if config.ENABLE_ADDITIONAL_WHEELS_TURN and \
                            i - 1 in bezier_points_indexes and i in bezier_points_indexes:
                        cur_pos = gps.get_last_position()
                        if cur_pos[2] != "4":
                            msg = f"Additional wheels turn got point {cur_pos} with non 4 quality - " \
                                  f"skipping wheels turn actions"
                            logger_full.write(msg + "\n")
                        else:
                            deviation, side = nav.get_deviation(path_points[i-1][0], path_points[i][0], cur_pos)
                            if deviation > config.ADDITIONAL_WHEELS_TURN_THRESHOLD and side == -1:
                                msg = f"Wheels turn deviation threshold hit, trying to turn wheels"
                                logger_full.write(msg + "\n")

                                if smoothie_tel_conn is not None:
                                    smoothie_tel_conn.write(
                                        f"G0 {config.ADDITIONAL_WHEELS_KEY}"
                                        f"{config.ADDITIONAL_WHEELS_VALUE} "
                                        f"F{config.ADDITIONAL_WHEELS_FORCE}")
                                    res = smoothie_tel_conn.read_some()
                                    if res != smoothie.RESPONSE_OK:
                                        msg = f"Couldn't do additional wheels turn, smoothie response:\n{res}"
                                        logger_full.write(msg + "\n")
                                else:
                                    msg = f"Couldn't turn wheels as smoothie telnet connector is None"
                                    logger_full.write(msg + "\n")

                    if config.NAVIGATION_TEST_MODE:
                        response = smoothie.custom_move_to(
                            A_F=config.A_F_MAX, A=0)
                        if response != smoothie.RESPONSE_OK:  # TODO: what if response is not ok?
                            msg = "Couldn't turn wheels before other navigation test, smoothie response:\n" + response
                            print(msg)
                            logger_full.write(msg + "\n")
                        else:
                            with open(config.LAST_ANGLE_WHEELS_FILE, "w+") as wheels_angle_file:
                                wheels_angle_file.write(
                                    str(smoothie.get_adapter_current_coordinates()["A"]))
                        test_continue = input(
                            "Press enter to continue the test, type anything to exit.")
                        if test_continue != "":
                            notification.close()
                            break
                        try:
                            start_position = utility.average_point(
                                gps, trajectory_saver, nav)
                        except KeyboardInterrupt:
                            raise KeyboardInterrupt
                        except:
                            pass
                        if ui_msg_queue is not None:
                            ui_msg_queue.send(json.dumps({"clear_path": True}))
                        # reload config if kp or ki change
                        importlib.reload(config)

                    # 1 -> moving forward #-1 -> moving backward
                    last_direction_of_travel = (speed >= 0) if 1 else -1

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
            notification.close()
    except KeyboardInterrupt:
        msg = "Stopped by a keyboard interrupt (Ctrl + C)\n" + \
            traceback.format_exc()
        print(msg)
        logger_full.write(msg + "\n")
        notification.close()
        if ui_msg_queue is not None:
            ui_msg_queue.send(json.dumps({"stopping": True}))
        if ui_msg_queue is not None:
            ui_msg_queue.close()
    except Exception as e:
        if "LEAVING_FIELD" not in e.args:
            notification.set_robot_state_and_wait_send(RobotSynthesis.HS)
            msg = "Exception occurred:\n" + traceback.format_exc()
            print(msg)
            logger_full.write(msg + "\n")
        if ui_msg_queue is not None:
            ui_msg_queue.close()
    finally:

        send_voltage_thread_alive["value"] = False
        if send_voltage_thread is not None:
            send_voltage_thread.join()

        # put the wheel straight
        # vesc z axis calibration in case it's used for z axis control instead of smoothie
        # (to prevent cork breaking when smoothie calibrates)
        smoothie_safe_calibration = True
        if config.EXTRACTION_CONTROLLER == 2:
            with adapters.VescAdapterV4(
                    vesc_address,
                    config.VESC_BAUDRATE,
                    config.VESC_ALIVE_FREQ,
                    config.VESC_CHECK_FREQ,
                    config.VESC_STOPPER_CHECK_FREQ) as vesc_engine:
                # Z-5 fix (move cork little down to stop touching stopper)
                vesc_engine.set_target_rpm(
                    config.VESC_EXTRACTION_CALIBRATION_Z5_FIX_RPM, vesc_engine.EXTRACTION_KEY)
                vesc_engine.set_time_to_move(
                    config.VESC_EXTRACTION_CALIBRATION_Z5_FIX_TIME,
                    vesc_engine.EXTRACTION_KEY)
                vesc_engine.start_moving(vesc_engine.EXTRACTION_KEY)
                vesc_engine.wait_for_stop(vesc_engine.EXTRACTION_KEY)

                # calibration
                vesc_engine.set_target_rpm(
                    config.VESC_EXTRACTION_CALIBRATION_RPM, vesc_engine.EXTRACTION_KEY)
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

        # put the robot's wheels straight
        if smoothie_safe_calibration:
            msg = f"Trying to put wheels straight before shutdown"
            logger_full.write(msg + "\n")

            if os.path.isfile(config.LAST_ANGLE_WHEELS_FILE):
                msg = f"Found '{config.LAST_ANGLE_WHEELS_FILE}' file, trying to read wheels smoothie position"
                logger_full.write(msg + "\n")

                with open(config.LAST_ANGLE_WHEELS_FILE, "r+") as wheels_angle_file:
                    line = wheels_angle_file.read().strip()
                    angle = None
                    try:
                        angle = float(line)
                        msg = f"Successfully loaded wheels smoothie position: {angle}"
                        logger_full.write(msg + "\n")
                    except ValueError:
                        msg = f"Couldn't convert '{line}' into float position, leaving wheels position as it is"
                        print(msg)
                        logger_full.write(msg + "\n")

                    if angle is not None:
                        with adapters.SmoothieAdapter(smoothie_address) as smoothie:
                            smoothie.set_current_coordinates(A=angle)
                            response = smoothie.custom_move_to(
                                A_F=config.A_F_MAX, A=0)
                            if response != smoothie.RESPONSE_OK:
                                msg = "Couldn't turn wheels before shutdown, smoothie response:\n" + response
                                print(msg)
                                logger_full.write(msg + "\n")
                            else:
                                wheels_angle_file.seek(0)
                                wheels_angle_file.write(
                                    str(smoothie.get_adapter_current_coordinates()["A"]))
            else:
                msg = f"Couldn't find '{config.LAST_ANGLE_WHEELS_FILE}' wheels smoothie position file.\n" \
                      f"Creating new file with 0.0 position containing. Robot may navigate wrong of wheels are turned."
                print(msg)
                logger_full.write(msg + "\n")
                with open(config.LAST_ANGLE_WHEELS_FILE, "w") as wheels_angle_file:
                    wheels_angle_file.write("0.0")

        # save adapter points history
        try:
            # TODO: reduce history positions to 1 to save RAM
            # TODO: gps adapter is blocking now if has no points
            adapter_points_history = gps.get_last_positions_list()
            if len(adapter_points_history) > 0:
                save_gps_coordinates(adapter_points_history,
                                     log_cur_dir + "adapter_gps_history.txt")
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
            data_collector.save_all_data(
                log_cur_dir + config.STATISTICS_OUTPUT_FILE)
            data_collector.dump_to_file(
                log_cur_dir + config.DATACOLLECTOR_SAVE_FILE)
        except:
            msg = "Failed to save txt statistics:\n" + traceback.format_exc()
            logger_full.write(msg + "\n")
            print(msg)
        try:
            data_collector.close()
        except:
            msg = "Failed to close properly DB:\n" + traceback.format_exc()
            logger_full.write(msg + "\n")
            print(msg)

        # close log and hardware connections
        msg = "Closing loggers..."
        logger_full.write_and_flush(msg + "\n")
        print(msg)
        logger_full.close()

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
    exit(0)
