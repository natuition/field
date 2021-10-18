"""Spiral movement, detection and extraction over given by ABCD points area"""

import os
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
from extraction_manager import ExtractionManager

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

def draw_zone_circle(image, circle_center_x, circle_center_y, circle_radius):
    """Draws received circle on image. Used for drawing undistorted zone edges on photo"""

    return cv.circle(image, (circle_center_x, circle_center_y), circle_radius, (0, 0, 255), thickness=3)


def draw_zone_poly(image, np_poly_points):
    """Draws received polygon on image. Used for drawing working zone edges on photo"""

    return cv.polylines(image, [np_poly_points], isClosed=True, color=(0, 0, 255), thickness=5)


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
        frame = draw_zone_circle(frame, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, undistorted_zone_radius)
        frame = draw_zone_poly(frame, poly_zone_points_cv)
        frame = detection.draw_boxes(frame, plants_boxes)
        save_image(img_output_dir, frame, IMAGES_COUNTER, label, cur_time)

def move_to_point_and_extract(coords_from_to: list, gps: adapters.GPSUbloxAdapter, vesc_engine: adapters.VescAdapter,
                              smoothie: adapters.SmoothieAdapter, camera: adapters.CameraAdapterIMX219_170,
                              periphery_det: detection.YoloOpenCVDetection, precise_det: detection.YoloOpenCVDetection,
                              client, logger_full: utility.Logger, logger_table: utility.Logger, report_field_names,
                              trajectory_saver: utility.TrajectorySaver, undistorted_zone_radius, working_zone_polygon,
                              working_zone_points_cv, view_zone_polygon, view_zone_points_cv, img_output_dir,
                              nav: navigation.GPSComputing, data_collector: datacollection.DataCollector, log_cur_dir,
                              image_saver: utility.ImageSaver, notification: NotificationClient, SI_speed: float, wheels_straight: bool):
    """
    Moves to the given target point and extracts all weeds on the way.
    :param coords_from_to:
    :param gps:
    :param vesc_engine:
    :param smoothie:
    :param camera:
    :param periphery_det:
    :param precise_det:
    :param client:
    :param logger_full:
    :param logger_table:
    :param report_field_names:
    :param used_points_history:
    :param nav:
    :param working_zone_polygon:
    :param undistorted_zone_radius:
    :param working_zone_points_cv:
    :param img_output_dir:
    :return:
    """

    vesc_speed = SI_speed*-14285
    print(vesc_speed)
    vesc_engine.apply_rpm(vesc_speed)

    raw_angles_history = []
    detections_period =[]
    navigations_period =[]
    stop_helping_point = nav.get_coordinate(coords_from_to[1], coords_from_to[0], 90, 1000)
    
    last_skipped_point = coords_from_to[0]
    start_Nav_while =True
    last_correct_raw_angle = 0
    point_status ="origin"
    nav_status = "pursuit"
    last_corridor_side = 0
    current_corridor_side = 1
    almost_start = 0
    
    prev_maneuver_time = time.time()
    current_working_mode = 1
    close_to_end = config.USE_SPEED_LIMIT  # True if robot is close to one of current movement vector points, False otherwise; False if speed limit near points is disabled

    lastNtripRestart = time.time()

    pierre_vitesse=1    # metre par seconde
    latprec=0
    longprec=0
    latB = coords_from_to[1][0]
    longB = coords_from_to[1][1]
    pierre_angle_max=math.radians(23)
    pierre_E=0.86    #empattement entre les deux axes des roues
    graph_lat=[latprec]
    graph_long=[longprec]

    #Rq: long=x, lat=y

    pierre_angle=0 #Angle du robot vers la cible (en radians) en considérant que le robot est dans l'axe AB


    # set camera to the Y min
    res = smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2 / config.XY_COEFFICIENT_TO_MM, Y=config.Y_MIN)
    if res != smoothie.RESPONSE_OK:
        msg = "INIT: Failed to move camera to Y min, smoothie response:\n" + res
        logger_full.write(msg + "\n")
    smoothie.wait_for_all_actions_done()

    # TODO: maybe should add sleep time as camera currently has delay

    if config.AUDIT_MODE:
        vesc_engine.apply_rpm(vesc_speed)
        vesc_engine.start_moving()
    
    extraction_manager = ExtractionManager(smoothie, camera, working_zone_polygon, working_zone_points_cv,
                                           logger_full, data_collector, image_saver, log_cur_dir, periphery_det, precise_det)

    msgQueue, notificationQueue = None, None

    try:
        msgQueue = posix_ipc.MessageQueue(config.QUEUE_NAME_UI_MAIN)
    except:
        pass

    try:
        notificationQueue = posix_ipc.MessageQueue(config.QUEUE_NAME_UI_NOTIFICATION)
    except:
        pass

    degraded_navigation_mode = False
    
    number_navigation_cycle_without_gps = 0

    # main navigation control loop
    while True:

        # NAVIGATION CONTROL
        if degraded_navigation_mode:
            number_navigation_cycle_without_gps += 1

        if (degraded_navigation_mode and number_navigation_cycle_without_gps == config.GPS_CHECK_IN_DEGRADED_MODE*2) or not degraded_navigation_mode:

            try:
                    
                if degraded_navigation_mode:
                    vesc_engine.stop_moving()
                
                cur_pos = gps.get_fresh_position()
            
                if degraded_navigation_mode and notificationQueue is not None:                    
                    notificationQueue.send(json.dumps({"message_name": "Degraded_navigation_mode_off"}))
                    msg = "Degraded navigation mode off"
                    logger_full.write(msg + "\n")
                    if config.VERBOSE:
                        print(msg)
                    degraded_navigation_mode = False
                
            except TimeoutError:
                
                if not degraded_navigation_mode and notificationQueue is not None:                    
                    notificationQueue.send(json.dumps({"message_name": "Degraded_navigation_mode_on"}))
                    
                msg = "Degraded navigation mode on"
                logger_full.write(msg + "\n")
                if config.VERBOSE:
                    print(msg)
                        
                degraded_navigation_mode = True
                start_Nav_while = True
                number_navigation_cycle_without_gps = 0

            finally:
                if not vesc_engine._allow_movement:
                    vesc_engine.start_moving()

        if config.CONTINUOUS_INFORMATION_SENDING and not degraded_navigation_mode:
            notification.set_current_coordinate(cur_pos)

        nav_start_t = time.time()

        if start_Nav_while:
            navigation_period =1
        else:
            navigation_period = nav_start_t - prev_maneuver_time 
        
        navigations_period.append(navigation_period)  
        prev_maneuver_time = nav_start_t #time reference to decide the number of detection before resuming gps.get
        #print("tock")
            
        if not degraded_navigation_mode:
            
            if start_Nav_while:
                prev_pos = cur_pos
                start_Nav_while=False

            mu_navigations_period, sigma_navigations_period = utility.mu_sigma(navigations_period)

            latprec=cur_pos[0]
            longprec=cur_pos[1]

            if str(cur_pos) == str(prev_pos):
                # msg = "Got the same position, added to history, calculations skipped. Am I stuck?"
                # print(msg)
                # logger_full.write(msg + "\n")
                continue

            trajectory_saver.save_point(cur_pos)
            if msgQueue is not None:
                msgQueue.send(json.dumps({"last_gps": cur_pos}))
            """
            if len(used_points_history) > 0:
                if str(used_points_history[-1]) != str(cur_pos):
                    used_points_history.append(cur_pos.copy())
            else:
                used_points_history.append(cur_pos.copy())
            """

            """
            if not client.sendData("{};{}".format(cur_pos[0], cur_pos[1])):
                msg = "[Client] Connection closed !"
                print(msg)
                logger_full.write(msg + "\n")
            """

            distance = nav.get_distance(cur_pos, coords_from_to[1])
            
            last_corridor_side = current_corridor_side
            perpendicular, current_corridor_side = nav.get_deviation(coords_from_to[0],coords_from_to[1],cur_pos)
            
            
                    
            
            
            
            # check if arrived
            _, side = nav.get_deviation(coords_from_to[1], stop_helping_point, cur_pos)
            # if distance <= config.COURSE_DESTINATION_DIFF:  # old way
            if side != 1:  # TODO: maybe should use both side and distance checking methods at once
                vesc_engine.stop_moving()
                # msg = "Arrived (allowed destination distance difference " + str(config.COURSE_DESTINATION_DIFF) + " mm)"
                msg = "Arrived to " + str(coords_from_to[1])  # TODO: service will reload script even if it done his work?
                # print(msg)
                logger_full.write(msg + "\n")
                
                #put the wheel straight
                if wheels_straight:
                    response = smoothie.nav_turn_wheels_to(0, config.A_F_MAX)
                    if response != smoothie.RESPONSE_OK:  # TODO: what if response is not ok?
                        msg = "Smoothie response is not ok: " + response
                        print(msg)
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

            #PIERRE PREDICTION
        
            if pierre_angle>pierre_angle_max :
                pierre_angle=pierre_angle_max

            #Rayon de braquage

            if math.sin(pierre_angle) != 0 :
                pierre_rayon=pierre_E/math.sin(pierre_angle)
                theta_rob=pierre_vitesse/pierre_rayon
            else :
                theta_rob = 0
            
            #Calcul nouvelles coordonnées

            pierre_k=1/(60*1852) #Facteur de conversion metre vers degrès mille nautique
            distance_deg=pierre_vitesse*navigation_period*pierre_k #Distance en degres parcourue par le robot

            #La vitesse étant constante, on souhaite que le temps entre chaque point soit de 1 sec, comme on a des m/s, "distance=vitesse"

            psi=(math.pi-theta_rob)/2 # Angle de trajectoire - pi/2
            delta=distance_deg*math.sin(psi) # Différence de latitude entre position courante et future
            beta=distance_deg*math.cos(psi) # Différence de longitude entre position courante et future

            latnew=latprec+delta
            longnew=longprec+beta

            graph_lat.append(latnew)
            graph_long.append(longnew)

            #Les coordonnées en n deviennent les n-1

            latprec=latnew
            longprec=longnew
            
            latB = coords_from_to[1][0]
            longB = coords_from_to[1][1]
            
            if (latB-latnew) != 0 :
                pierre_angle=math.atan((longB-longnew)/(latB-latnew))# Angle co,signe roues
            else :
                pierre_angle = math.pi/2

            print("pierre_angle",pierre_angle)
            
            new_foreseen_point = list()
            new_foreseen_point.append(latnew)
            new_foreseen_point.append(longnew)
            new_foreseen_point.append("new_foreseen_point")

            trajectory_saver.save_point(new_foreseen_point)

            pierre_error = nav.get_distance(new_foreseen_point, cur_pos)

            print("pierre_error",pierre_error)


        
            #raw_angle_cruise = nav.get_angle(coords_from_to[0], cur_pos, cur_pos, coords_from_to[1])
            raw_angle_legacy = nav.get_angle(prev_pos, cur_pos, cur_pos, coords_from_to[1])
            raw_angle_cruise = - current_corridor_side * math.log(1+perpendicular)
            raw_angle = raw_angle_legacy + raw_angle_cruise
        
        
            #NAVIGATION STATE MACHINE
            
            
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
            
            if nav_status=="pursuit":
                if almost_start >=config.PURSUIT_LIMIT:
                    nav_status="cruise"
            
            
            
            if nav_status=="cruise":
                if almost_start < config.PURSUIT_LIMIT:
                    nav_status="pursuit"
        

        
            

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

            
            KP = 0.2*0,55
            KI = 0.0092*0,91

            if SI_speed in config.KP:
                KP = config.KP[SI_speed]
            else:
                msg = f"Speed SI {SI_speed} not present in KP."
                if config.VERBOSE:
                    print(msg)
                logger_full.write(msg + "\n")
            
            if SI_speed in config.KI:
                KI = config.KI[SI_speed]
            else:
                msg = f"Speed SI {SI_speed} not present in KI."
                if config.VERBOSE:
                    print(msg)
                logger_full.write(msg + "\n")

            angle_kp_ki = raw_angle * KP + sum_angles * KI 
            
            if SI_speed in config.CLOSE_TARGET_THRESHOLD: #check that rpm configuration is present in CLOSE_TARGET_THRESHOLD

                if distance < config.CLOSE_TARGET_THRESHOLD[SI_speed]:

                    if SI_speed in config.SMALL_RAW_ANGLE_SQUARE_THRESHOLD: #check that rpm configuration is present in SMALL_RAW_ANGLE_SQUARE_THRESHOLD

                        if (raw_angle * raw_angle) < config.SMALL_RAW_ANGLE_SQUARE_THRESHOLD[SI_speed]:

                            if SI_speed in config.SMALL_RAW_ANGLE_SQUARE_GAIN: #check that rpm configuration is present in SMALL_RAW_ANGLE_SQUARE_GAIN
                                angle_kp_ki *= config.SMALL_RAW_ANGLE_SQUARE_GAIN[SI_speed]

                            else: #rpm not present in SMALL_RAW_ANGLE_SQUARE_GAIN
                                msg = f"Speed SI {SI_speed} not present in SMALL_RAW_ANGLE_SQUARE_GAIN."
                                #print(msg)
                                logger_full.write(msg + "\n")
                    
                    else: #rpm not present in SMALL_RAW_ANGLE_SQUARE_THRESHOLD
                        msg = f"Speed SI {SI_speed} not present in SMALL_RAW_ANGLE_SQUARE_THRESHOLD."
                        #print(msg)
                        logger_full.write(msg + "\n")

                    if SI_speed in config.BIG_RAW_ANGLE_SQUARE_THRESHOLD: #check that rpm configuration is present in BIG_RAW_ANGLE_SQUARE_THRESHOLD

                        if (raw_angle * raw_angle) > config.BIG_RAW_ANGLE_SQUARE_THRESHOLD[SI_speed]:

                            if SI_speed in config.BIG_RAW_ANGLE_SQUARE_GAIN: #check that rpm configuration is present in BIG_RAW_ANGLE_SQUARE_GAIN
                                angle_kp_ki *= config.BIG_RAW_ANGLE_SQUARE_GAIN[SI_speed]
                                
                            else: #rpm not present in BIG_RAW_ANGLE_SQUARE_GAIN
                                msg = f"Speed SI {SI_speed} not present in BIG_RAW_ANGLE_SQUARE_GAIN."
                                #print(msg)
                                logger_full.write(msg + "\n")

                    else: #rpm not present in BIG_RAW_ANGLE_SQUARE_THRESHOLD
                        msg = f"Speed SI {SI_speed} not present in BIG_RAW_ANGLE_SQUARE_THRESHOLD."
                        #print(msg)
                        logger_full.write(msg + "\n")

            else: #rpm not present in CLOSE_TARGET_THRESHOLD
                msg = f"Speed SI {SI_speed} not present in CLOSE_TARGET_THRESHOLD."
                #print(msg)
                logger_full.write(msg + "\n")

            if SI_speed in config.FAR_TARGET_THRESHOLD: #check that rpm configuration is present in FAR_TARGET_THRESHOLD
                if distance > config.FAR_TARGET_THRESHOLD[SI_speed]:

                    if SI_speed in config.FAR_TARGET_GAIN: #check that rpm configuration is present in FAR_TARGET_GAIN
                        angle_kp_ki *= config.FAR_TARGET_GAIN[SI_speed]   
                    else:
                        msg = f"Speed SI {SI_speed} not present in FAR_TARGET_GAIN."
                        #print(msg)
                        logger_full.write(msg + "\n")  

            else:
                msg = f"Speed SI {SI_speed} not present in FAR_TARGET_THRESHOLD."
                #print(msg)
                logger_full.write(msg + "\n")     



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
                response = smoothie.nav_turn_wheels_to(order_angle_sm, config.A_F_MAX)
            else:
                response = smoothie.nav_turn_wheels_to(-order_angle_sm, config.A_F_MAX)
            if response != smoothie.RESPONSE_OK:  # TODO: what if response is not ok?
                msg = "Smoothie response is not ok: " + response
                print(msg)
                logger_full.write(msg + "\n")

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

            if str(gps_quality) not in ["4","5"] and time.time() - lastNtripRestart > config.NTRIP_RESTART_TIMEOUT and config.NTRIP:
                msg="Restart Ntrip because 60 seconds without corrections"
                logger_full.write(msg + "\n")
                if config.VERBOSE: 
                    print(msg)
                os.system("sudo systemctl restart ntripClient.service")
                lastNtripRestart = time.time()

            msg = str(gps_quality).ljust(5) + str(raw_angle).ljust(8) + str(angle_kp_ki).ljust(8) + str(
                order_angle_sm).ljust(8) + str(sum_angles).ljust(8) + str(distance).ljust(13) + str(ad_wheels_pos).ljust(
                8) + str(sm_wheels_pos).ljust(9) + point_status.ljust(12)+str(perpendicular).ljust(8)+corridor+nav_status+str(raw_angle_cruise).ljust(8)
            print(msg)
            logger_full.write(msg + "\n")

            # load sensors data to csv
            s = ","
            msg = str(gps_quality) + s + str(raw_angle) + s + str(angle_kp_ki) + s + str(order_angle_sm) + s + \
                str(sum_angles) + s + str(distance) + s + str(ad_wheels_pos) + s + str(sm_wheels_pos)
            vesc_data = vesc_engine.get_sensors_data(report_field_names)
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

        extraction_manager.reset_map()

        # EXTRACTION CONTROL
        while True:   # perform detection until either it is time to perform a new navigation, or there is something to goget
            start_t = time.time()
            frame = camera.get_image()
            frame_t = time.time()
            #print("tick")
                    
            plants_boxes = periphery_det.detect(frame)
            per_det_t = time.time()
            detections_period.append(per_det_t-start_t)
            

            if config.SAVE_DEBUG_IMAGES:
                image_saver.save_image(frame, img_output_dir,
                                    label="(periphery view scan M=" + str(current_working_mode) + ")",
                                    plants_boxes=plants_boxes)

            msg = "View frame time: " + str(frame_t - start_t) + "\t\tPeri. det. time: " + str(per_det_t - frame_t)
            logger_full.write(msg + "\n")
            
            if ExtractionManager.any_plant_in_zone(plants_boxes, working_zone_polygon):
                break
                
            # do maneuvers not more often than specified value
            
            mu_detections_period, sigma_detections_period = utility.mu_sigma(detections_period)
    
            cur_time = time.time()
            detections_time = cur_time - prev_maneuver_time
            #print(detections_time,"mu detection =%2.13f"%mu_detections_period, " sigma =%E"%sigma_detections_period)
            
            #is there enough time before the next navigation to complete a last detection :
            # the average detection time is mu_detections_period. Assuming that no period exceed mu+3*standard_deviation
            #print("if estimate next station",detections_time + mu_detections_period + 3*sigma_detections_period,"> threshold ",config.MANEUVERS_FREQUENCY-config.GPS_CLOCK_JITTER)
            if (detections_time + mu_detections_period + 3*sigma_detections_period) > config.MANEUVERS_FREQUENCY-config.GPS_CLOCK_JITTER:
                break

        extraction_manager.extraction_control(plants_boxes, img_output_dir, vesc_engine, close_to_end, current_working_mode, vesc_speed)


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

    if config.AUDIT_MODE in config.MANEUVER_START_DISTANCE: 
        maneuverStartDistance = config.MANEUVER_START_DISTANCE[config.AUDIT_MODE]
    else:
        msg = f"Speed SI {config.AUDIT_MODE} not present in MANEUVER_START_DISTANCE."
        #print(msg)
        logger.write(msg + "\n")   
        return None

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
    
    if config.AUDIT_MODE in config.MANEUVER_START_DISTANCE: 
        maneuverStartDistance = config.MANEUVER_START_DISTANCE[config.AUDIT_MODE]
    else:
        msg = f"Speed SI {config.AUDIT_MODE} not present in MANEUVER_START_DISTANCE."
        #print(msg)
        logger.write(msg + "\n")   
        return None

    if config.AUDIT_MODE in config.SPIRAL_SIDES_INTERVAL: 
        spiralSidesInterval = config.SPIRAL_SIDES_INTERVAL[config.AUDIT_MODE]
    else:
        msg = f"Speed SI {config.AUDIT_MODE} not present in MANEUVSPIRAL_SIDES_INTERVALER_START_DISTANCE."
        #print(msg)
        logger.write(msg + "\n")  
        return None

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

    if config.AUDIT_MODE in config.SPIRAL_SIDES_INTERVAL: 
        spiralSidesInterval = config.SPIRAL_SIDES_INTERVAL[config.AUDIT_MODE]
    else:
        msg = f"Speed SI {config.AUDIT_MODE} not present in MANEUVSPIRAL_SIDES_INTERVALER_START_DISTANCE."
        #print(msg)
        logger.write(msg + "\n")  
        return None

    # check if moving vector is too small for maneuvers
    if spiralSidesInterval * 2 >= cur_vec_dist:
        msg = "No place for maneuvers; Config spiral interval (that will be multiplied by 2): " + \
              str(spiralSidesInterval) + " Current moving vector distance is: " + str(cur_vec_dist) + \
              " Given points are: " + str(point_a) + " " + str(point_b)
        # print(msg)
        logger.write(msg + "\n")
        return None

    point_x1_int = nav.get_point_on_vector(point_a, point_b, spiralSidesInterval)
    point_x2_int = nav.get_point_on_vector(point_a, point_b, cur_vec_dist - spiralSidesInterval)
    return point_x1_int, point_x2_int


def compute_x1_x2_int_points_bend(point_a: list, point_b: list, nav: navigation.GPSComputing, logger: utility.Logger,SI_speed: float):
    """
    Computes spiral interval points x1, x2
    :param point_a:
    :param point_b:
    :param nav:
    :param logger:
    :return:
    """

    cur_vec_dist = nav.get_distance(point_a, point_b)

    """if config.AUDIT_MODE in config.SPIRAL_SIDES_INTERVAL: 
        spiralSidesInterval = config.SPIRAL_SIDES_INTERVAL[config.AUDIT_MODE]
    else:
        msg = f"Speed SI {config.AUDIT_MODE} not present in MANEUVSPIRAL_SIDES_INTERVALER_START_DISTANCE."
        #print(msg)
        logger.write(msg + "\n")  
        return None
    """
    period_gps = 1.00                   # seconds
    spiralSidesInterval = SI_speed * period_gps * 1000              # distance (mm ) = speed(m/s) * time *1000 
    # check if moving vector is too small for maneuvers
    if spiralSidesInterval * 2 >= cur_vec_dist:
        msg = "No place for maneuvers; Config spiral interval (that will be multiplied by 2): " + \
              str(spiralSidesInterval) + " Current foreseen distance is: " + str(cur_vec_dist) + \
              " Given points are: " + str(point_a) + " " + str(point_b) + " at speed(m/s) " + str(SI_speed)
        # print(msg)
        logger.write(msg + "\n")
        return None

    point_x1_int = nav.get_point_on_vector(point_a, point_b, spiralSidesInterval)
    point_x2_int = nav.get_point_on_vector(point_a, point_b, cur_vec_dist - spiralSidesInterval)
    return point_x1_int, point_x2_int


def add_points_to_path(path: list, *args):
    for point in args:
        if point is None:
            print("point is none")
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
    print("black corridor width at full steering %2.0f"%get_rectangle_isosceles_side(turning_radius)," millimeters")
    return int((get_rectangle_isosceles_side(turning_radius))/config.FIELD_REDUCE_SIZE)+1 #how many corner round due to robot working width

def add_forward_backward_path(abcd_points: list, nav: navigation.GPSComputing, logger: utility.Logger,SI_speed: float, currently_path: list):
    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    fwd = SI_speed
    rev = - SI_speed

    if config.AUDIT_MODE in config.SPIRAL_SIDES_INTERVAL: 
        spiralSidesInterval = config.SPIRAL_SIDES_INTERVAL[config.AUDIT_MODE]
    else:
        msg = f"Boolean {config.AUDIT_MODE} isn't present in SPIRAL_SIDES_INTERVAL."
        #print(msg)
        logger.write(msg + "\n")   
        return None

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


def build_bezier_with_corner_path(abcd_points: list, nav: navigation.GPSComputing, logger: utility.Logger,SI_speed: float):
    path = []
    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    fwd = SI_speed
    rev = - SI_speed

    if config.AUDIT_MODE in config.SPIRAL_SIDES_INTERVAL: 
        spiralSidesInterval = config.SPIRAL_SIDES_INTERVAL[config.AUDIT_MODE]
    else:
        msg = f"Boolean {config.AUDIT_MODE} isn't present in SPIRAL_SIDES_INTERVAL."
        #print(msg)
        logger.write(msg + "\n")   
        return None

    # get moving points A1 - ... - D2 spiral
    a1, a2 = compute_x1_x2_points(a, b, nav, logger)
    b1, b2 = compute_x1_x2_points(b, c, nav, logger)
    c1, c2 = compute_x1_x2_points(c, d, nav, logger)
    d1, d2 = compute_x1_x2_points(d, a, nav, logger)
    a1_spiral = nav.get_coordinate(a1, a, 90, spiralSidesInterval)
    d_spiral, a_spiral = compute_x1_x2(d,a,spiralSidesInterval,nav)

    if not add_points_to_path(path, [a, fwd]):
        return path

    first_bezier_turn = compute_bezier_points(a2,b,b1)
    second_bezier_turn = compute_bezier_points(b2,c,c1)
    third_bezier_turn = compute_bezier_points(c2,d,d1)
    #fourth_bezier_turn = compute_bezier_points(d2,a,a1)
    fourth_bezier_turn = compute_bezier_points(d2,a_spiral,a1_spiral)


    if config.AUDIT_MODE in config.MANEUVER_START_DISTANCE: 
        maneuverStartDistance = config.MANEUVER_START_DISTANCE[config.AUDIT_MODE]
    else:
        msg = f"Boolean {config.AUDIT_MODE} isn't present in MANEUVER_START_DISTANCE."
        #print(msg)
        logger.write(msg + "\n")   
        return None


    turning_radius = maneuverStartDistance   # minimum turning radius given in millimeter
    rnd = 0
    rnds = corner_finish_rounds(turning_radius)
    print(" corner rounds :" ,str(rnds))

    for rnd in range(rnds+1):             # example a 3meter radius requires 4 corners finish
        # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
        mxt = "corner rnd "+str(rnd)+"/"+str(rnds)
        
        print(mxt)
        
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
        # b1_int, b2_int = compute_x1_x2_int_points_bend(b, c, nav, logger, SI_speed)
        # d1_int, d2_int = compute_x1_x2_int_points_bend(d, a, nav, logger, SI_speed)
        b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
        d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

        if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
            return path

        # a_new, b_new = compute_x1_x2_int_points_bend(d2_int, b1_int, nav, logger, SI_speed)
        # c_new, d_new = compute_x1_x2_int_points_bend(b2_int, d1_int, nav, logger, SI_speed)
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
        a1_spiral = nav.get_coordinate(a1, a, 90, spiralSidesInterval)
        d_spiral, a_spiral = compute_x1_x2(d,a,spiralSidesInterval,nav)

        for point in [a,b,c,d,a1,b1,c1,d1,a2,b2,c2,d2]:
            if point is None:
                return path

        first_bezier_turn = compute_bezier_points(a2,b,b1)
        second_bezier_turn = compute_bezier_points(b2,c,c1)
        third_bezier_turn = compute_bezier_points(c2,d,d1)
        #fourth_bezier_turn = compute_bezier_points(d2,a,a1)
        fourth_bezier_turn = compute_bezier_points(d2,a_spiral,a1_spiral)

    _break = False

    while True:
        # get A'B'C'D' (prepare next ABCD points)
        # b1_int, b2_int = compute_x1_x2_int_points_bend(b, c, nav, logger, SI_speed)
        # d1_int, d2_int = compute_x1_x2_int_points_bend(d, a, nav, logger, SI_speed)
        b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
        d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

        if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
            return path

        # a_new, b_new = compute_x1_x2_int_points_bend(d2_int, b1_int, nav, logger, SI_speed)
        # c_new, d_new = compute_x1_x2_int_points_bend(b2_int, d1_int, nav, logger, SI_speed)
        a_new, b_new = compute_x1_x2_int_points(d2_int, b1_int, nav, logger)
        c_new, d_new = compute_x1_x2_int_points(b2_int, d1_int, nav, logger)

        if not check_points_for_nones(a_new, b_new, c_new, d_new):
            break

        # get moving points A1 - ... - D2 spiral
        a1, a2 = compute_x1_x2_points(d2_int_prev, b, nav, logger)
        b1, b2 = compute_x1_x2_points(b, c, nav, logger)
        c1, c2 = compute_x1_x2_points(c, d, nav, logger)
        d1, d2 = compute_x1_x2_points(d, a, nav, logger)
        a1_spiral = nav.get_coordinate(a1, a, 90, spiralSidesInterval)
        d_spiral, a_spiral = compute_x1_x2(d,a,spiralSidesInterval,nav)

        for point in [a,b,c,d,a1,b1,c1,d1,a2,b2,c2,d2]:
            if point is None:
                if not _break:
                    _break = True
                    break
        if _break : 
            break
        
        first_bezier_turn = compute_bezier_points(a2,b,b1)
        second_bezier_turn = compute_bezier_points(b2,c,c1)
        third_bezier_turn = compute_bezier_points(c2,d,d1)
        #fourth_bezier_turn = compute_bezier_points(d2,a,a1)
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

    if config.ADD_FORWARD_BACKWARD_PATH_TO_BEZIER_CORNER_PATH:
        path = add_forward_backward_path([a, b, c, d], nav, logger, SI_speed, path)

    return path

def build_path(abcd_points: list, nav: navigation.GPSComputing, logger: utility.Logger):
    path = []
    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    # get moving points A1 - ... - D2 spiral
    a1, a2 = compute_x1_x2_points(a, b, nav, logger)
    b1, b2 = compute_x1_x2_points(b, c, nav, logger)
    c1, c2 = compute_x1_x2_points(c, d, nav, logger)
    d1, d2 = compute_x1_x2_points(d, a, nav, logger)
    d2_spiral = compute_x2_spiral(d, a, nav, logger)

    # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
    if not add_points_to_path(path, a, a1, a2, b1, b2, c1, c2, d1, d2_spiral):
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
        d2_spiral = compute_x2_spiral(d, a, nav, logger)

        # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
        if not add_points_to_path(path, a1, a2, b1, b2, c1, c2, d1, d2_spiral):
            return path

        a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int


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


def emergency_field_defining(vesc_engine: adapters.VescAdapter, gps: adapters.GPSUbloxAdapter,
                             nav: navigation.GPSComputing, cur_log_dir, logger_full: utility.Logger):
    msg = "Using emergency field creation..."
    logger_full.write(msg + "\n")
    print(msg)

    starting_point = gps.get_last_position()

    msg = "Moving forward..."
    logger_full.write(msg + "\n")
    print(msg)
    vesc_engine.start_moving()
    time.sleep(config.EMERGENCY_MOVING_TIME)
    vesc_engine.start_moving()

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


def main():
    time_start = utility.get_current_time()
    log_cur_dir = config.LOG_ROOT_DIR + time_start + "/"
    utility.create_directories(config.LOG_ROOT_DIR, log_cur_dir, config.DEBUG_IMAGES_PATH, config.DATA_GATHERING_DIR)
        
    notification = NotificationClient(time_start)
    image_saver = utility.ImageSaver()
    data_collector = datacollection.DataCollector(notification)
    working_zone_polygon = Polygon(config.WORKING_ZONE_POLY_POINTS)
    working_zone_points_cv = np.array(config.WORKING_ZONE_POLY_POINTS, np.int32).reshape((-1, 1, 2))
    view_zone_polygon = Polygon(config.VIEW_ZONE_POLY_POINTS)
    view_zone_points_cv = np.array(config.VIEW_ZONE_POLY_POINTS, np.int32).reshape((-1, 1, 2))
    nav = navigation.GPSComputing()
    used_points_history = []
    logger_full = utility.Logger(log_cur_dir + "log full.txt")
    logger_table = utility.Logger(log_cur_dir + "log table.csv")
    
    

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
        periphery_detector = detection.YoloDarknetDetector(config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_CONFIG_FILE,
                                                           config.PERIPHERY_DATA_FILE,
                                                           config.PERIPHERY_CONFIDENCE_THRESHOLD,
                                                           config.PERIPHERY_HIER_THRESHOLD,
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
        precise_detector = detection.YoloDarknetDetector(config.PRECISE_WEIGHTS_FILE, config.PRECISE_CONFIG_FILE,
                                                         config.PRECISE_DATA_FILE, config.PRECISE_CONFIDENCE_THRESHOLD,
                                                         config.PRECISE_HIER_THRESHOLD, config.PRECISE_NMS_THRESHOLD)
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

    

    # sensors picking
    report_field_names = ['temp_fet_filtered', 'temp_motor_filtered', 'avg_motor_current',
                          'avg_input_current', 'rpm', 'input_voltage']

    """
    # QGIS and sensor data transmitting
    path = os.path.abspath(os.getcwd())
    sensor_processor = SensorProcessing.SensorProcessing(path, 0)
    sensor_processor.startServer()
    client = socketForRTK.Client.Client(4000)
    time.sleep(1)
    if not client.connectionToServer():
        msg = "Connection refused for Server RTK."
        print(msg)
        logger_full.write(msg + "\n")
    sensor_processor.startSession()
    """
    client = None

    try:
        msg = "Initializing..."
        print(msg)
        logger_full.write(msg + "\n")

        # stubs.GPSStub(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP) as gps, \
        # utility.MemoryManager(config.DATA_GATHERING_DIR, config.FILES_TO_KEEP_COUNT) as memory_manager, \
        with \
            utility.TrajectorySaver(log_cur_dir + "used_gps_history.txt") as trajectory_saver, \
            adapters.SmoothieAdapter(smoothie_address) as smoothie, \
            adapters.VescAdapter(config.VESC_RPM_SLOW, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                                 config.VESC_CHECK_FREQ, vesc_address, config.VESC_BAUDRATE) as vesc_engine, \
            adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP) as gps, \
            adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                             config.CROP_H_TO, config.CV_ROTATE_CODE,
                                             config.ISP_DIGITAL_GAIN_RANGE_FROM,
                                             config.ISP_DIGITAL_GAIN_RANGE_TO,
                                             config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                             config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                             config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                             config.CAMERA_H, config.CAMERA_FRAMERATE,
                                             config.CAMERA_FLIP_METHOD) as camera:

            
            
                
            # load previous path
            if config.CONTINUE_PREVIOUS_PATH:
                # TODO: create path manager

                msg = "Loading previous path points"
                logger_full.write(msg + "\n")

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
                """if config.RECEIVE_FIELD_FROM_RTK:
                    msg = "Loading field coordinates from RTK"
                    logger_full.write(msg + "\n")

                    try:
                        field_gps_coords = load_coordinates(rtk.CURRENT_FIELD_PATH)
                    except AttributeError:
                        msg = "Couldn't get field file name from RTK script as it is wasn't assigned there."
                        print(msg)
                        logger_full.write(msg + "\n")
                        notification.setStatus(SyntheseRobot.HS)
                        exit(1)
                    except FileNotFoundError:
                        msg = "Couldn't not find " + rtk.CURRENT_FIELD_PATH + " file."
                        print(msg)
                        logger_full.write(msg + "\n")
                        notification.setStatus(SyntheseRobot.HS)
                        exit(1)
                    if len(field_gps_coords) < 5:
                        msg = "Expected at least 4 gps points in " + rtk.CURRENT_FIELD_PATH + ", got " + \
                              str(len(field_gps_coords))
                        print(msg)
                        logger_full.write(msg + "\n")
                        notification.setStatus(SyntheseRobot.HS)
                        exit(1)
                    field_gps_coords = nav.corner_points(field_gps_coords, config.FILTER_MAX_DIST, config.FILTER_MIN_DIST)
                """
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
                        path_points = build_path(field_gps_coords, nav, logger_full)
                    #path_points = build_bezier_path(field_gps_coords, nav, logger_full)
                    if config.BEZIER_CORNER_PATH:
                        path_points = build_bezier_with_corner_path(field_gps_coords, nav, logger_full, config.VESC_RPM_SLOW_SI)
                    if config.JUST_FORWARD_BACKWARD_PATH:
                        a,b,c,d = field_gps_coords[0], field_gps_coords[1], field_gps_coords[2], field_gps_coords[3]
                        path_points = add_forward_backward_path([d,a,b,c], nav, logger_full, config.VESC_RPM_SLOW_SI, [])
                    
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

            msg = 'GpsQ|Raw ang|Res ang|Ord ang|Sum ang|Distance    |Adapter|Smoothie|PointStatus|deviation|'
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
                
                GARAGE = [46.1336841, -1.1226950200000294, '1']
                SQUARE = [46.13394686, -1.1225468000000054, '1']
                TROUVE = [46.13402132, -1.1228246000006645, '1']
                BRISACH = [46.15437552, -1.118523500000007, '1']
                EVIDENCE = [46.15457596, -1.118661520000015, '1']
                
                
                
                for i in range(path_start_index, len(path_points)):
                    from_to = [path_points[i - 1][0], path_points[i][0]] 
                    print(from_to)
                    print(path_points[i][1])
                    
                    """
                    #start_dist1 = nav.get_distance(start_position, BRISACH)
                    #start_dist2 = nav.get_distance(start_position, EVIDENCE)
                    #start_dist1 = nav.get_distance(start_position, GARAGE)
                    #start_dist2 = nav.get_distance(start_position, SQUARE)
                    start_dist1 = nav.get_distance(start_position, TROUVE)
                    start_dist2 = nav.get_distance(start_position, SQUARE)
                    
                    
                    #print("distance to brisach",start_dist1)
                    #print("distance to evidence",start_dist2)
                    #print("distance to GARAGE",start_dist1)
                    print("distance to TROUVE",start_dist1)
                    print("distance to SQUARE",start_dist2)
                    
                    
                    #from_to = [GARAGE, SQUARE]
                    #from_to = [SQUARE, GARAGE]
                    #from_to = [EVIDENCE, BRISACH]
                    if start_dist1>start_dist2:
                        #from_to = [EVIDENCE, BRISACH]
                        #from_to = [SQUARE, GARAGE]
                        from_to = [SQUARE, TROUVE]
                    else:
                        #from_to = [BRISACH, EVIDENCE]
                        #from_to = [GARAGE, SQUARE]
                        from_to = [TROUVE, SQUARE]
                          
                    """
                    
                    from_to_dist = nav.get_distance(from_to[0], from_to[1])
                    
                    """

                    msg = "Current movement vector: " + str(from_to) + " Vector size: " + str(from_to_dist)
                    # print(msg)
                    logger_full.write(msg + "\n\n")
                    """

                    msg = "KP: " + str(config.KP) + " KI: " + str(config.KI) + " VESC_RPM_FAST: " + str(config.VESC_RPM_FAST)+" SMALL_RAW_ANGLE_SQUARE_GAIN: " + str(config.SMALL_RAW_ANGLE_SQUARE_GAIN)
                    # print(msg)
                    logger_full.write(msg + "\n\n")



                    if path_points[i][1] > 0 :
                        move_to_point_and_extract(from_to, gps, vesc_engine, smoothie, camera, periphery_detector,
                                              precise_detector, client, logger_full, logger_table, report_field_names,
                                              trajectory_saver, config.UNDISTORTED_ZONE_RADIUS, working_zone_polygon,
                                              working_zone_points_cv, view_zone_polygon, view_zone_points_cv,
                                              config.DEBUG_IMAGES_PATH, nav, data_collector, log_cur_dir, image_saver, notification, 
                                              path_points[i][1], False)
                    else:
                        if config.STOP_AND_WHEELS_STRAIGHT_BEFORE_MOVE_BACK :
                            vesc_engine.stop_moving()
                            response = smoothie.nav_turn_wheels_to(0, config.A_F_MAX)
                            if response != smoothie.RESPONSE_OK: 
                                msg = "Smoothie response is not ok: " + response
                                print(msg)
                                logger_full.write(msg + "\n")
                        navigation.NavigationV3.move_to_point(from_to, gps, vesc_engine, smoothie, logger_full, logger_table, 
                                                          report_field_names, trajectory_saver, nav, notification, path_points[i][1], False)

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
    except:
        msg = "Exception occurred:\n" + traceback.format_exc()
        print(msg)
        logger_full.write(msg + "\n")
        notification.setStatus(SyntheseRobot.HS)
    finally:
        """
        # save used gps points
        print("Saving positions histories...")
        if len(used_points_history) > 0:
            # TODO: don't accumulate a lot of points - write each of them to file as soon as they come
            save_gps_coordinates(used_points_history, log_cur_dir + "used_gps_history.txt")
        else:
            msg = "used_gps_history list has 0 elements!"
            print(msg)
            logger_full.write(msg + "\n")
        """
        
        #put the wheel straight
        with adapters.SmoothieAdapter(smoothie_address) as smoothie:
            #put the wheel straight
            msg = "Put the wheel straight"
            logger_full.write(msg + "\n")
            if config.VERBOSE:
                print(msg)
            with open(config.LAST_ANGLE_WHEELS_FILE, "r") as angle_file:
                angle = float(angle_file.read())
                smoothie._a_cur.value = angle
                response = smoothie.nav_turn_wheels_to(0,config.A_F_MAX)
                if response != smoothie.RESPONSE_OK:  # TODO: what if response is not ok?
                    msg = "Smoothie response is not ok: " + response
                    print(msg)
                    logger_full.write(msg + "\n")
                
        
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
            data_collector.save_extractions_data(log_cur_dir + config.STATISTICS_OUTPUT_FILE)
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

        # TODO: remove this to the multiple with statement used for smoothie, gps and rest
        """
        # close transmitting connections
        print("Closing transmitters...")
        sensor_processor.endSession()
        client.closeConnection()
        sensor_processor.stopServer()
        """

        try:
            posix_ipc.unlink_message_queue(config.QUEUE_NAME_UI_MAIN)
        except:
            pass

        try:
            sharedMemory = posix_ipc.SharedMemory(config.SHARED_MEMORY_NAME_DETECTED_FRAME)
            sharedMemory.unlink()
        except:
            pass

        detection.webStream.terminate()
        detection.webStream.join()

        print("Safe disable is done.")


if __name__ == '__main__':
    try:
        sharedMemory = posix_ipc.SharedMemory(config.SHARED_MEMORY_NAME_DETECTED_FRAME)
        sharedMemory.unlink()
    except:
        pass
    main()
