import math
from haversine import haversine
import numpy as np
from scipy.spatial import ConvexHull
import utility
import adapters
import time
from config import config
from notification import NotificationClient
import os
import posix_ipc
import json

class GPSComputing:
    """
    Class containing methods for handling GPS-coordinates
    """

    def get_coordinate(self, point_1, point_2, angle, distance):
        """
        Returns the coordinates of a point that is in 'angle' degrees and at 'distance' distance from a given vector
        :param point_1: start point of a known vector [lat, long]
        :param point_2: endpoint of a known vector [lat, long]
        :param angle: angle between a known vector and an unknown point (in degrees)
        :param distance: distance between starting point and unknown point (in millimeters)
        :return: coordinates of an unknown point [lat, long]
        """

        azimuth_1 = self._get_azimuth(point_1, point_2)
        if azimuth_1 > 180:
            azimuth_1 -= 360
        azimuth2 = azimuth_1 - angle
        if azimuth2 < -180:
            azimuth2 += 360
        elif azimuth2 > 180:
            azimuth2 -= 360
        azimuth2 = math.radians(azimuth2)

        lat_point2 = math.radians(point_1[0])
        long_point2 = math.radians(point_1[1])
        distance = math.radians(distance / 111195101.17748393)

        sin_lat = math.sin(lat_point2) * math.cos(distance) + math.cos(lat_point2) * math.sin(distance) * math.cos(
            azimuth2)
        lat = math.degrees(math.asin(sin_lat))

        tg_long = (math.sin(distance) * math.sin(azimuth2)) / (math.cos(lat_point2) * math.cos(distance) - (
                    math.sin(lat_point2) * math.sin(distance) * math.sin(azimuth2)))
        long = math.atan(tg_long)
        long = math.degrees(long + long_point2)

        coord = [lat, long]
        return coord

    def _get_azimuth(self, point_1, point_2):
        """
        Method for determining the azimuth - the angle between the north direction and the direction from the start point
        to the endpoint
        :param point_1: coordinates of the starting point (latitude, longitude)
        :param point_2: coordinates of the endpoint (latitude, longitude)
        :return: azimuth in degrees
        """

        # in radians
        lat1 = math.radians(point_1[0])
        lat2 = math.radians(point_2[0])
        long1 = math.radians(point_1[1])
        long2 = math.radians(point_2[1])
        # cosines and sines of latitudes and difference of longitudes
        cos_lat1 = math.cos(lat1)
        cos_lat2 = math.cos(lat2)
        sin_lat1 = math.sin(lat1)
        sin_lat2 = math.sin(lat2)

        delta = long2 - long1
        cdelta = math.cos(delta)
        sdelta = math.sin(delta)

        # calculation of the initial azimuth
        x = (cos_lat1 * sin_lat2) - (sin_lat1 * cos_lat2 * cdelta)
        if x == 0:
            x += 0.000000000000000000000001
        y = sdelta * cos_lat2
        z = math.degrees(math.atan(-y / x))

        if x < 0:
            z = z + 180.

        z2 = (z + 180.) % 360. - 180.

        z2 = - math.radians(z2)
        anglerad2 = z2 - (2 * math.pi) * math.floor(z2 / (2 * math.pi))
        angledeg = math.degrees(anglerad2)
        return angledeg

    def get_angle(self, point_1, point_2, point_3, point_4):
        """
        Method for finding the angle between two vectors represented by points (GPS coordinates)
        :param point_1: starting point of the first vector
        :param point_2: endpoint of the first vector
        :param point_3: starting point of the second vector
        :param point_4: endpoint of the second vector
        :return: angle between two vectors in degrees
        """

        azimuth_1 = self._get_azimuth(point_1, point_2)
        azimuth_2 = self._get_azimuth(point_3, point_4)
        if azimuth_1 > 180:
            azimuth_1 -= 360
        if azimuth_2 > 180:
            azimuth_2 -= 360

        angle = azimuth_1 - azimuth_2

        if angle < -180:
            angle += 360
        elif angle > 180:
            angle -= 360

        angle *= -1
        return angle

    def get_distance(self, point_1, point_2):
        """
        Function for finding the distance between two GPS-points
        :param point_1: coordinates of the first point (latitude, longitude)
        :param point_2: coordinates of the second point (latitude, longitude)
        :return: distance between point_1 and point_2 in millimeters
        """

        distance = haversine(point_1[:2], point_2[:2])
        distance *= 1000000
        return distance

    def get_point_on_vector(self, start_point, stop_point, distance):
        """
        Function for finding the GPS coordinates of a point located at a given distance from the beginning of the motion
        vector
        :param start_point: coordinates of the starting point (latitude, longitude)
        :param stop_point: coordinates of the endpoint (latitude, longitude)
        :param distance: distance from start to point in millimeters
        :return: coordinates (latitude, longitude) of the point
        """

        start = [start_point[1], start_point[0]]
        stop = [stop_point[1], stop_point[0]]
        dis_start_stop = self.get_distance(start_point, stop_point)
        k = distance / dis_start_stop
        long_point = start[0] + (stop[0] - start[0]) * k
        lat_point = start[1] + (stop[1] - start[1]) * k
        point = [lat_point, long_point]
        return point

    def get_deviation(self, start_point, stop_point, deviation_point):
        """
        Function for finding deviations from a given motion vector
        :param start_point: coordinates of the starting point (latitude, longitude)
        :param stop_point: coordinates of the endpoint (latitude, longitude)
        :param deviation_point: coordinates of the point of current location (latitude, longitude)
        :return: perpendicular from the point of deviation to the motion vector in millimeters and
        the flag indicating the location relative to the motion vector (-1 - left, 0 - on a vector, 1 - right)
        """

        start_stop = self.get_distance(start_point, stop_point)
        point_start = self.get_distance(start_point, deviation_point)
        point_stop = self.get_distance(stop_point, deviation_point)
        per = (start_stop + point_start + point_stop) / 2
        if (per - start_stop) <= 0:
            perpendicular = 0
            flag = 0
        else:
            perpendicular = 2 / start_stop * (
                        per * (per - start_stop) * (per - point_start) * (per - point_stop)) ** 0.5

            d = (deviation_point[0] - start_point[0]) * (stop_point[1] - start_point[1]) - \
                (deviation_point[1] - start_point[1]) * (stop_point[0] - start_point[0])
            if d > 0:
                flag = -1
            elif d < 0:
                flag = 1
            else:
                flag = 0

        return perpendicular, flag

    def get_vector(self, start_point, stop_point, interval):
        """
        Function for constructing a motion vector from the start point to the end point, by generating GPS points
        between the start and stop at a given interval
        :param start_point: coordinates of the starting point (latitude, longitude)
        :param stop_point: coordinates of the endpoint (latitude, longitude)
        :param interval: distance between a points forming a vector (in centimeters)
        :return: list of points forming a vector
        """

        interval = interval / 0.93 / 11119510.117748393

        # conversion of centimeters to degrees and an ellipsoidal measurement system

        start = [start_point[1], start_point[0]]
        stop = [stop_point[1], stop_point[0]]

        dis = (((stop[0] - start[0]) ** 2 + (stop[1] - start[1]) ** 2) ** 0.5)

        r = dis // interval * interval  # multiple to interval

        if start[0] != stop[0]:
            k = (stop[1] - start[1]) / (stop[0] - start[0])  # angle tangent
            angle = math.atan(k)
            length_long = r * math.cos(angle)  # projection length on the axis of longitude
            length_lat = r * math.sin(angle)  # projection length on latitude axis

            count = int(r / interval)  # the number of points on the vector
            delta_length_long = length_long / count  # longitude increment

            delta_length_lat = length_lat / count  # latitude increment

            list_long = [start[0]]
            list_lat = [start[1]]

            if stop[0] < start[0]:
                for i in range(1, count + 1):
                    long = (list_long[i - 1] - delta_length_long)
                    list_long.append(long)

                if stop[1] > start[1]:
                    for i in range(1, count + 1):
                        lat = (list_lat[i - 1] - delta_length_lat)
                        list_lat.append(lat)
                else:
                    for i in range(1, count + 1):
                        lat = (list_lat[i - 1] - delta_length_lat)
                        list_lat.append(lat)
            else:
                for i in range(1, count + 1):
                    long = (list_long[i - 1] + delta_length_long)
                    list_long.append(long)

                for i in range(1, count + 1):
                    lat = (list_lat[i - 1] + delta_length_lat)
                    list_lat.append(lat)

        else:
            count = int(r / interval)
            list_long = [start[0]]
            list_lat = [start[1]]

            for i in range(1, count + 1):
                long = list_long[i - 1]
                list_long.append(long)

            if stop[1] < start[1]:
                for i in range(1, count + 1):
                    lat = list_lat[i - 1] - interval
                    list_lat.append(lat)
            else:
                for i in range(1, count + 1):
                    lat = list_lat[i - 1] + interval
                    list_lat.append(lat)

        list_long.append(stop[0])
        list_lat.append(stop[1])
        list_point = [[list_lat[i], list_long[i]] for i in range(0, len(list_long))]
        return list_point

    def corner_points_old(self, gps_points):
        """
        Function for finding corner points of a quadrangle in a given list of GPS points
        :param gps_points: list of GPS-points
        :return: list of sorted corner GPS-points
        """

        corner_points = []
        points = [gps_points[len(gps_points) - 2]] + [gps_points[len(gps_points) - 1]] + gps_points
        low_level = 180
        high_level = 180
        while True:
            if len(corner_points) == 4:
                break
            elif len(corner_points) < 4:
                low_level += 5
                high_level -= 5
            elif len(corner_points) > 4:
                low_level -= 5
                high_level += 5
            corner_points = []
            for i in range(1, len(points) - 1):
                a = [points[i - 1][1], points[i - 1][0]]
                b = [points[i][1], points[i][0]]
                c = [points[i + 1][1], points[i + 1][0]]
                if i == len(points) - 2:
                    d = [points[2][1], points[2][0]]
                else:
                    d = [points[i + 2][1], points[i + 2][0]]

                angle_abc = self.get_angle(b, a, b, c)

                if angle_abc < low_level or angle_abc > high_level:
                    angle_abd = self.get_angle(b, a, b, d)
                    if angle_abd < low_level or angle_abc > high_level:
                        corner_points.append([b[1], b[0]])
        return self._corner_sort(corner_points)

    def corner_points(self, gps_points, filter_max_dist, filter_min_dist):
        """
        Function for filtering and finding corner points of a quadrangle in a given list of GPS points
        :param gps_points: list of GPS-points
        :param filter_max_dist: maximum allowable distance between consecutive points (in millimeters)
        :param filter_min_dist: minimum allowable distance between consecutive points (in millimeters)
        :return: list of sorted corner GPS-points
        """

        gps_points = [gps_points[len(gps_points) - 1]] + gps_points
        i = 0
        while i < (len(gps_points) - 1):
            distance = self.get_distance(gps_points[i], gps_points[i + 1])
            if distance >= filter_max_dist or distance <= filter_min_dist:
                gps_points.remove(gps_points[i])
            else:
                i += 1

        temp_points = np.array(gps_points, dtype=np.float)
        hull = ConvexHull(temp_points)
        hull_indices = np.unique(hull.simplices.flat)
        hull_pts = temp_points[hull_indices, :]
        filter_points = hull_pts.tolist()

        while True:
            length = len(filter_points)
            corners = self._test_corner(filter_points)
            for i in range(len(corners)):
                if i == len(corners) - 1:
                    distance = self.get_distance(corners[i], corners[0])
                else:
                    distance = self.get_distance(corners[i], corners[i + 1])
                if distance < 5000:
                    filter_points.remove(corners[i])

            if len(filter_points) == length:
                break
        return corners

    def _test_corner(self, gps_points):
        """
        Function for finding corner points of a quadrangle in a given list of GPS points
        :param gps_points: list of GPS-points
        :return: list of sorted corner GPS-points
        """

        corner_points = []
        points = [gps_points[len(gps_points) - 2]] + [gps_points[len(gps_points) - 1]] + gps_points
        level = 150
        while True:
            if len(corner_points) == 4:
                break
            elif len(corner_points) < 4:
                level += 1
            elif len(corner_points) > 4:
                level -= 1
            corner_points = []
            for i in range(1, len(points) - 1):
                a = [points[i - 1][1], points[i - 1][0]]
                b = [points[i][1], points[i][0]]
                c = [points[i + 1][1], points[i + 1][0]]
                if i == len(points) - 2:
                    d = [points[2][1], points[2][0]]
                else:
                    d = [points[i + 2][1], points[i + 2][0]]

                angle_abc = abs(self.get_angle(b, a, b, c))

                if angle_abc < level:
                    angle_abd = abs(self.get_angle(b, a, b, d))
                    if angle_abd < level:
                        corner_points.append([b[1], b[0]])
        return self._corner_sort(corner_points)

    def _get_angle(self, vector_1, vector_2):
        """
        Function for finding the angle between two vectors
        :param vector_1: coordinates of the first vector (latitude, longitude)
        :param vector_2: coordinates of the second vector (latitude, longitude)
        :return: angle between two vectors (in degrees)
        """

        cos_angle = (vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1]) / \
                    ((vector_1[0] ** 2 + vector_1[1] ** 2) ** 0.5 * (vector_2[0] ** 2 + vector_2[1] ** 2) ** 0.5)
        cos_angle = round(cos_angle, 5)
        angle = math.acos(cos_angle)
        angle = math.degrees(angle)
        return angle

    def _corner_sort(self, corner_list):
        """
        Function for determining the position of a quadrangle in space and arranges the corners clockwise
        :param corner_list: list of corner GPS-points of a quadrangle
        :return: list of sorted corner GPS-points
        """

        corner_temp = [[corner_list[0][1], corner_list[0][0]], [corner_list[1][1], corner_list[1][0]],
                       [corner_list[2][1], corner_list[2][0]], [corner_list[3][1], corner_list[3][0]]]
        corner_copy = list(corner_temp)
        min_long = sorted(corner_copy)[:2]
        min_long_reverse = [[min_long[0][1], min_long[0][0]], [min_long[1][1], min_long[1][0]]]
        min_lat_reverse = min(min_long_reverse)
        first_point = [min_lat_reverse[1], min_lat_reverse[0]]

        corner_copy.remove(first_point)
        min_long_sort = sorted(corner_copy)[:1]
        second_point = min_long_sort[0]

        corner_copy.remove(second_point)
        max_long_reverse = [[corner_copy[0][1], corner_copy[0][0]], [corner_copy[1][1], corner_copy[1][0]]]
        max_lat_reverse = max(max_long_reverse)
        third_point = [max_lat_reverse[1], max_lat_reverse[0]]

        corner_copy.remove(third_point)
        fourth_point = corner_copy[0]

        sort_points = [[first_point[1], first_point[0]], [second_point[1], second_point[0]],
                       [third_point[1], third_point[0]], [fourth_point[1], fourth_point[0]]]
        return sort_points

    def get_square_corners(self, center_point, corner_point):
        distance = self.get_distance(center_point, corner_point)
        corner_1 = self.get_coordinate(center_point, corner_point, 90, distance)
        corner_2 = self.get_coordinate(center_point, corner_point, -90, distance)
        corner_3 = self.get_coordinate(center_point, corner_point,  180, distance)
        corners = self._corner_sort([corner_point, corner_1, corner_2, corner_3])
        return corners

class NavigationV3:

    @staticmethod
    def move_to_point(coords_from_to: list, gps: adapters.GPSUbloxAdapter, vesc_engine: adapters.VescAdapter,
                        smoothie: adapters.SmoothieAdapter, logger_full: utility.Logger, logger_table: utility.Logger, 
                        report_field_names, trajectory_saver: utility.TrajectorySaver, nav: GPSComputing, 
                        notification: NotificationClient, SI_speed: float, wheels_straight: bool):
                        #, next_point: list = None):
        """
        Moves to the given target point and extracts all weeds on the way.
        :param coords_from_to:
        :param gps:
        :param vesc_engine:
        :param smoothie:
        :param logger_full:
        :param logger_table:
        :param report_field_names:
        :param trajectory_saver:
        :param notification:
        :param SI_speed:
        :param wheels_straight:
        :return:
        """

        vesc_speed = SI_speed*-14285
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
        current_corridor_side = 1
        almost_start = 0
        
        prev_maneuver_time = time.time()
        #close_to_end = config.USE_SPEED_LIMIT  # True if robot is close to one of current movement vector points, False otherwise; False if speed limit near points is disabled

        lastNtripRestart = time.time()

        pierre_vitesse=SI_speed    # metre par seconde
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

            if degraded_navigation_mode:
                number_navigation_cycle_without_gps += 1

            if (degraded_navigation_mode and number_navigation_cycle_without_gps == config.GPS_CHECK_IN_DEGRADED_MODE*2) or not degraded_navigation_mode:

                try:
                        
                    if degraded_navigation_mode:
                        vesc_engine.stop_moving()
                    
                    cur_pos = gps.get_last_position()
                
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

                distance = nav.get_distance(cur_pos, coords_from_to[1])
                
                perpendicular, current_corridor_side = nav.get_deviation(coords_from_to[0],coords_from_to[1],cur_pos)
                
                
                # check if arrived
                _, side = nav.get_deviation(coords_from_to[1], stop_helping_point, cur_pos)
                # if distance <= config.COURSE_DESTINATION_DIFF:  # old way
                if side != 1:  # TODO: maybe should use both side and distance checking methods at once
                    """if isinstance(next_point,list) and config.TREAT_SEVERAL_POINT:
                        if next_point[-1] == coords_from_to[1]:
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
                        else:
                            coords_from_to[0] = coords_from_to[1]
                            coords_from_to[1] = coords_from_to[coords_from_to.index(coords_from_to[0])+1]
                    else:"""
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
                    #close_to_end = distance < config.DECREASE_SPEED_TRESHOLD or distance_from_start < config.DECREASE_SPEED_TRESHOLD

                

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

                #print("pierre_angle",pierre_angle)
                
                new_foreseen_point = list()
                new_foreseen_point.append(latnew)
                new_foreseen_point.append(longnew)
                new_foreseen_point.append("new_foreseen_point")

                trajectory_saver.save_point(new_foreseen_point)

                pierre_error = nav.get_distance(new_foreseen_point, cur_pos)

                #print("pierre_error",pierre_error)


            
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
                    #print(msg)
                    logger_full.write(msg + "\n")
                
                if SI_speed in config.KI:
                    KI = config.KI[SI_speed]
                else:
                    msg = f"Speed SI {SI_speed} not present in KI."
                    #print(msg)
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

                lastNtripRestart = NavigationV3.check_reboot_Ntrip(gps_quality, lastNtripRestart, logger_full)

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

            while True: 
        
                cur_time = time.time()

                last_maneuver_time = cur_time - prev_maneuver_time
                
                if last_maneuver_time > config.MANEUVERS_FREQUENCY-config.GPS_CLOCK_JITTER:
                    break

    @staticmethod
    def check_reboot_Ntrip(gps_quality: str, lastNtripRestart: float, logger_full: utility.Logger):
        if str(gps_quality) not in ["4","5"] and time.time() - lastNtripRestart > config.NTRIP_RESTART_TIMEOUT and config.NTRIP:
            msg="Restart Ntrip because 60 seconds without corrections"
            logger_full.write(msg + "\n")
            if config.VERBOSE: 
                print(msg)
            os.system("sudo systemctl restart ntripClient.service")
            lastNtripRestart = time.time()
        return lastNtripRestart