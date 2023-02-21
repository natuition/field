import math
from haversine import haversine
import numpy as np
from scipy.spatial import ConvexHull
import utility
import time
from config import config
import os

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

class AntiTheftZone:

    def __init__(self, field: list):
        field = field
        self.nav = GPSComputing()
        d1 = self.nav.get_distance(field[0],field[1])
        middle_d1 = self.nav.get_point_on_vector(field[0],field[1], d1/2)
        middle_opposite_d1 = self.nav.get_point_on_vector(field[2],field[3], d1/2)
        d2 = self.nav.get_distance(middle_d1,middle_opposite_d1)
        self.center = self.nav.get_point_on_vector(middle_d1,middle_opposite_d1, d2/2)
        self.circumcircle_radius = self.hypotenuse(d1/2,d2/2)

    def get_center(self):
        return self.center

    def hypotenuse(self,a,b):
        return math.sqrt(a**2+b**2)

    def coordianate_are_in_zone(self, coords: list):
        return self.nav.get_distance(self.center,coords) <= self.circumcircle_radius + config.ANTI_THEFT_ZONE_RADIUS


class NavigationV3:
    __ntrip_restart_ts = 0

    @staticmethod
    def restart_ntrip_service(logger_full: utility.Logger):
        """Will restart Ntrip service if time passed after last Ntrip restart is bigger than allowed in config

        Returns True if did Ntrip restart, returns False otherwise.
        """

        if not config.NTRIP:
            msg = f"Ntrip restart is aborted as Ntrip usage is disabled in config.NTRIP={config.NTRIP} key"
            print(msg)
            logger_full.write(msg + "\n")
            return False

        if time.time() - NavigationV3.__ntrip_restart_ts > config.NTRIP_RESTART_TIMEOUT:
            msg = "Restarting Ntrip service"
            logger_full.write(msg + "\n")
            if config.VERBOSE:
                print(msg)
            os.system("sudo systemctl restart ntripClient.service")
            NavigationV3.__ntrip_restart_ts = time.time()
            return True

    @staticmethod
    def get_last_ntrip_restart_ts():
        return NavigationV3.__ntrip_restart_ts


class NavigationPrediction:

    def __init__(self, logger_full: utility.Logger, nav: GPSComputing, log_cur_dir: str):
        self.angle=0
        self.index_angle = 0

        self.logger_full = logger_full
        self.nav = nav
        self.trajectorySaver = utility.TrajectorySaver(log_cur_dir + "navigation_prediction_history.txt", config.CONTINUE_PREVIOUS_PATH)

        #const
        self.max_angle = math.radians(23)
        self.E = 0.86 #wheelbase between the two axles of the wheels
        self.K = 1/(60*1852) #Conversion factor meter to degree nautical mile
        self.navigation_period = config.MANEUVERS_FREQUENCY #1

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.trajectorySaver.__exit__(exc_type, exc_val, exc_tb)

    def set_SI_speed(self, SI_speed: float):
        self.speed = SI_speed #meter per second

    def set_current_lat_long(self, cur_pos: list):
        self.lat_current=cur_pos[0]
        self.long_current=cur_pos[1]

    def run_prediction(self, coords_from_to: list, cur_pos: list):

        if self.angle>self.max_angle :
            self.angle=self.max_angle

        #Turning radius

        if math.sin(self.angle) != 0 :
            rayon=self.E/math.sin(self.angle)
            theta_rob=self.speed/rayon
        else :
            theta_rob = 0
        
        #Calculation of new coordinates
        distance_deg=self.speed*self.navigation_period*self.K #Distance in degrees traveled by the robot

        #The speed being constant, we want the time between each point to be 1 sec, as we have m/s, "distance=speed"

        psi=(math.pi-theta_rob)/2 #Trajectory angle - pi/2
        delta=distance_deg*math.sin(psi) #Latitude difference between current and future position
        beta=distance_deg*math.cos(psi) #Longitude difference between current and future position

        latnew=self.lat_current+delta
        longnew=self.long_current+beta

        #Coordinates at n become n-1
        self.set_current_lat_long([latnew,longnew])
        
        self.latB = coords_from_to[1][0]
        self.longB = coords_from_to[1][1]

        if (self.latB-latnew) != 0:
            angle = math.atan((self.longB-longnew)/(self.latB-latnew)) #Angle co,wheels sign
        else:
            angle = math.pi/2

        msg = f"[PREDICTOR] Angle : {angle}."
        self.logger_full.write(msg + "\n")
        new_foreseen_point = [latnew, longnew, f"quality_cur_pos:{cur_pos[2]}", self.index_angle]

        self.trajectorySaver.save_point(new_foreseen_point)
        self.index_angle+=1

        msg = f"[PREDICTOR] Error : {self.nav.get_distance(new_foreseen_point, cur_pos)}."
        self.logger_full.write(msg + "\n")
