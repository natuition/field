import math
from haversine import haversine


class GPSComputing:
    """
    Class containing methods for handling GPS-coordinates
    """

    def get_distance(self, point_1, point_2):
        """
        Function for finding the distance between two GPS-points
        :param point_1: coordinates of the first point (latitude, longitude)
        :param point_2: coordinates of the second point (latitude, longitude)
        :return: distance between point_1 and point_2 in millimeters
        """

        distance = haversine(point_1, point_2)
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
            perpendicular = 2 / start_stop * (per * (per - start_stop) * (per - point_start) * (per - point_stop)) ** 0.5

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

    def corner_points(self, gps_points):
        """
        Function for finding corner points of a quadrangle in a given list of GPS points
        :param gps_points: list of GPS-points
        :return: list of sorted corner GPS-points
        """

        corner_points = []
        points = [gps_points[len(gps_points)-2]] + [gps_points[len(gps_points)-1]] + gps_points
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
                    a = [points[i-1][1], points[i-1][0]]
                    b = [points[i][1], points[i][0]]
                    c = [points[i+1][1], points[i+1][0]]
                    if i == len(points) - 2:
                        d = [points[2][1], points[2][0]]
                    else:
                        d = [points[i+2][1], points[i+2][0]]

                    vector_ba = [(a[0] - b[0]), (a[1] - b[1])]
                    vector_bc = [(c[0] - b[0]), (c[1] - b[1])]
                    angle_abc = self._get_angle(vector_ba, vector_bc)

                    if angle_abc < low_level or angle_abc > high_level:
                        vector_bd = [(d[0] - b[0]), (d[1] - b[1])]
                        angle_abd = self._get_angle(vector_ba, vector_bd)
                        if angle_abd < low_level or angle_abc > high_level:
                            corner_points.append([b[1], b[0]])
        return self._corner_sort(corner_points)

    def _get_angle(self, vector_1, vector_2):
        """
        Function for finding the angle between two vectors
        :param vector_1: coordinates of the first vector (latitude, longitude)
        :param vector_2: coordinates of the second vector (latitude, longitude)
        :return: angle between two vectors (in degrees)
        """

        cos_angle = (vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1]) /\
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
