from navigation import GPSComputing
from typing import Dict
from config import config


class GearboxProtection:
    """
		Class detecting the physical robot blocage.
	"""

    def __init__(self):
        """
			Inits GearboxProtection.\n
			Create an empy list of cooridinates. \n
            Inits some parameters.
		"""
        
        self.__min_nb_valid_distances: int = config.MIN_NB_VALID_DISTANCES
        self.__max_nb_coords_stored: int = config.MAX_NB_COORDS_STORED
        self.__min_speed: int = config.MIN_SPEED #millimeters per second

        self.__coord_list = []
        self.__nb_extracts: int = 0
        self.__gps_computing: GPSComputing = GPSComputing()
        self.__percentage_of_min_speed: int = 0
        self.__step_percentage_of_min_speed: int = config.STEP_PERCENTAGE_OF_MIN_SPEED
        
    
    def store_coord(self, lat: float, long: float, quality: int) -> None:
        """
			Function for storing GPS coordinates in a list.\n
			:param lat: robot position latitude.\n
            :param long: robot position longitude.\n
            :param quality: quality of the RTK signal.
		"""
        coord = [lat, long, quality, self.__nb_extracts]
        if len(self.__coord_list) >= self.__max_nb_coords_stored :
            self.__coord_list.pop(0)
        self.__coord_list.append(coord)
        
    def store_number_of_extracts(self, extracts: Dict[str, int]) -> None:
        """
			Function for storing the number of extractions performed by the robot.\n
			:param extracts: dictionary of the extractions performed by the robot.
		"""
        self.__nb_extracts = self.__compute_number_of_extracts(extracts)

    def __compute_number_of_extracts(extracts: Dict[str, int]) -> int:
        """
			Function for computing the number of extractions performed by the robot.\n
			:param extracts: dictionary of the extractions.\n
            :return: total of the extractions.
		"""
        sum = 0
        for nb_extracts in extracts.values():
            sum += nb_extracts
        return sum
    
    def __check_same_gps_quality(self, point_A: list, point_B: list) -> bool:
        """
			Function for checking the equivalent quality between two points.\n
			:param point_A: coordinates of the first point (latitude, longitude, quality).\n
            :param point_B: coordinates of the secoond point (latitude, longitude, quality).\n
            :return: True if the GPS quality is equivalent, else otherwise.
		"""
        return ((point_A[3], point_B[3]) == (4, 4)) \
            or \
                (4 not in (point_A[3], point_B[3]))
        
    def is_physically_blocked(self) -> None:
        """
			Function for checking if the robot is physically blocked.\n
            :return: True if the robot is physically blocked, else otherwise.
		"""
        nb_coords = len(self.__coord_list)
        list_valid_distances = []
        for i in range(nb_coords - 2) :
            point_A = self.__coord_list[i]
            point_B = self.__coord_list[i+1]
            if not self.__check_same_gps_quality(point_A, point_B):
                continue
            list_valid_distances.append(self.__gps_computing.get_distance(point_A, point_B))
        
        if len(list_valid_distances) < self.__min_nb_valid_distances :
            return False
        
        # Calculate median distance
        list_valid_distances.sort()
        median_index = len(list_valid_distances) // 2
        median_value = list_valid_distances[median_index]
        print("Median distance = ", median_value)
        
        # Calculate the percentage of the minimum speed
        if self.__percentage_of_min_speed < 100 :
            self.__percentage_of_min_speed += self.__step_percentage_of_min_speed
        print("Minimum distance = ", self.__min_speed * (self.__percentage_of_min_speed / 100))
        
        return median_value < self.__min_speed * (self.__percentage_of_min_speed / 100)
    
    def is_remote(self, start_point: list) -> bool:
        """
			Function for checking the distance between two points.\n
			:param point_A: coordinates of the first point (latitude, longitude, quality).\n
            :param point_B: coordinates of the secoond point (latitude, longitude, quality).\n
            :return: True if the two points are remoted, else otherwise.
		"""
        current_point = self.__coord_list[-1]
        distance = self.__gps_computing.get_distance(start_point, current_point)
        return distance > config.REVERSING_DISTANCE
