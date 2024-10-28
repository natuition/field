from navigation import GPSComputing

class GearboxProtection:
    def __init__(self):
        self.__coord_list = []
        self.__min_nb_coords = 10
        self.__nb_extracts = 0
        self.__gps_computing = GPSComputing()
        self.__min_speed = 300 #millimeters per second
    
    def store_coord(self, lat: float, long: float, quality: int):
        coord = [lat, long, quality, self.__nb_extracts]
        if len(self.__coord_list) >= self.__min_nb_coords :
            self.__coord_list.pop(0)
        self.__coord_list.append(coord)
        
    def __are_coord_closed(self, coord1, coord2):
        distance_millimeters = self.__gps_computing.get_distance(coord1, coord2)
        return distance_millimeters < self.__min_speed
        
    def store_number_of_extracts(self, extracts):
        self.__nb_extracts = self.__compute_number_of_extracts(extracts)

    def __compute_number_of_extracts(extracts):
        sum = 0
        for nb_extracts in extracts.values():
            sum += nb_extracts
        return sum
    
    def __check_same_gps_quality(self, point_A: list, point_B: list) -> bool:
        return ((point_A[3],point_B[3])==(4,4)) \
            or \
                (4 not in (point_A[3],point_B[3]))

    def is_physically_blocked(self):
        nb_coords = len(self.__coord_list)
        sum_valid_distances = 0
        nb_valid_distances = 0
        if nb_coords >= self.__min_nb_coords :
            for i in range(nb_coords - 2) :
                point_A = self.__coord_list[i]
                point_B = self.__coord_list[i+1]
                if not self.__check_same_gps_quality(point_A, point_B):
                    continue
                nb_valid_distances += 1
                sum_valid_distances += self.__gps_computing.get_distance(point_A, point_B)
            print("average = ", sum_valid_distances / nb_valid_distances)
            return (sum_valid_distances / nb_valid_distances) < self.__min_speed
        else :
            return False