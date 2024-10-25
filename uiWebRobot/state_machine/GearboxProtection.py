from navigation import GPSComputing

class GearboxProtection:
    def __init__(self):
        self.__coord_list = []
        self.__min_nb_coords = 10
        self.__nb_extracts = 0
        self.__gps_computing = GPSComputing()
        self.__min_speed = 100 #millimeters per second
        self.__min_time = 30 #seconds
        self.__min_distance = self.__min_speed * self.__min_time 
    
    def store_coord(self, lat: float, long: float, quality: int):
        coord = [lat, long, quality, self.__nb_extracts]
        if len(self.__coord_list) >= self.__min_nb_coords :
            self.__coord_list.pop(0)
        self.__coord_list.append(coord)
        
    def __are_coord_closed(self, coord1, coord2):
        if coord1[2] == coord2[2] :
            distance_millimeters = self.__gps_computing.get_distance(coord1, coord2)
            print("Coordinates are closed : ", distance_millimeters, ", ", (distance_millimeters < self.__min_distance))
            return distance_millimeters < self.__min_distance
        else :
            print("Coordinates have not the same gps quality.")
            return False
        
    def store_number_of_extracts(self, extracts):
        self.__nb_extracts = self.__compute_number_of_extracts(extracts)

    def __compute_number_of_extracts(extracts):
        sum = 0
        for nb_extracts in extracts.values():
            sum += nb_extracts
        return sum

    def is_physically_blocked(self):
        nb_coords = len(self.__coord_list)
        print("Nb coordinates = ", nb_coords)
        print("Min nb coordinates = ", self.__min_nb_coords)
        if nb_coords >= self.__min_nb_coords :
            last_coord = self.__coord_list[-1]
            for i in range(1, nb_coords, 1) :
                i_coord = self.__coord_list[i]
                if not self.__are_coord_closed(last_coord, i_coord) and last_coord[3] == i_coord[3]:
                    return False 
            return True
        else :
            return False