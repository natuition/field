import numpy as np

import navigation
import utility

from check_config import prepare_valid_config
prepare_valid_config()
from config import config

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

def compute_x1_x2_points(point_a: list, point_b: list, nav: navigation.GPSComputing, logger: utility.Logger, add=0):
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
    if config.MANEUVER_START_DISTANCE * 2 + add >= cur_vec_dist:
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
ADD_FINAL_VIRAGE = True
ADD_DIRECT_TO_FINAL_CORNER = True
ADD_FORWARD_BACKWARD_TO_END_OF_BEZIER_PATH = False

def build_maneuvre_path(
                abcd_points,
                abcd_points_prev,
                nav,
                logger,
                SI_speed_fwd,
                SI_speed_rev,
                continue_path = list()):
    
    path = continue_path
    
    _ , a2 = compute_x1_x2_points(abcd_points[0], abcd_points[1], nav, logger)
    
    if a2 is not None:
        a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]
    else:
        a, b, c, d = abcd_points_prev[0], abcd_points_prev[1], abcd_points_prev[2], abcd_points_prev[3]
        path = path[:(-config.NUMBER_OF_BEZIER_POINT*4)]   
    
    if nav.get_distance(a,b) > nav.get_distance(b,c):
        a, b, c, d = d, a, b, c
        if len(path) == 0:
            path.append([b,SI_speed_fwd])
    else:
        if len(path) == 0:
            path.append([a,SI_speed_fwd])
        _ , a2 = compute_x1_x2_points(a, b, nav, logger)
        b1, _ = compute_x1_x2_points(b, c, nav, logger)
        
        b_corner_bezier = compute_bezier_points(a2, b, b1)
        if not add_points_to_path(path, *map(lambda gps_point: [gps_point, SI_speed_fwd], b_corner_bezier)):
            raise RuntimeError(
                "Failed to add B corner's bezier curve to path. This expected never to happen.")
            
    path.append([c,SI_speed_fwd])
        
    angle_current_half = 90
    sign_current_half = 1
        
    largeur_zone_final = nav.get_distance(c,d)
    
    while True:
        
        largeur_zone_final_en_cours = nav.get_distance(c,d)
        
        if largeur_zone_final/2 > largeur_zone_final_en_cours:
            angle_current_half = 270
            sign_current_half = -1
        
        angle = int(-nav.get_angle(b,c,c,d))
        angle = angle/abs(angle) * angle_current_half
    
        point_x1 = nav.get_point_on_vector(c, b, config.MANEUVER_START_DISTANCE)
        
        point_x2 = nav.get_coordinate(point_x1, c, angle, config.MANEUVER_START_DISTANCE)
        
        b_corner_bezier = compute_bezier_points(c, point_x1, point_x2)
        if not add_points_to_path(path, *map(lambda gps_point: [gps_point, SI_speed_rev], b_corner_bezier)):
            raise RuntimeError(
                "Failed to add B corner's bezier curve to path. This expected never to happen.")
            
        point_x3 = nav.get_point_on_vector(point_x2, point_x1, -config.SPIRAL_SIDES_INTERVAL*sign_current_half)
        
        path.append([point_x3,SI_speed_rev])
        
        point_x4 = nav.get_point_on_vector(point_x3, point_x1, config.MANEUVER_START_DISTANCE)
        
        point_x5 = nav.get_coordinate(point_x4, point_x3, angle, config.MANEUVER_START_DISTANCE)
        
        b_corner_bezier = compute_bezier_points(point_x3, point_x4, point_x5)
        if not add_points_to_path(path, *map(lambda gps_point: [gps_point, SI_speed_fwd], b_corner_bezier)):
            raise RuntimeError(
                "Failed to add B corner's bezier curve to path. This expected never to happen.")
            
        point_x6 = nav.get_point_on_vector(point_x4, point_x5, -config.MANEUVER_START_DISTANCE)
        
        path.append([point_x6,SI_speed_rev])
        
        point_x7 = nav.get_coordinate(b, c, angle, config.SPIRAL_SIDES_INTERVAL*sign_current_half)
        
        path.append([point_x7,SI_speed_fwd])
        
        if nav.get_deviation(a,d,c)[0] < config.SPIRAL_SIDES_INTERVAL:
            break
        
        c, b, a, d = point_x7, point_x6, d, a
        
    return path

def build_bezier_path(abcd_points: list,
                      nav: navigation.GPSComputing,
                      logger: utility.Logger,
                      SI_speed_fwd: float,
                      SI_speed_rev: float):
    
    """Builds spiral path to fill given ABCD field.

    Fills field's missing center with zigzag (forward-backward) movement if config.ADD_FORWARD_BACKWARD_TO_END_OF_BEZIER_PATH
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
        
    a_prev, b_prev, c_prev, d_prev= list(),list(),list(),list()
    a1, a2, b1, b2, c1, c2, d1 = list(),list(),list(),list(),list(),list(),list()

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

        a_prev, b_prev, c_prev, d_prev = a, b, c, d
        a, b, c, d = a_new, b_new, c_new, d_new
        
    if center_fill_start_point == 1: 
        print("robot is going to stop spiral movement at point A")
    if center_fill_start_point == 2: 
        print("robot is going to stop spiral movement at point D")
        
    path = build_maneuvre_path(
        [a, b, c, d],
        [a_prev, b_prev, c_prev, d_prev],
        nav,
        logger,
        SI_speed_fwd,
        SI_speed_rev,
        path
    )
    
    return path
        
    if config.ADD_FORWARD_BACKWARD_TO_END_OF_BEZIER_PATH:
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

if __name__=="__main__":
    
    logger_full = utility.Logger("./log_full.txt", append_file=False)
    nav = navigation.GPSComputing()
    field_gps_coords = utility.load_coordinates(config.INPUT_GPS_FIELD_FILE)
    
    #field_gps_coords = [field_gps_coords[1],field_gps_coords[2],field_gps_coords[3],field_gps_coords[0]]
    #field_gps_coords = [field_gps_coords[2],field_gps_coords[3],field_gps_coords[0],field_gps_coords[1]]
    #field_gps_coords = [field_gps_coords[3],field_gps_coords[0],field_gps_coords[1],field_gps_coords[2]]
    
    path_points = build_bezier_path(
        field_gps_coords,
        nav,
        logger_full,
        config.SI_SPEED_FWD,
        config.SI_SPEED_REV
    )
    
    # path_points = build_maneuvre_path(
    #     [field_gps_coords[0], field_gps_coords[1], field_gps_coords[2], field_gps_coords[3]],
    #     [field_gps_coords[0], field_gps_coords[1], field_gps_coords[2], field_gps_coords[3]],
    #     nav,
    #     logger_full,
    #     config.SI_SPEED_FWD,
    #     config.SI_SPEED_REV
    # )
    
    with open("manoeuvre_path_points.txt", "w") as f:
        for coords, value in path_points:
            line = f"{coords[0]} {coords[1]} {value}\n"
            f.write(line)        # écriture dans le fichier