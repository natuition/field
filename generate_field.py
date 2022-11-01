import utility
import navigation
from urllib.parse import quote, unquote
import os
import json
from main import build_bezier_with_corner_path
from config import config

def save_gps_coordinates(points: list, file_name: str):

    with open(file_name, "w") as file:
        for point in points:            
            if isinstance(point[0],list):
                str_point = str(point[0][0]) + " " + str(point[0][1]) + " " + str(point[1]) + "\n"
            else:
                str_point = str(point[0]) + " " + str(point[1]) + "\n"
            file.write(str_point)

class FeatureType:
    POLYGON = 0
    LINE = 1

def create_geojson_feature(points: list, name: str, type: FeatureType):
    data = dict()
    data["type"] = "Feature"
    data["geometry"] = dict()
    data["properties"] = dict()
    data["properties"]["name"] = name
    

    if type == FeatureType.POLYGON:
        data["properties"]["fill"] = "red"
        data["properties"]["fill-opacity"] = 0.3
        data["properties"]["stroke"] = "#E70000"
        data["properties"]["stroke-width"] = 2
        data["properties"]["stroke-opacity"] = 0.7

        data["geometry"]["type"] = "Polygon"
        data["geometry"]["coordinates"] = list()
        data["geometry"]["coordinates"].append(points)
        data["geometry"]["coordinates"][0].append(points[0])

    elif type == FeatureType.LINE:
        data["properties"]["stroke"] = "#00eeff"
        data["properties"]["stroke-width"] = 2
        data["properties"]["stroke-opacity"] = 1
        data["geometry"]["type"] = "LineString"
        data["geometry"]["coordinates"] = list(points)

    return data

def create_geojson_feature_collection():
    data = dict()
    data["type"] = "FeatureCollection"
    data["features"] = list()
    return data

def save_gps_coordinates_geojson(points: list, file_name: str):
    collection = create_geojson_feature_collection()
    field_i = 1
    line_i = 1
    for point in points:
        if len(point) > 2 and len(point) < 5:
            collection["features"].append(create_geojson_feature(point,f"field_{field_i}",FeatureType.POLYGON))
            field_i+=1
        else:
            collection["features"].append(create_geojson_feature(point,f"line_{line_i}",FeatureType.LINE))
            line_i+=1

    with open(file_name, 'w') as outfile:
        json.dump(collection, outfile, indent=4)
    os.chown(file_name, -1, -1)

def create_field(nav: navigation.GPSComputing,length_field: float, width_field: float):
    a_point = [46.15763479582952, -1.13396555185318]
    tmp_point = [46.15720608290609, -1.1349992081522942]
    b_point = nav.get_coordinate(a_point, tmp_point, 0, length_field)
    c_point = nav.get_coordinate(b_point, a_point, 90, width_field)
    d_point = nav.get_coordinate(c_point, b_point, 90, length_field)
    return [a_point, b_point], [b_point, c_point, d_point, a_point]

def reverse_element_list(*args: list):
    new = list()
    for lists in args:
        new.append([li[::-1] for li in lists])
    return new

def main():

    length_field = 25000
    width_field = 25000
    nav = navigation.GPSComputing()
    path_robot, field = create_field(nav, length_field, width_field)
    save_gps_coordinates(field,"./fields/field_1.txt")
    field, path_robot = reverse_element_list(field, path_robot)
    save_gps_coordinates_geojson([field, path_robot], "./fields/fields.json")

def test_field_generate():

    nav = navigation.GPSComputing()
    logger_full = utility.Logger("./fields/test_generate_path.log", append_file=False)

    path = "./fields/test_generate_field/"

    utility.create_directories(path)

    for index, field_test in enumerate([[30000, 30000],[35000, 30000],[25000, 25000],[30000, 35000],[30000, 20000],[20000, 30000]]):
        length_field = field_test[0]
        width_field = field_test[1]

        total_path = path+f"field_{index+1}_l-{length_field}_w-{width_field}/"
        utility.create_directories(total_path)
        
        _, field = create_field(nav, length_field, width_field)
        path_points = build_bezier_with_corner_path(field, nav, logger_full, config.SI_SPEED_FWD, config.SI_SPEED_REV)
        save_gps_coordinates(field, total_path+f"field_{index+1}.txt")
        save_gps_coordinates_geojson([field], total_path+f"field_{index+1}.json")
        save_gps_coordinates(path_points, total_path+f"current_path_points_field_{index+1}.txt")

if __name__ == "__main__":
    #main()
    test_field_generate()