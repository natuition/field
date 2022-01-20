import utility
import navigation
from urllib.parse import quote, unquote
import os
import json

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

def create_field_and_save_geojson(nav: navigation.GPSComputing, start_point: list, end_point: list, width_field: float):
    length_field = nav.get_distance(start_point,end_point)
    c_point = nav.get_coordinate(end_point, start_point, 90, width_field)
    d_point = nav.get_coordinate(c_point, end_point, 90, length_field)
    return [end_point, c_point, d_point, start_point]

def reverse_list_content(points: list):
    for point in points:
        point.reverse()

def main():
    nav = navigation.GPSComputing()
    coords_1 = [46.15763479582952, -1.13396555185318]
    coords_2 = [46.15720608290609, -1.1349992081522942]
    path_robot = [coords_1.copy(), coords_2.copy()]
    field_1 = create_field_and_save_geojson(nav, coords_1.copy(), coords_2.copy(), 22500)
    field_2 = create_field_and_save_geojson(nav, coords_1.copy(), coords_2.copy(), 55000)

    save_gps_coordinates(field_1,"./fields/field_1.txt")
    save_gps_coordinates(field_2,"./fields/field_2.txt")

    reverse_list_content(field_1)
    reverse_list_content(field_2)
    reverse_list_content(path_robot)

    save_gps_coordinates_geojson([field_1, field_2, path_robot], "./fields/fields.json")


if __name__ == "__main__":
    main()