"""Tool for parsing json file which contains points of some polygon and zipping them into one list.
Example: x = [1, 2, 3]; y = [4, 5, 6] will be converted into z = [[1, 4], [2, 5], [3, 6]]
Aso note: script will extract and use FIRST polygon and probably won't work with the rest shapes"""

import json


def main():
    with open("1.json") as file:
        data = json.loads(file.read())["_via_img_metadata"]
        for file_key in data:
            regions = data[file_key]["regions"]
            for region in regions:
                points_x = region["shape_attributes"]["all_points_x"]
                points_y = region["shape_attributes"]["all_points_y"]
                break
            break
    res = []
    for el in zip(points_x, points_y):
        res.append(list(el))
    print(res)


if __name__ == '__main__':
    main()
