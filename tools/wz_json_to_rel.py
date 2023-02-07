"""Loads via's json file which must contain working zone as polygon.

Saves converted to relative working zone as list of lists in txt file.
"""


import json

INPUT_JSON_PATH = "../1_working_zone.json"
OUTOUT_TXT_PATH = "../1_working_zone.txt"
SCENE_CENTER_X = 1000
SCENE_CENTER_Y = 980


def process_working_zone(region: dict):
    res = []
    for item in zip(region["shape_attributes"]["all_points_x"], region["shape_attributes"]["all_points_y"]):
        res.append(list(item))
    return res


def main():
    print("Make sure you have set in this script settings proper SCENE CENTERs of image which was used when making given json!")
    input("Press enter to continue:")

    with open(INPUT_JSON_PATH) as inp_json_file:
        data = json.loads(inp_json_file.read())["_via_img_metadata"]

    for file_key in data:
        regions = data[file_key]["regions"]
        for region in regions:
            if region["shape_attributes"]["name"] == "polygon":
                abs_working_zone = process_working_zone(region)
                break
        else:
            print("Couldn't finy any polygons in given json. Exiting.")
            exit()
        break

    rel_working_zone = []
    for point in abs_working_zone:
        rel_working_zone.append([
            point[0] - SCENE_CENTER_X,
            point[1] - SCENE_CENTER_Y
        ])

    with open(OUTOUT_TXT_PATH, "w") as out_txt_file:
        out_txt_file.write(str(rel_working_zone))

    print("Done!")


if __name__ == '__main__':
    main()
