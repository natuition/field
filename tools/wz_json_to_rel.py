"""Loads via's json file which must contain working zone as polygon.

Saves converted to relative working zone as list of lists in txt file.
"""
import sys
import fileinput
import pwd
import grp
import os
sys.path.append('../')

import json
from config import config

INPUT_JSON_PATH = "./"+config.ROBOT_SN+"_wz.json"
OUTOUT_TXT_PATH = "./"+config.ROBOT_SN+"_wz.txt"

def changeConfigValue(path: str, value):
        with fileinput.FileInput("../config/config.py", inplace=True, backup='.bak') as file:
            found_key = False

            for line in file:
                # skip comments
                if line.startswith("#"):
                    print(line, end='')
                    continue

                # if key name is strictly equal
                elements = line.split("=")
                if len(elements) > 0 and elements[0].strip() == path:
                    print(path + " = " + str(value), end='\n')
                    found_key = True
                else:
                    print(line, end='')

            # if key is absent - add it to the end of the file
            if not found_key:
                print(path + " = " + str(value), end='\n')
        
        print(f"Change config value for '{path}', set at : '{value}'")

        uid = pwd.getpwnam("violette").pw_uid
        gid = grp.getgrnam("violette").gr_gid
        os.chown("../config/config.py", uid, gid)

def process_working_zone(region: dict):
    res = []
    for item in zip(region["shape_attributes"]["all_points_x"], region["shape_attributes"]["all_points_y"]):
        res.append(list(item))
    return res


def main():
    
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
            point[0] - config.SCENE_CENTER_X,
            point[1] - config.SCENE_CENTER_Y
        ])

    # with open(OUTOUT_TXT_PATH, "w") as out_txt_file:
    #    out_txt_file.write(str(rel_working_zone))

    changeConfigValue("WORKING_ZONE_POLY_POINTS_REL", rel_working_zone)
    changeConfigValue("WORKING_ZONE_POLY_POINTS", abs_working_zone)

    print("Done, config updated !")


if __name__ == '__main__':
    main()
