"""Determine and save the new setting (working zone, cropped zone and scene center) into config.py"""
import os
from circle_center import scene_center_x
from circle_center import scene_center_y
from circle_center import circle_rad
import subprocess
import time

FILE = "../config/config.py"
os.rename(FILE, FILE + "~")

OUTPUT_FILE = open(FILE,"w")
INPUT_FILE = open(FILE+"~","r")

SCENE_CENTER_X = scene_center_x #1508 #constant just for SN003, will be recupered automatically after from a file
SCENE_CENTER_Y = scene_center_y #1098
SCENE_CENTER_RAD = circle_rad #not used yet but could be useful
HEIGHT = 1500
WIDTH = 2000
K = 230 #offset between the scene_center and the center of the cropping rectangle
#Values below are applied as offsets from the scene_center for the working zone points
P1_Y = 165
P2_Y = 710
P3_P4_X = 770
P3_P4_Y = 100
P5_P6_X = 660
P5_P6_Y = 580
P7_P8_X = 755
P7_P8_Y = 225
P9_P10_X = 360
P9_P10_Y = 665


def process_working_zone(scene_center_x: int, scene_center_y: int): 
    p1 = [scene_center_x,scene_center_y + P1_Y]
    p2 = [scene_center_x,scene_center_y - P2_Y]
    p3 = [scene_center_x - P3_P4_X, scene_center_y + P3_P4_Y]
    p4 = [scene_center_x + P3_P4_X, scene_center_y + P3_P4_Y]
    p5 = [scene_center_x - P5_P6_X, scene_center_y - P5_P6_Y]
    p6 = [scene_center_x + P5_P6_X, scene_center_y - P5_P6_Y]
    p7 = [scene_center_x - P7_P8_X, scene_center_y - P7_P8_Y]
    p8 = [scene_center_x + P7_P8_X, scene_center_y - P7_P8_Y]
    p9 = [scene_center_x - P9_P10_X, scene_center_y - P9_P10_Y]
    p10 = [scene_center_x + P9_P10_X, scene_center_y - P9_P10_Y]
    res = []
    res.extend([p1,p3,p7,p5,p9,p2,p10,p6,p8,p4])
    return res

def adapt_working_zone(working_zone: list, crop_w_from, crop_h_from):
    adapted_working_zone = []
    for i in range(len(working_zone)):
        adapted_working_zone.append([working_zone[i][0] - crop_w_from, working_zone[i][1] - crop_h_from])
    return adapted_working_zone

def adapt_scene_center(scene_center_x, scene_center_y, crop_w_from, crop_h_from):
    return scene_center_x - crop_w_from, scene_center_y - crop_h_from

def process_crop_values(scene_center_x: int, scene_center_y: int):#centre de scene et retourne crop_w_from, crop_w_to, crop_h_from, crop_h_to
    rectX = scene_center_x - WIDTH/2
    rectY = scene_center_y - HEIGHT/2 - K
    crop_w_from = int(rectX)
    crop_w_to = int(crop_w_from + WIDTH)
    crop_h_from = int(rectY)
    crop_h_to = int(crop_h_from + HEIGHT)
    return crop_w_from, crop_w_to, crop_h_from, crop_h_to

def main():

    #working zone
    working_zone = process_working_zone(SCENE_CENTER_X,SCENE_CENTER_Y)
    #cropped zone
    crop_w_from, crop_w_to, crop_h_from, crop_h_to = process_crop_values(SCENE_CENTER_X, SCENE_CENTER_Y) 

    # adapt settings according image crop values
    adapted_working_zone = adapt_working_zone(working_zone, crop_w_from, crop_h_from)
    adapted_scene_c_x, adapted_scene_c_y = adapt_scene_center(SCENE_CENTER_X, SCENE_CENTER_Y, crop_w_from, crop_h_from)

    # save settings to the config file
    
    for line in INPUT_FILE:
        #scene_center
        if "SCENE_CENTER_X =" in line:
            OUTPUT_FILE.write("SCENE_CENTER_X = " + str(adapted_scene_c_x) + "  # " + str(SCENE_CENTER_X) + " for uncropped\n")
        elif "SCENE_CENTER_Y =" in line:
            OUTPUT_FILE.write("SCENE_CENTER_Y = " + str(adapted_scene_c_y) + "  # " + str(SCENE_CENTER_Y) + " for uncropped\n")
        #cropped zone
        elif "CROP_W_FROM =" in line:
            OUTPUT_FILE.write("CROP_W_FROM = " + str(crop_w_from) + "\n")
        elif "CROP_W_TO =" in line:
            OUTPUT_FILE.write("CROP_W_TO = " + str(crop_w_to) + "\n")
        elif "CROP_H_FROM =" in line:
            OUTPUT_FILE.write("CROP_H_FROM = " + str(crop_h_from) + "\n")
        elif "CROP_H_TO =" in line:
            OUTPUT_FILE.write("CROP_H_TO = " + str(crop_h_to) + "\n")
        #working zone
        elif "WORKING_ZONE_POLY_POINTS =" in line:
            OUTPUT_FILE.write("WORKING_ZONE_POLY_POINTS = " + str(adapted_working_zone) + "\n")
        elif "WORKING_ZONE_UNCROPPED_POLY_POINTS =" in line:
            OUTPUT_FILE.write("WORKING_ZONE_UNCROPPED_POLY_POINTS = " + str(working_zone) + "\n")
        elif "APPLY_IMAGE_CROPPING = False" in line:
            OUTPUT_FILE.write("APPLY_IMAGE_CROPPING = True \n")
        else:
            OUTPUT_FILE.write(line)
    
    INPUT_FILE.close()
    OUTPUT_FILE.close()

        
if __name__ == '__main__':
    main()