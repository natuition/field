import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
old_config_name = "../config/config.py"
new_config_name = "../config/config.py.bak"
os.rename(old_config_name,new_config_name)
INPUT_FILE = open(new_config_name,"r")
OUTPUT_FILE = open("../config/config.py","w")

#Verify APPLY_IMAGE_CROPPING is False before taking the picture
for line in INPUT_FILE:
    if "APPLY_IMAGE_CROPPING = True" in line:
        OUTPUT_FILE.write("APPLY_IMAGE_CROPPING = False \n")
    else:
        OUTPUT_FILE.write(line)
INPUT_FILE.close()
OUTPUT_FILE.close()

#Grayscale because half-circles requires a grayscale.
#image = 'SN007_09-06-2021.jpg'
#os.system('python3 ../make_photo_calibration.py')
image = 'calibration.jpg'

#cv2.imread(path, flag), 1 for a colored image
img = cv2.imread(image, 1)
img_origine = img.copy()
img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)#convert RGB/BGR to GRAY, Grayscale because half-circles requires a grayscale.
plt.rcParams["figure.figsize"] = (16,9) #size in inches
plt.imshow(img,cmap='gray')

#Apply a blur
img = cv2.GaussianBlur(img, (21,21), cv2.BORDER_DEFAULT)
plt.rcParams["figure.figsize"] = (16,9)
plt.imshow(img,cmap='gray')

##cv2.HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) → circles
##Hough_gradient is currently the only implemented method of detection 
##dp defines the size of this accumulator matrix relatively to the image size. The smaller is the dp
#the more accurate the circle detection is, but it can miss some degenrated circles or detect multiple circles
##minDist -> between the centers of the detected circles.
#param1 refers to the higher edge threshold that will be used by the Canny edge detector, (about sensitivity)
#param2 accumulator threshold, must be reached to trace the cercle
#Params 1 and 2 affect more reliability than accuracy but were important to detect the perfect circle without offset
#SN008 : param 1 : 40, param 2 : 20, SN003 : param 1 : 30 param 2 : 10
all_circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,0.9, 2500, param1 = 30, param2 = 10, minRadius = 1200, maxRadius = 1300)
all_circles_rounded = np.uint16(np.around(all_circles)) #around the results in unsigned integer of 16 bits


print(all_circles_rounded)
print(all_circles_rounded.shape) #return the shape of an array, The elements of the shape tuple give the lengths of the corresponding array dimensions.
print('I have found ' + str(all_circles_rounded.shape[1]) + ' circles')
#return x,y,rad
if len(all_circles_rounded) == 1:
    scene_center_x = all_circles_rounded[0][0][0]
    scene_center_y = all_circles_rounded[0][0][1]
    circle_rad = all_circles_rounded[0][0][2]
else:
    print("Attention plusieurs cercles détectés")


count = 1
for i in all_circles_rounded[0, :]:
    #cv2.circle(image, center_coordinates, radius, color, thickness)
    cv2.circle(img_origine, (i[0],i[1]),i[2],(50,200,200),5)
    #cv2.circle(img_origine,(i[0],i[1]),i[2],(255,0,0),3) 
    #cv2.putText(image, text, org, font, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
    cv2.putText(img_origine, ".", (i[0],i[1]), cv2.FONT_HERSHEY_SIMPLEX,1.1, (255,0,0), 2)
    count += 1


plt.rcParams["figure.figsize"] = (16,9)
plt.imshow(img_origine)
plt.show()
cv2.imwrite('scene_center.jpg',img_origine)

