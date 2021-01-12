from config import config
import adapters
import utility
import time
import math
import navigation
import sys


A=[46.1339853, -1.122859, '1'] #,A   # Menthe
B=[46.13419003299053, -1.1227249204669285, '1'] #,B   # Fraise

nav = navigation.GPSComputing()
        
distance = nav.get_distance(A, B)

        #perpendicular, side = nav.get_deviation(A,C,B)
print("distance AB %2.2f"%distance)
        #print("perpendicular %2.2f"%perpendicular)
 
C = nav.get_coordinate(B, A, 90, 10000)
print("C:",C)
D = nav.get_coordinate(A, B, -90, 10000)
print("D:",D)


print(A[0]," ",A[1], " ", "1st")
print(B[0]," ",B[1], " ", "2nd")
print(C[0]," ",C[1], " ", "3rd")
print(D[0]," ",D[1], " ", "4st")

