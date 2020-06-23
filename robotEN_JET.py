# sudo python3 robot.py
# Start in Rover mode, listen M8P serial bus UBX-NAV-PVT, STATUS and SVIN threads send by the beacon to know when its Survey is finished
# and if the button to erase known field was used
# Until the position of the beacon is find in PVT thread, we check if button was used, if needed we remove the line of the
# file where was the closest position (<Dmin) and then check if this field is still known.
# If the field is know we send NMEA threads to the beacon,then wait to receive its RTCM threads before begin the treatment of the field
# If not we send RTCM threads, then wait UBX-PVT threads to record in a file to acquire perimeter of the field 


# debug mode, verb = 1
verb = 1
t_pushButt =10
Dmin = 0.2   # Max distance in kilometers from a base, maybe 0,1

#!/usr/bin/env python
import time
import serial
from math import radians, sin, cos, acos
import RPi.GPIO as GPIO

GPIO.cleanup()
# Button for test
GPIO.setmode(GPIO.BOARD)
BUTTON_PIN = 15 #GPIO22 in BCM mode
GPIO.setup(BUTTON_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.input(BUTTON_PIN)

file_data = "data.txt"
FlagErase = False

# Config of the serial port
ser = serial.Serial(
 port='/dev/ttyTHS1',
 baudrate = 19200,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 bytesize=serial.EIGHTBITS,
 timeout=0
)

i = 1
# Permit UBX OUT
def enableUBXOUT():
    Mythread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 01 00 01 00 00 00 00 00 40 1B"



# Read a byte on serial port and send its hexadecimal value ,ex : 0x3d 
def readhexa():
    while (ser.in_waiting==0):
        time.sleep(0.1)
    x = ser.read()
    if verb > 1:
        print(x)
    y=hex(int.from_bytes(x, byteorder='big', signed=False))
    return(y)

# check if 2 following bytes on serial port are an UBX thread's header (0xB5 et 0x62)
def findUBXthread():
    i=readhexa()
    if i == hex(181):
        i=readhexa()
        if i == hex(98):
            return(1)
    return(0)

# check if 2 following bytes on serial port match with class and Id of the thread UBX-NAV-SVIN (0x01 et 0x3d)
def findUCFG_SVIN():
    i=readhexa()
    j=readhexa()
    if verb == 1:
        print('')
        print("Start of the UBX thread class ", i, " and ID" , j)
    if i==hex(1) and j==hex(59):
        return(1)
    return(0)

# check if 2 following bytes on serial port match with class and Id of the thread UBX-NAV-PVT (0x01 et 0x07)
def findCFG_PVT():
    i=readhexa()
    j=readhexa()
    if verb == 1:
        print("Start of the UBX thread class ", i, " and ID" , j)
    if i==hex(1) and j==hex(7):
        return(1)
    return(0)

# check if 2 following bytes on serial port match with class and Id of the thread UBX-NAV-STATUS (0x01 et 0x03)
def findNAV_STATUS():
    i=readhexa()
    j=readhexa()
    if verb == 1:
        print('')
        print("Start of the UBX thread class ", i, " and ID" , j)
    if i==hex(1) and j==hex(3):
        return(1)
    return(0)

# calculate the size of the payload of a thread on 2 bytes
def calculsizepayload():
    i=readhexa()
    j=readhexa()
    return(256*int(j,16)+int(i,16))

# push the payload of the localsize in a readable structure
def pushpayload(localsize):
    x=0
    list=[]
    while x < localsize:
        local=readhexa()
        if verb > 1:
            print(local)
        list.append(local)
        x=x+1
    return(list)

# twos-complement for 4 bytes signed values  from I4 format of UBX protocol.ex: from 0-255 to -127-128
def translateI4(val_dec):
    import sys
    n_bytes = 4
    b = val_dec.to_bytes(n_bytes, byteorder=sys.byteorder, signed=False)                                                          
    return int.from_bytes(b, byteorder=sys.byteorder, signed=True)

# reverse twos-complement
def rev_translateI4(val_dec):
    import sys
    n_bytes = 4
    b = val_dec.to_bytes(n_bytes, byteorder=sys.byteorder, signed=True)                                                          
    return int.from_bytes(b, byteorder=sys.byteorder, signed=False)

# calculate distance between 2 GPS positions in dd.dddddddd format
def calculdistance(lLat, lLon,lLatitude, lLongitude):
    slat = radians(lLat)
    slon = radians(lLon)
    elat = radians(lLatitude)
    elon = radians(lLongitude)
    if verb == 1:
        print(lLat-lLatitude)
    return( 6371.01 * acos(sin(slat)*sin(elat) + cos(slat)*cos(elat)*cos(slon - elon)))

# change a decimal number in 4 bytes format
def decto4bytes(ltemp):
    ltemp2a,ltemp = divmod(ltemp,256*256*256)
    if verb == 1:
        print(hex(ltemp2a), ltemp)
    ltemp2b,ltemp = divmod(ltemp,256*256)
    if verb == 1:
        print(hex(ltemp2b), ltemp)
    ltemp2c,ltemp2d = divmod(ltemp,256)
    if verb == 1:
        print(hex(ltemp2c), hex(ltemp2d))
    return(' {:02x}'.format(ltemp2d)+' {:02x}'.format(ltemp2c)+' {:02x}'.format(ltemp2b)+' {:02x}'.format(ltemp2a))

# Calculate checksum (Fletcher) of a UBX thread
def checksum(lMythread):
    x=0
    ck_a=0
    ck_b=0
    if verb == 1:
        print(bytearray.fromhex(lMythread))
    lentemp=len(bytearray.fromhex(lMythread))
    
    while x < lentemp:
        ck_a = ck_a + bytearray.fromhex(lMythread)[x]
        ck_b = ck_b + ck_a
        x+=1
    if verb == 1:
        print('ck_a='+str(hex(ck_a%256)))
        print('ck_b='+str(hex(ck_b%256)))
    return(' {:02X}'.format(ck_a%256)+' {:02X}'.format(ck_b%256))

#Configuring Base with variable coordinates
def configBaseFix(lLat, lLon, lHei):
    lThread = "06 71 28 00 00 00 02 01"
    dLat = rev_translateI4(int(lLat*10000000))
    lThread = lThread + decto4bytes(dLat)
    dLon = rev_translateI4(int(lLon*10000000))
    lThread = lThread + decto4bytes(dLon)
    dHei = rev_translateI4(int(lHei*100))
    if verb == 1:
        print("lHEi and dHei", lHei, dHei)
    lThread = lThread + decto4bytes(dHei) + " 37 B3 1B 00 10 27 00 00 1E 00 00 00 F0 49 02 00 00 00 00 00 00 00 00 00"
    if verb == 1:
        print("test", lThread)
    lThread = "B5 62 " + lThread + checksum(lThread)
    if verb == 1:
        print(" ConfigBaseFix finished ",lThread)

    ser.write(bytearray.fromhex(lThread))


    #lThread = "B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB"
    #ser.write(bytearray.fromhex(lThread))

# enable RTCM out of the robot
def enabRTCM_OUT():
    if verb == 1:
        print("enabRTCM_OUT")
    lThread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 01 00 20 00 00 00 00 00 5F D5" 
    ser.write(bytearray.fromhex(lThread))
    #lThread = "B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB"
    #ser.write(bytearray.fromhex(lThread))

# enable NMEA out of the robot
def enabNMEA_OUT_RTCM_IN():
    if verb == 1:
        print("enabNMEA_OUT_RTCM_IN")
    lThread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 20 00 02 00 00 00 00 00 60 19"
    ser.write(bytearray.fromhex(lThread))
    #lThread = "B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB"
    #ser.write(bytearray.fromhex(lThread))

# configure robot in rover mode
def configRover():
    if verb == 1:
        print("configRover")
    lThread = "B5 62 06 71 28 00 00 00 00 01 FC 97 87 18 EF 62 55 FF FC 1B 00 00 37 B3 1B 00 10 27 00 00 1E 00 00 00 F0 49 02 00 00 00 00 00 00 00 00 00 23 DE"
    ser.write(bytearray.fromhex(lThread))

# enable UBX RTCM and NMEA IN and NMEA OUT on the robot
def enabUBXRTCMNMEA_INOUT():
    if verb == 1:
        print("enabUBXRTCMNMEA_INOUT")
    lThread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 23 00 02 00 00 00 00 00 63 31"
    ser.write(bytearray.fromhex(lThread))

# enable PVT thread to be send by the radio link
def enabpollingPVT():
    if verb == 1:
        print("enabpollingPVT")
    lThread = "B5 62 06 01 08 00 01 07 00 01 00 01 00 00 19 E4"
    ser.write(bytearray.fromhex(lThread))

# disable PVT thread to be send by the radio link
def stoppollingPVT():
    if verb == 1:
        print("stoppollingPVT")
    lThread = "B5 62 06 01 08 00 01 07 00 00 00 00 00 00 17 DC" 
    ser.write(bytearray.fromhex(lThread))

# enable NMEA thread to be send by USB
def enabUSBNMEA():
    if verb == 1:
        print("enableNMEA on USB")
    lThread = "B5 62 06 00 14 00 03 00 00 00 00 00 00 00 00 00 00 00 01 00 02 00 00 00 00 00 20 98" 
    ser.write(bytearray.fromhex(lThread))
    
###########################################################################################################################
#
###########################################################################################################################

################## Waiting for the start of Survey
flagSurvey = 0x0
if verb == 1:
    print("***** Waiting Survey start and check if not already ended*****\n . = None thread UBX", flagSurvey)
   
while flagSurvey == 0x0:
    if findUBXthread():
        if findUCFG_SVIN():
            size = calculsizepayload()
            payload = pushpayload(size)
            if (payload[37]=='0x1' or payload[36]=='0x1'):
                flagSurvey = 1
            if verb == 1:
                print('')
                print("Survey start or still ended ? 0 or 1 :", flagSurvey)
        else:
            if verb == 1:
                print("None SVIN thread ")
    else:
        if verb == 1:
            print(".",end='')

if verb == 1:
    print("Survey start...", flagSurvey)

##################  Checking running Survey
if verb == 1:
    print("***** Checking erase button *****\n .= None thread UBX")
ser.flushInput()
t=time.time()
while(time.time()<t_pushButt+t):
    if findUBXthread():
        if findNAV_STATUS():
            FlagErase = True
            if verb == 1:
                print("***** STATUS thread received, position will be removed *****")
        else:
            if verb == 1:
                print("None thread STATUS")
    else:
        if verb == 1:
            print(".",end='')

print("***** Waiting end of Survey *****\n .= None thread UBX")
ser.flushInput()
while 1:      
    if findUBXthread():
        if findUCFG_SVIN():
            size = calculsizepayload()
            payload = pushpayload(size)
            if payload[37]=='0x1': 
                if verb == 1:
                    print("Survey running...")
            else: 
                if verb == 1:
                    print("Survey ended")
                break
        else:
            if verb == 1:
                print("None thread SVIN")
    else:
        if verb == 1:
            print(".",end='')

              
# Reading detected position
print("***** Reading detected position *****\n .= None thread UBX")
ser.flushInput()
while 1:
    if findUBXthread():
        if findCFG_PVT():
            size = calculsizepayload()
            payload = pushpayload(size)
            if verb == 1:
                print("Base Position (in dd.ddddddd) ") 
            
            meanX= int(payload[24],16)+256*int(payload[25],16)+256*256*int(payload[26],16)+256*256*256*int(payload[27],16)
            if verb > 1:
                print(meanX)
            Longitude = translateI4(meanX)/10000000
            if verb == 1:
                print(" Longitude : ", Longitude )
            
            meanY= int(payload[28],16)+256*int(payload[29],16)+256*256*int(payload[30],16)+256*256*256*int(payload[31],16)
            if verb > 1:
                print(meanY)
            Latitude = translateI4(meanY)/10000000
            if verb == 1:
                print(" Latitude : ",Latitude)
            
            meanZ= int(payload[32],16)+256*int(payload[33],16)+256*256*int(payload[34],16)+256*256*256*int(payload[35],16)
            Height = translateI4(meanZ)/10000000
            if verb == 1:
                print(" Height : ",Height)
            break
        else:
            if verb == 1:
                print("None PVT thread")
    else:
        if verb == 1:
            print(".",end='')

"""
Latitude =  40.1539453
Longitude = -1.1181329
Height = 71.642
"""

# Comparison with known positions in data file                      

""" Erase button was used, we read the positions in the file data, write them again in the file but the one which had to
 be removed """

if FlagErase == True:
    print("***** erasing position *****")
    fileIn = open(file_data, "r")
    fileOut = open(file_data, "w")
    flagPositionfunded = 0

    with fileIn, fileOut: 
        for line in fileIn: 
            Position=[]
            """for x in line.split(' '):
                Position.append(float(x))
            Lat = Position[0]
            Lon = Position[1]
            Hei = Position[2]"""
            for x in line.split(' '):
                Position.append(x)
            Lat = float(Position[0])
            Lon = float(Position[1])
            Hei = float(Position[2])
            if verb == 1:
                print(Lat, Lon, Hei)
            dist= calculdistance(Lat, Lon, Latitude, Longitude)
            if verb == 1:
                print("Distance : %.2fkm." % dist)
            if dist<Dmin:
                if verb == 1:
                    print("Known position detected")
                continue
            fileOut.write(line)
    if verb == 1:
        print("***** End of erasing *****")

file = open(file_data, "r")
flagPositionfunded = 0

if verb == 1:
    print("***** Compare position to file *****")
for line in file:
    Position=[]
    """for x in line.split(' '):
                Position.append(float(x))
            Lat = Position[0]
            Lon = Position[1]
            Hei = Position[2]"""
    for x in line.split(' '):
        Position.append(x)
    Lat = float(Position[0])
    Lon = float(Position[1])
    Hei = float(Position[2])
    if verb == 1:
        print(Lat, Lon, Hei)
    dist= calculdistance(Lat, Lon, Latitude, Longitude)
    if verb == 1:
        print("Distance : %.2fkm." % dist)
    if dist<Dmin:
        if verb == 1:
            print("Known position detected")
        flagPositionfunded = 1
        Latknown = Lat
        Lonknown = Lon    
        Heigknown = Hei
        Filknown = Position[3]
        CURRENT_FIELD_PATH = Filknown
        break

file.close()
if verb == 1:
    print("***** Compare position to file *****")
    print("Find a position ? ", flagPositionfunded)

exit()

if flagPositionfunded == 1:
    if verb == 1:
        print("Position known, prepare robot to work, sending position to beacon")
    configBaseFix(Latknown, Lonknown, Heigknown)
    time.sleep(1)
    enabNMEA_OUT_RTCM_IN()
    input("Attente")
    #  Checking RTCM threads
    if verb == 1:
        print("Checking RTCM received from beacon")
    while 1:       
        x=ser.readline()
        time.sleep(0.1)
        configBaseFix(Latknown, Lonknown, Heigknown)
        time.sleep(1)
        enabNMEA_OUT_RTCM_IN()
        x=str(x)
        y = x.find(r"xd3")
        print(x)
        if  y != 3:
            continue
        else:
            if verb == 1:
                print("RTCM received : " + x)
            break
    ser.reset_output_buffer()
    time.sleep(1)
    configRover()
    time.sleep(1)
    enabUBXRTCMNMEA_INOUT()
    time.sleep(1)
    enabUSBNMEA()
    if verb == 1:
        print("Ready to work")
    ################  Waiting the start of treatment function
    ################ with the file of the perimeter for this field
    # treatment(Filknown)


if flagPositionfunded == 0:
    if verb == 1:
        print("Position unknown,prepare to save contour")
    configBaseFix(Latitude, Longitude, Height)
    time.sleep(1)
    enabRTCM_OUT()
    time.sleep(1)
    #enabpollingPVT()
    time.sleep(1)

    # Recording the line for this new base, name of the file is date and hour
    t = time.time()
    file_contour = time.ctime(t) 
    text_file = open(file_data, "a")
    text_file.write(str(Latitude) + ' ' + str(Longitude) + ' ' + str(Height) + ' ' + file_contour + "\n")
    text_file.close()
 
    text_file = open(file_contour, "w")
    time.sleep(2)
    while GPIO.input(BUTTON_PIN):
        if findUBXthread():
            if findCFG_PVT():
                size = calculsizepayload()
                if verb == 1:
                    print("size", size)
                payload = pushpayload(size)
                if verb == 1:
                    print("Beacon Position (in dd.ddddddd) ") 

                meanX= int(payload[24],16)+256*int(payload[25],16)+256*256*int(payload[26],16)+256*256*256*int(payload[27],16)
                print(meanX)
                Longitude = translateI4(meanX)/10000000
                if verb == 1:
                    print(" Longitude : ", Longitude )
            
                meanY= int(payload[28],16)+256*int(payload[29],16)+256*256*int(payload[30],16)+256*256*256*int(payload[31],16)
                print(meanY)
                Latitude = translateI4(meanY)/10000000
                if verb == 1:
                    print(" Latitude : ",Latitude)
                text_file.write(str(Latitude) + ' ' + str(Longitude) + "\n")
            else:
                if verb == 1:
                    print("None PVT thread")
        else:
            if verb == 1:
                print("None UBX thread")
    text_file.close()

    CURRENT_FIELD_PATH = file_contour
