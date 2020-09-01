# !/usr/bin/env python
import time
import serial
from math import radians, sin, cos, acos
import RPi.GPIO as GPIO
import psycopg2
import sys
import os
from systemd import journal
sys.path.append("../field/")
import navigation
from config import config

file_data = "BeaconCoords.data"
Dmin = 0.1  # Max distance in kilometers from a base, maybe 0,1
verb = 1
allTimeout = 20

# Permit UBX OUT
"""
def enableUBXOUT(ser: serial.Serial):
    Mythread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 01 00 01 00 00 00 00 00 40 1B"
    ser.write(bytearray.fromhex(Mythread))
"""

# enable NMEA out of the robot
"""
def enabNMEA_OUT_RTCM_IN(ser: serial.Serial):
    if verb >= 2:
        print("enabNMEA_OUT_RTCM_IN")
    lThread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 20 00 02 00 00 00 00 00 60 19"
    ser.write(bytearray.fromhex(lThread))
    # lThread = "B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB"
    # ser.write(bytearray.fromhex(lThread))

    # enable NMEA out RTCM + UBX INof the robot
"""

# enable UBX RTCM and NMEA IN and NMEA OUT on the robot
"""
def enabUBXRTCMNMEA_INOUT(ser: serial.Serial):
    if verb >= 2:
        print("enabUBXRTCMNMEA_INOUT")
    lThread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 23 00 02 00 00 00 00 00 63 31"
    ser.write(bytearray.fromhex(lThread))
"""

# enable PVT thread to be send by the radio link
"""
def enabpollingPVT(ser: serial.Serial):
    if verb >= 2:
        print("enabpollingPVT")
    lThread = "B5 62 06 01 08 00 01 07 00 01 00 01 00 00 19 E4"
    ser.write(bytearray.fromhex(lThread))
"""


# disable PVT thread to be send by the radio link
"""
def stoppollingPVT(ser: serial.Serial):
    if verb >= 2:
        print("stoppollingPVT")
    lThread = "B5 62 06 01 08 00 01 07 00 00 00 00 00 00 17 DC"
    ser.write(bytearray.fromhex(lThread))
"""

# Read a byte on usb port and send its hexadecimal value ,eg : 0x3d
def readhexa(usb: serial.Serial):
    startTime = time.time()
    while (usb.in_waiting == 0):
        if time.time() >= startTime + 10:
            if verb >= 2:
                print("Timeout with connection of beacon")
            return hex(0)
        time.sleep(0.1)
    x = usb.read()
    if verb >= 2:
        print("Data read on usb port : ",x)
    y = hex(int.from_bytes(x, byteorder='big', signed=False))
    return y


# check if 2 following bytes on usb port are an UBX thread's header (0xB5 et 0x62)
def findUBXthread(usb: serial.Serial) -> bool:
    read = list()
    read.append(readhexa(usb))
    read.append(readhexa(usb))
    if read[0] == hex(181) and read[1] == hex(98):
        return True
    return False


# check if 2 following bytes on usb port match with class and Id of the thread UBX-NAV-SVIN (0x01 et 0x3d)
def findUCFG_SVIN(usb: serial.Serial) -> bool:
    read = list()
    read.append(readhexa(usb))
    read.append(readhexa(usb))
    if verb >= 2:
        print("Start of the UBX thread class ", read[0], " and ID", read[1])
    if read[0] == hex(1) and read[1] == hex(59):
        return True
    return False


# check if 2 following bytes on usb port match with class and Id of the thread UBX-NAV-PVT (0x01 et 0x07)
def findCFG_PVT(usb: serial.Serial) -> bool:
    read = list()
    read.append(readhexa(usb))
    read.append(readhexa(usb))
    if verb >= 2:
        print("Start of the UBX thread class ", read[0], " and ID", read[1])
    if read[0] == hex(1) and read[1] == hex(7):
        return True
    return False


# check if 2 following bytes on usb port match with class and Id of the thread UBX-NAV-STATUS (0x01 et 0x03)
def findNAV_STATUS(usb: serial.Serial) -> bool:
    read = list()
    read.append(readhexa(usb))
    read.append(readhexa(usb))
    if verb >= 2:
        print("Start of the UBX thread class ", read[0], " and ID", read[1])
    if read[0] == hex(1) and read[1] == hex(3):
        return True
    return False


# calculate the size of the payload of a thread on 2 bytes
def calculsizepayload(usb: serial.Serial) -> int:
    read = list()
    read.append(readhexa(usb))
    read.append(readhexa(usb))
    return (256 * int(read[1], 16) + int(read[0], 16))


# push the payload of the localsize in a readable structure
def pushpayload(localsize: int, usb: serial.Serial) -> list:
    x = 0
    struct = []
    while x < localsize:
        local = readhexa(usb)
        if verb >= 2:
            print(local)
        struct.append(local)
        x = x + 1
    return struct


# twos-complement for 4 bytes signed values from I4 format of UBX protocol.ex: from 0-255 to -127-128
def translateI4(val_dec: int) -> int:
    n_bytes = 4
    b = val_dec.to_bytes(n_bytes, byteorder=sys.byteorder, signed=False)
    return int.from_bytes(b, byteorder=sys.byteorder, signed=True)


# reverse twos-complement
def rev_translateI4(val_dec: int) -> int:
    n_bytes = 4
    b = val_dec.to_bytes(n_bytes, byteorder=sys.byteorder, signed=True)
    return int.from_bytes(b, byteorder=sys.byteorder, signed=False)


# calculate distance between 2 GPS positions in dd.dddddddd format
def calculdistance(lLat: float, lLon: float, lLatitude: float, lLongitude: float) -> float:
    slat = radians(lLat)
    slon = radians(lLon)
    elat = radians(lLatitude)
    elon = radians(lLongitude)
    if verb >= 2:
        print(lLat - lLatitude)
    return (6371.01 * acos(sin(slat) * sin(elat) + cos(slat) * cos(elat) * cos(slon - elon)))


# change a decimal number in 4 bytes format
def decto4bytes(ltemp: float):
    ltemp2a, ltemp = divmod(ltemp, 256 * 256 * 256)
    if verb >= 2:
        print(hex(ltemp2a), ltemp)
    ltemp2b, ltemp = divmod(ltemp, 256 * 256)
    if verb >= 2:
        print(hex(ltemp2b), ltemp)
    ltemp2c, ltemp2d = divmod(ltemp, 256)
    if verb >= 2:
        print(hex(ltemp2c), hex(ltemp2d))
    return (' {:02x}'.format(ltemp2d) + ' {:02x}'.format(ltemp2c) + ' {:02x}'.format(ltemp2b) + ' {:02x}'.format(
        ltemp2a))


# Calculate checksum (Fletcher) of a UBX thread
def checksum(lMythread):
    x = 0
    ck_a = 0
    ck_b = 0
    if verb >= 2:
        print(bytearray.fromhex(lMythread))
    lentemp = len(bytearray.fromhex(lMythread))

    while x < lentemp:
        ck_a = ck_a + bytearray.fromhex(lMythread)[x]
        ck_b = ck_b + ck_a
        x += 1
    if verb >= 2:
        print('ck_a=' + str(hex(ck_a % 256)))
        print('ck_b=' + str(hex(ck_b % 256)))
    return (' {:02X}'.format(ck_a % 256) + ' {:02X}'.format(ck_b % 256))


# Configuring Base with variable coordinates
def configBaseFix(lLat: float, lLon: float, lHei: float, ser: serial.Serial):
    lThread = "06 71 28 00 00 00 02 01"
    dLat = rev_translateI4(int(lLat * 10000000))
    lThread = lThread + decto4bytes(dLat)
    dLon = rev_translateI4(int(lLon * 10000000))
    lThread = lThread + decto4bytes(dLon)
    dHei = rev_translateI4(int(lHei * 100))
    if verb >= 2:
        print("lHEi and dHei", lHei, dHei)
    lThread = lThread + decto4bytes(dHei) + " 37 B3 1B 00 10 27 00 00 1E 00 00 00 F0 49 02 00 00 00 00 00 00 00 00 00"
    if verb >= 2:
        print("test", lThread)
    lThread = "B5 62 " + lThread + checksum(lThread)
    if verb >= 2:
        print(" ConfigBaseFix finished ", lThread)
    ser.write(bytearray.fromhex(lThread))

    # lThread = "B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB"
    # ser.write(bytearray.fromhex(lThread))


# enable RTCM out of the robot
def enabRTCM_OUT(ser: serial.Serial):
    if verb >= 2:
        print("enabRTCM_OUT")
    lThread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 01 00 20 00 00 00 00 00 5F D5"
    ser.write(bytearray.fromhex(lThread))
    # lThread = "B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB"
    # ser.write(bytearray.fromhex(lThread))

def enabNMEAUBX_OUT_RTCMUBX_IN(ser: serial.Serial):
    if verb >= 2:
        print("enabNMEAUBX_OUT_RTCMUBX_IN")
    lThread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 23 00 03 00 00 00 00 00 64 37"
    ser.write(bytearray.fromhex(lThread))
    # lThread = "B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB"
    # ser.write(bytearray.fromhex(lThread))


# configure robot in rover mode
def configRover(ser: serial.Serial):
    if verb >= 2:
        print("configRover")
    lThread = "B5 62 06 71 28 00 00 00 00 01 FC 97 87 18 EF 62 55 FF FC 1B 00 00 37 B3 1B 00 10 27 00 00 1E 00 00 00 F0 49 02 00 00 00 00 00 00 00 00 00 23 DE"
    ser.write(bytearray.fromhex(lThread))

# enable NMEA thread to be send by USB
def enabUSBNMEA(ser: serial.Serial):
    if verb >= 2:
        print("enableNMEA on USB")
    lThread = "B5 62 06 00 14 00 03 00 00 00 00 00 00 00 00 00 00 00 01 00 02 00 00 00 00 00 20 98"
    ser.write(bytearray.fromhex(lThread))


# enable STATUS thread to be send by the radio link
def enabSTATUS(ser: serial.Serial):
    if verb >= 2:
        print("enabSTATUS")
    lThread = "B5 62 06 01 08 00 01 03 00 01 00 01 00 00 15 C8"
    ser.write(bytearray.fromhex(lThread))

def saveContourOfFieldInDatatbase(LatitudeBase: float, LongitudeBase: float, listOfContour: list, listOfCorner: list):

    connectionDatabase = None
    try:
        print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** [Database] Connection to database... (timeout in 60 seconds) *****")
        # connect to the PostgreSQL database
        connectionDatabase = psycopg2.connect(dbname="postgres", user="postgres", host="172.16.0.9", password="natuition", connect_timeout = 60)
        # create a new cursor
        cur = connectionDatabase.cursor()
		# execute the INSERT statement
        print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** [Database] Insert points of field... *****")
        cpt=1
        for coords in listOfContour:
            lat = coords[0]
            long = coords[1]
            iscorner = coords in listOfCorner
            request = "INSERT INTO field_contour(base_coordinates,contour_point,robot,isCorner) VALUES('({},{})','({},{})','SN002',{});".format(LongitudeBase,LatitudeBase,long,lat,iscorner)
            cur.execute(request)
            # commit the changes to the database
            connectionDatabase.commit()
            if verb >= 1:
                print(cpt)
            cpt+=1

        request = "INSERT INTO field_contour(base_coordinates,contour_point,robot,isBase) VALUES('({},{})','({},{})','SN002',True);".format(LongitudeBase,LatitudeBase,LongitudeBase,LatitudeBase)
        cur.execute(request)
        # commit the changes to the database
        connectionDatabase.commit()

        print("Close...")
        # close communication with the database
        cur.close()
    except (Exception, psycopg2.DatabaseError, psycopg2.OperationalError) as error:
        print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** [Database] "+ str(error) + "*****")
    finally:
        if connectionDatabase is not None:
            connectionDatabase.close()
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** [Database] Done *****")
        else:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** [Database] Error during connection. *****")


#############################################

#############################################

class RTK_Robot:

    def __init__(self):
        # Config of the USB port
        self.usb = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )
        # Config of the serial port
        self.ser = serial.Serial(
            port='/dev/ttyTHS1',
            baudrate=19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )


    #############################################

    def checkSurveyStartOrEnded(self) -> bool:
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Waiting Survey start and check if not already ended... (timeout in {} seconds) *****".format(allTimeout))

        startTime = time.time()
        while (time.time() < startTime + allTimeout):
            if findUBXthread(self.usb):
                if findUCFG_SVIN(self.usb):
                    size = calculsizepayload(self.usb)
                    payload = pushpayload(size,self.usb)
                    if (payload[37] == '0x1' or payload[36] == '0x1'):
                        if verb >= 1:
                            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Survey start or still ended. *****")
                            return True
                else:
                    if verb >= 2:
                        print("None SVIN thread ")
            else:
                if verb >= 3:
                    print(".", end='')

        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Waiting Survey timeout ! *****")
        return False


    #############################################


    # Checking erase button
    def checkButtonArePressed(self) -> bool:
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Checking erase button during 15 secondes *****")

        self.usb.flushInput()
        startTime = time.time()
        while (time.time() < startTime + 15):
            if findUBXthread(self.usb):
                if findNAV_STATUS(self.usb):
                    if verb >= 1:
                        print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Buuton pressed, position will be removed *****")
                    return True
                else:
                    if verb >= 2:
                        print("None thread STATUS")
            else:
                if verb >= 3:
                    print(".", end='')

        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Button not pressed during this ten seconds *****")
        return False


    #############################################
    # Checking satellites


    def checkBeaconHaveSatellites(self) -> bool:
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Waiting satellites of beacon... (timeout in {} seconds) *****".format(allTimeout))

        self.usb.flushInput()
        startTime = time.time()
        while (time.time() < startTime + allTimeout):
            if findUBXthread(self.usb):
                if findUCFG_SVIN(self.usb):
                    size = calculsizepayload(self.usb)
                    payload = pushpayload(size,self.usb)
                    if payload[32] == '0x0':
                        if verb >= 2:
                            print("Searching satellites...")
                    else:
                        if verb >= 1:
                            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Satellites found *****")
                        y = -1
                        while y == -1:
                            # time.sleep(0.1)
                            enabSTATUS(self.ser)
                            time.sleep(0.1)
                            x = self.ser.readline()
                            x = str(x)
                            y = x.find(r"xb5b\x05\x01\x02\x00\x06\x01")  # ACK config MSG
                            if verb >= 2:
                                print(" x = ", x)
                        time.sleep(2)
                        return True
                else:
                    if verb >= 2:
                        print("None thread SVIN")
            else:
                if verb >= 3:
                    print("!", end='')

        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Waiting satellites timeout ! *****")
        return False


    #############################################
    # Checking end of survey

    def checkBeaconFinishSurvey(self) -> bool:
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Waiting end of Survey... (timeout in {} seconds) *****".format(allTimeout))

        self.usb.flushInput()
        startTime = time.time()
        while (time.time() < startTime + allTimeout):
            if findUBXthread(self.usb):
                if findUCFG_SVIN(self.usb):
                    size = calculsizepayload(self.usb)
                    payload = pushpayload(size,self.usb)
                    if payload[37] == '0x1':
                        if verb >= 2:
                            print("Survey running...")
                    else:
                        if verb >= 1:
                            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Survey ended *****")
                            time.sleep(2)
                            return True
                else:
                    if verb >= 2:
                        print("None thread SVIN")
            else:
                if verb >= 3:
                    print(".", end='')

        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Waiting end of Survey timeout ! *****")
        return False


    #############################################
    # Reading detected position

    def getDetectedPositionOfBeacon(self) -> list:
        """
            :return coords: List of coordinates of beacon (Latitude:0,Longitude:1,Height:2)
        """
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Reading detected position... (timeout in {} seconds) *****".format(allTimeout))

        coords = list()
        self.usb.flushInput()
        startTime = time.time()
        while (time.time() < startTime + allTimeout):
            if findUBXthread(self.usb):
                if findCFG_PVT(self.usb):
                    size = calculsizepayload(self.usb)
                    payload = pushpayload(size,self.usb)
                    if verb >= 2:
                        print("Base Position (in dd.ddddddd) ")

                    #######################
                    # Calculation of latitude
                    meanY = int(payload[28], 16) + 256 * int(payload[29], 16) + 256 * 256 * int(payload[30],
                                                                                                16) + 256 * 256 * 256 * int(
                        payload[31], 16)
                    if verb >= 2:
                        print(meanY)
                    Latitude = translateI4(meanY) / 10000000
                    coords.append(Latitude)
                    if verb >= 2:
                        print(" Latitude : ", Latitude)

                    #######################
                    # Calculation of longitude
                    meanX = int(payload[24], 16) + 256 * int(payload[25], 16) + 256 * 256 * int(payload[26],
                                                                                                16) + 256 * 256 * 256 * int(
                        payload[27], 16)
                    if verb >= 2:
                        print(meanX)
                    Longitude = translateI4(meanX) / 10000000
                    coords.append(Longitude)
                    if verb >= 2:
                        print(" Longitude : ", Longitude)

                    #######################
                    # Calculation of height
                    meanZ = int(payload[32], 16) + 256 * int(payload[33], 16) + 256 * 256 * int(payload[34],
                                                                                                16) + 256 * 256 * 256 * int(
                        payload[35], 16)
                    Height = translateI4(meanZ) / 10000000
                    coords.append(Height)
                    if verb >= 2:
                        print(" Height : ", Height)

                    if verb >= 1:
                        print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Beacon position are : {}[Lat]:{}[Long]:{}[Hei] *****".format(Latitude,Longitude,Height))
                    return coords

                else:
                    if verb >= 2:
                        print("None PVT thread")
            else:
                if verb >= 3:
                    print(".", end='')



    #############################################
    # Comparison with known positions in data file

    def erasePositionIfExist(self, coords: list):
            if verb >= 1:
                print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Erasing position because user press button's beacon for the first ten seconds *****")

            fileIn = open(file_data, "r")
            fileOut = open(file_data, "w")
            flagPositionfunded = False
            with fileIn, fileOut:
                for line in fileIn:
                    Position = list()
                    for x in line.split(' '):
                        Position.append(x)
                    Lat = float(Position[0])
                    Lon = float(Position[1])
                    Hei = float(Position[2])
                    if verb >= 2:
                        print(Lat, Lon, Hei)
                    dist = calculdistance(Lat, Lon, coords[0], coords[1])
                    if verb >= 2:
                        print("Distance : %.2fkm." % dist)
                    if dist < Dmin:
                        if verb >= 1:
                            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Known position detected, removing... *****")
                        continue
                    fileOut.write(line)

            fileIn.close()
            fileOut.close()

            if verb >= 1:
                print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** End of erasing *****")


    #############################################
    # Position known

    def returnPathOfFileAndCoordsIfIsKnow(self, coords: list) -> list:
        """
            :return Position: List of coordinates of beacon and path of file (Latitude:0,Longitude:1,Height:2,PathOfFile:3)
        """
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Compare position to file *****")

        file = open(file_data, "r")
        flagPositionfunded = False
        for line in file:
            Position = []
            for x in line.split(' '):
                Position.append(x)
            Lat = float(Position[0])
            Lon = float(Position[1])
            Hei = float(Position[2])
            if verb >= 2:
                print(Lat, Lon, Hei)
            dist = calculdistance(Lat, Lon, coords[0], coords[1])
            if verb >= 2:
                print("Distance : %.2fkm." % dist)
            if dist < Dmin:
                if verb >= 1:
                    print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Known position detected *****")
                return Position

        file.close()
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Known position not detected *****")
        return None


    #############################################
    # Position known

    def positionKnow(self, beaconCoords: list):
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Position known, prepare robot to work, sending position to beacon *****")

        time.sleep(1)
        self.ser.reset_output_buffer()
        y = -1
        while y == -1:
            configBaseFix(beaconCoords[0], beaconCoords[1], beaconCoords[2], self.ser)
            time.sleep(0.1)
            x = self.ser.readline()
            x = str(x)
            y = x.find(r"xb5b\x05\x01\x02\x00\x06q")  # ACK config TMODE3
            if verb == 1:
                print(" x = ", x)

        y = -1
        # input("Attente")
        while y == -1:
            # time.sleep(0.1)
            enabNMEAUBX_OUT_RTCMUBX_IN()
            x = ser.readline()
            x = str(x)
            y = x.find(r"xb5b\x05\x01\x02\x00\x06\x00")  # ACK config PRT
            if verb == 1:
                print(" x = ", x)

                # input("Attente")
        #############################################
        #  Checking RTCM threads
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Checking RTCM received from beacon *****")
        y = -1
        while y == -1:
            x = self.usb.readline()
            time.sleep(0.1)
            if verb >= 2:
                print("Waiting RTCM : ")
            x = str(x)
            y = x.find(r"xd3\x00\x06L\xe0\x00\x88\x10\x97\xc2D\x8b\xd3\x00\x13")  # string find in an RTCM thread
            print(" x = ", x)

        if verb >= 2:
            print("RTCM received : " + x)


    #############################################
    # Config ready to work

    def configRobotReadyToWorkAndCloseUART(self):

        self.ser.reset_output_buffer()

        y = -1
        while y == -1:
            configRover(self.ser)
            time.sleep(0.1)
            x = self.ser.readline()
            x = str(x)
            y = x.find(r"xb5b\x05\x01\x02\x00\x06q")  # ACK config TMODE3
            if verb >= 2:
                print(" x = ", x)

        time.sleep(0.1)
        y = -1
        # input("Attente")
        while y == -1:
            # time.sleep(0.1)
            enabNMEAUBX_OUT_RTCMUBX_IN(self.ser)
            x = self.ser.readline()
            x = str(x)
            y = x.find(r"xb5b\x05\x01\x02\x00\x06\x00")  # ACK config PRT
            if verb >= 2:
                print(" x = ", x)

        # enabUSBNMEA()
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Ready to work *****")
        # Waiting the start of treatment function
        # with the file of the perimeter for this field
        # treatment(Filknown)

        self.usb.close()
        self.ser.close()


    #############################################
    # Position unknown

    def positionUnknow(self, beaconCoords: list) -> str:
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Position unknown,prepare to save contour *****")

        time.sleep(1)
        self.ser.reset_output_buffer()
        y = -1
        while y == -1:
            configBaseFix(beaconCoords[0], beaconCoords[1], beaconCoords[2], self.ser)
            time.sleep(0.1)
            x = self.ser.readline()
            x = str(x)
            y = x.find(r"xb5b\x05\x01\x02\x00\x06q")  # ACK config TMODE3
            if verb >= 2:
                print(" x = ", x)

        y = -1
        # input("Attente")
        while y == -1:
            # time.sleep(0.1)
            enabRTCM_OUT(self.ser)
            time.sleep(0.1)
            x = self.ser.readline()
            x = str(x)
            y = x.find(r"xb5b\x05\x01\x02\x00\x06\x00")  # ACK config PRT
            if verb >= 2:
                print(" x = ", x)

                # input("Attente")
        LatitudeBase = beaconCoords[0]
        LongitudeBase = beaconCoords[1]

        # Recording the line for this new base, name of the file is date and hour
        t = time.time()
        file_contour = time.ctime(t)
        text_file = open(file_data, "a+")
        text_file.write(str(beaconCoords[0]) + ' ' + str(beaconCoords[1]) + ' ' + str(beaconCoords[2]) + ' ' + file_contour + "\n")
        text_file.close()

        text_file = open(file_contour, "w")
        time.sleep(2)
        self.ser.reset_output_buffer()

        t=time.time()
        flagPVT = False
        isRecording = True
        while isRecording:
            # with open(file_contour, "w") as text_file:
            if findUBXthread(self.usb):
                if findCFG_PVT(self.usb):
                    # text_file = open(file_contour, "a")
                    if verb >= 2:
                        print(text_file.closed)
                    size = calculsizepayload(self.usb)
                    if verb >= 2:
                        print("size", size)
                    payload = pushpayload(size,self.usb)
                    if verb >= 2:
                        print("Beacon Position (in dd.ddddddd) ")

                    meanX = int(payload[24], 16) + 256 * int(payload[25], 16) + 256 * 256 * int(payload[26],
                                                                                                16) + 256 * 256 * 256 * int(
                        payload[27], 16)
                    if verb >= 2:
                        print(meanX)
                    Longitude = translateI4(meanX) / 10000000
                    if verb >= 2:
                        print(" Longitude : ", Longitude)

                    meanY = int(payload[28], 16) + 256 * int(payload[29], 16) + 256 * 256 * int(payload[30],
                                                                                                16) + 256 * 256 * 256 * int(
                        payload[31], 16)
                    if verb >= 2:
                        print(meanY)
                    Latitude = translateI4(meanY) / 10000000
                    if verb >= 2:
                        print(" Latitude : ", Latitude)
                    if ((LatitudeBase != Latitude) or (LongitudeBase != Longitude)):
                        text_file.write(str(Latitude) + ' ' + str(Longitude) + "\n")
                        flagPVT = True
                        text_file.flush()
                        t=time.time()
                    # text_file.close()
                    # print(text_file.closed)
                # elif findNAV_STATUS(self.usb):
                #    text_file.close
                #    print(" Fin d'enregitrement, fichier fermé ?  ", text_file.closed)
                #    exit()

                else:
                    if verb >= 2:
                        print("None PVT thread")

            else:
                if verb >= 2:
                    print("None UBX thread")
            if ((time.time()-t)>5 and (flagPVT==True)):
                while isRecording:
                    t=time.time()
                    if findUBXthread(self.usb):
                        if findNAV_STATUS(self.usb):
                            text_file.close()
                            if verb >= 2:
                                print("File closed")
                            isRecording = False
                            return file_contour
                    if ((time.time()-t)>5):
                        t=time.time()
                        break


    #############################################
    # save contour

    def saveContour(self, coordsOfBeacon: list, pathOfContourFile: str):
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Calculates four corner points... *****")

        listOfContour = list()
        with open(pathOfContourFile, "r") as readFile :
            for line in readFile.readlines():
                coords = line.split(" ")
                long = float(coords[1].rstrip('\n'))
                lat = float(coords[0])
                listOfContour.append([lat,long])

        nav = navigation.GPSComputing()
        listOfCorner = nav.corner_points(listOfContour, config.FILTER_MAX_DIST, config.FILTER_MIN_DIST)

        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Calculates four corner points finished. *****")
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Save corner in field.txt ... *****")

        fieldFile = open("../field/field.txt", "w")
        for corner in listOfCorner:
            fieldFile.write("{} {}\n".format(corner[0],corner[1]))
        fieldFile.close()

        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Save corner in field.txt finished. *****")
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Save in database... *****")

        saveContourOfFieldInDatatbase(coordsOfBeacon[0],coordsOfBeacon[1],listOfContour,listOfCorner)

        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Save in database finished. *****")

    #############################################
    # run main_steps

    def startMainSteps(self):
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Starting emergency_field... *****")
        os.system("sudo python3 v3_emergency_field.py 1")
        
        if verb >= 1:
            print(time.strftime('%H:%M:%S', time.localtime()) + " : ***** Starting main_steps... *****")
        os.system("sudo python3 main_steps.py 1")
