# sudo python3 robot.py
# Start in Rover mode, listen M8P serial bus UBX-NAV-PVT, STATUS and SVIN threads send by the beacon to know when its Survey is finished
# and if the button to erase known field was used
# Until the position of the beacon is find in PVT thread, we check if button was used, if needed we remove the line of the
# file where was the closest position (<Dmin) and then check if this field is still known.
# If the field is know we send NMEA threads to the beacon,then wait to receive its RTCM threads before begin the treatment of the field
# If not we send RTCM threads, then wait UBX-PVT threads to record in a file to acquire perimeter of the field


# debug mode, verb = 1
verb = 1
t_pushButt = 10
Dmin = 0.1  # Max distance in kilometers from a base, maybe 0,1

# !/usr/bin/env python
import time
import serial
from math import radians, sin, cos, acos
import RPi.GPIO as GPIO
import psycopg2
import sys
from systemd import journal
sys.path.append("../field/")
import navigation
from config import config

journal.send('Hello world')

GPIO.cleanup()
# Button for test
GPIO.setmode(GPIO.BOARD)
BUTTON_PIN = 15  # GPIO22 in BCM mode
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.input(BUTTON_PIN)

file_data = "data.txt"
FlagErase = False

# Config of the USB port
usb = serial.Serial(
    port='/dev/ttyUSB0',  # port='/dev/ttyS0',
    baudrate=19200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)

# Config of the USB port
ser = serial.Serial(
    port='/dev/ttyTHS1',  # port='/dev/ttyS0',
    baudrate=19200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)

i = 1


# Permit UBX OUT
def enableUBXOUT():
    Mythread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 01 00 01 00 00 00 00 00 40 1B"
    ser.write(bytearray.fromhex(Mythread))


# Read a byte on serial port and send its hexadecimal value ,ex : 0x3d
def readhexa():
    while (usb.in_waiting == 0):
        time.sleep(0.1)
    x = usb.read()
    if verb > 1:
        print(x)
        sys.stdout.flush()
    y = hex(int.from_bytes(x, byteorder='big', signed=False))
    return (y)


# check if 2 following bytes on serial port are an UBX thread's header (0xB5 et 0x62)
def findUBXthread():
    i = readhexa()
    if i == hex(181):
        i = readhexa()
        if i == hex(98):
            return (1)
    return (0)


# check if 2 following bytes on serial port match with class and Id of the thread UBX-NAV-SVIN (0x01 et 0x3d)
def findUCFG_SVIN():
    i = readhexa()
    j = readhexa()
    if verb == 1:
        print('')
        sys.stdout.flush()
        print("Start of the UBX thread class ", i, " and ID", j)
        sys.stdout.flush()
    if i == hex(1) and j == hex(59):
        return (1)
    return (0)


# check if 2 following bytes on serial port match with class and Id of the thread UBX-NAV-PVT (0x01 et 0x07)
def findCFG_PVT():
    i = readhexa()
    j = readhexa()
    if verb == 1:
        print("Start of the UBX thread class ", i, " and ID", j)
        sys.stdout.flush()
    if i == hex(1) and j == hex(7):
        return (1)
    return (0)


# check if 2 following bytes on serial port match with class and Id of the thread UBX-NAV-STATUS (0x01 et 0x03)
def findNAV_STATUS():
    i = readhexa()
    j = readhexa()
    if verb == 1:
        print('')
        sys.stdout.flush()
        print("Start of the UBX thread class ", i, " and ID", j)
        sys.stdout.flush()
    if i == hex(1) and j == hex(3):
        return (1)
    return (0)


# calculate the size of the payload of a thread on 2 bytes
def calculsizepayload():
    i = readhexa()
    j = readhexa()
    return (256 * int(j, 16) + int(i, 16))


# push the payload of the localsize in a readable structure
def pushpayload(localsize):
    x = 0
    list = []
    while x < localsize:
        local = readhexa()
        if verb > 1:
            print(local)
            sys.stdout.flush()
        list.append(local)
        x = x + 1
    return (list)


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
def calculdistance(lLat, lLon, lLatitude, lLongitude):
    slat = radians(lLat)
    slon = radians(lLon)
    elat = radians(lLatitude)
    elon = radians(lLongitude)
    if verb == 1:
        print(lLat - lLatitude)
        sys.stdout.flush()
    return (6371.01 * acos(sin(slat) * sin(elat) + cos(slat) * cos(elat) * cos(slon - elon)))


# change a decimal number in 4 bytes format
def decto4bytes(ltemp):
    ltemp2a, ltemp = divmod(ltemp, 256 * 256 * 256)
    if verb == 1:
        print(hex(ltemp2a), ltemp)
        sys.stdout.flush()
    ltemp2b, ltemp = divmod(ltemp, 256 * 256)
    if verb == 1:
        print(hex(ltemp2b), ltemp)
        sys.stdout.flush()
    ltemp2c, ltemp2d = divmod(ltemp, 256)
    if verb == 1:
        print(hex(ltemp2c), hex(ltemp2d))
        sys.stdout.flush()
    return (' {:02x}'.format(ltemp2d) + ' {:02x}'.format(ltemp2c) + ' {:02x}'.format(ltemp2b) + ' {:02x}'.format(
        ltemp2a))


# Calculate checksum (Fletcher) of a UBX thread
def checksum(lMythread):
    x = 0
    ck_a = 0
    ck_b = 0
    if verb == 1:
        print(bytearray.fromhex(lMythread))
        sys.stdout.flush()
    lentemp = len(bytearray.fromhex(lMythread))

    while x < lentemp:
        ck_a = ck_a + bytearray.fromhex(lMythread)[x]
        ck_b = ck_b + ck_a
        x += 1
    if verb == 1:
        print('ck_a=' + str(hex(ck_a % 256)))
        sys.stdout.flush()
        print('ck_b=' + str(hex(ck_b % 256)))
        sys.stdout.flush()
    return (' {:02X}'.format(ck_a % 256) + ' {:02X}'.format(ck_b % 256))


# Configuring Base with variable coordinates
def configBaseFix(lLat, lLon, lHei):
    lThread = "06 71 28 00 00 00 02 01"
    dLat = rev_translateI4(int(lLat * 10000000))
    lThread = lThread + decto4bytes(dLat)
    dLon = rev_translateI4(int(lLon * 10000000))
    lThread = lThread + decto4bytes(dLon)
    dHei = rev_translateI4(int(lHei * 100))
    if verb == 1:
        print("lHEi and dHei", lHei, dHei)
        sys.stdout.flush()
    lThread = lThread + decto4bytes(dHei) + " 37 B3 1B 00 10 27 00 00 1E 00 00 00 F0 49 02 00 00 00 00 00 00 00 00 00"
    if verb == 1:
        print("test", lThread)
        sys.stdout.flush()
    lThread = "B5 62 " + lThread + checksum(lThread)
    if verb == 1:
        print(" ConfigBaseFix finished ", lThread)
        sys.stdout.flush()
    ser.write(bytearray.fromhex(lThread))

    # lThread = "B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB"
    # ser.write(bytearray.fromhex(lThread))


# enable RTCM out of the robot
def enabRTCM_OUT():
    if verb == 1:
        print("enabRTCM_OUT")
        sys.stdout.flush()
    lThread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 01 00 20 00 00 00 00 00 5F D5"
    ser.write(bytearray.fromhex(lThread))
    # lThread = "B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB"
    # ser.write(bytearray.fromhex(lThread))


# enable NMEA out of the robot
def enabNMEA_OUT_RTCM_IN():
    if verb == 1:
        print("enabNMEA_OUT_RTCM_IN")
        sys.stdout.flush()
    lThread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 20 00 02 00 00 00 00 00 60 19"
    ser.write(bytearray.fromhex(lThread))
    # lThread = "B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB"
    # ser.write(bytearray.fromhex(lThread))

    # enable NMEA out RTCM + UBX INof the robot


def enabNMEAUBX_OUT_RTCMUBX_IN():
    if verb == 1:
        print("enabNMEAUBX_OUT_RTCMUBX_IN")
        sys.stdout.flush()
    lThread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 23 00 03 00 00 00 00 00 64 37"
    ser.write(bytearray.fromhex(lThread))
    # lThread = "B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB"
    # ser.write(bytearray.fromhex(lThread))


# configure robot in rover mode
def configRover():
    if verb == 1:
        print("configRover")
        sys.stdout.flush()
    lThread = "B5 62 06 71 28 00 00 00 00 01 FC 97 87 18 EF 62 55 FF FC 1B 00 00 37 B3 1B 00 10 27 00 00 1E 00 00 00 F0 49 02 00 00 00 00 00 00 00 00 00 23 DE"
    ser.write(bytearray.fromhex(lThread))


# enable UBX RTCM and NMEA IN and NMEA OUT on the robot
def enabUBXRTCMNMEA_INOUT():
    if verb == 1:
        print("enabUBXRTCMNMEA_INOUT")
        sys.stdout.flush()
    lThread = "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 23 00 02 00 00 00 00 00 63 31"
    ser.write(bytearray.fromhex(lThread))


# enable PVT thread to be send by the radio link
def enabpollingPVT():
    if verb == 1:
        print("enabpollingPVT")
        sys.stdout.flush()
    lThread = "B5 62 06 01 08 00 01 07 00 01 00 01 00 00 19 E4"
    ser.write(bytearray.fromhex(lThread))


# disable PVT thread to be send by the radio link
def stoppollingPVT():
    if verb == 1:
        print("stoppollingPVT")
        sys.stdout.flush()
    lThread = "B5 62 06 01 08 00 01 07 00 00 00 00 00 00 17 DC"
    ser.write(bytearray.fromhex(lThread))


# enable NMEA thread to be send by USB
def enabUSBNMEA():
    if verb == 1:
        print("enableNMEA on USB")
        sys.stdout.flush()
    lThread = "B5 62 06 00 14 00 03 00 00 00 00 00 00 00 00 00 00 00 01 00 02 00 00 00 00 00 20 98"
    ser.write(bytearray.fromhex(lThread))

def saveContourOfFieldInDatatbase(LatitudeBase, LongitudeBase, pathOfContourFile : str):

    listOfContour = list()
    with open(pathOfContourFile, "r") as readFile :
        for line in readFile.readlines():
            coords = line.split(" ")
            long = float(coords[1].rstrip('\n'))
            lat = float(coords[0])
            listOfContour.append([lat,long])

    nav = navigation.GPSComputing()
    listOfCorner = nav.corner_points(listOfContour, config.FILTER_MAX_DIST, config.FILTER_MIN_DIST)

    conn = None
    try:
        print("Connection...")
        # connect to the PostgreSQL database
        connectionDatabase = psycopg2.connect(dbname="postgres", user="postgres", host="172.16.0.9", password="natuition")
        # create a new cursor
        cur = connectionDatabase.cursor()
		# execute the INSERT statement
        print("Insert...")
        cpt=1
        for coords in listOfContour:
            lat = coords[0]
            long = coords[1]
            iscorner = coords in listOfCorner
            request = "INSERT INTO field_contour(base_coordinates,contour_point,robot,isCorner) VALUES('({},{})','({},{})','SN002',{});".format(LongitudeBase,LatitudeBase,long,lat,iscorner)      
            cur.execute(request)
            # commit the changes to the database
            connectionDatabase.commit()
            print(cpt)
            cpt+=1

        request = "INSERT INTO field_contour(base_coordinates,contour_point,robot,isBase) VALUES('({},{})','({},{})','SN002',True);".format(LongitudeBase,LatitudeBase,LongitudeBase,LatitudeBase)
        cur.execute(request)
        # commit the changes to the database
        connectionDatabase.commit()

        print("Close...")
        # close communication with the database
        cur.close()
    except (Exception, psycopg2.DatabaseError) as error:
        print("[Database] "+error)
    finally:
        if connectionDatabase is not None:
            connectionDatabase.close()
            print("Done")

###########################################################################################################################
#
###########################################################################################################################

################## Waiting for the start of Survey
flagSurvey = 0x0

if verb == 1:
    print("***** Waiting Survey start and check if not already ended *****\n . = None thread UBX", flagSurvey)
    sys.stdout.flush()

while flagSurvey == 0x0:
    if findUBXthread():
        if findUCFG_SVIN():
            size = calculsizepayload()
            payload = pushpayload(size)
            if (payload[37] == '0x1' or payload[36] == '0x1'):
                flagSurvey = 1
            if verb == 1:
                print('')
                sys.stdout.flush()
                print("Survey start or still ended ? 0 or 1 :", flagSurvey)
                sys.stdout.flush()
        else:
            if verb == 1:
                print("None SVIN thread ")
                sys.stdout.flush()
    else:
        if verb == 1:
            print(".", end='')
            sys.stdout.flush()

if verb == 1:
    print("Survey start...", flagSurvey)
    sys.stdout.flush()

##################  Checking running Survey
if verb == 1:
    print("***** Checking erase button *****\n .= None thread UBX")
    sys.stdout.flush()
usb.flushInput()
t = time.time()
while (time.time() < t_pushButt + t):
    if findUBXthread():
        if findNAV_STATUS():
            FlagErase = True
            if verb == 1:
                print("***** STATUS thread received, position will be removed *****")
                sys.stdout.flush()
        else:
            if verb == 1:
                print("None thread STATUS")
                sys.stdout.flush()
    else:
        if verb == 1:
            print(".", end='')
            sys.stdout.flush()

print("***** Waiting end of Survey *****\n .= None thread UBX")
sys.stdout.flush()
usb.flushInput()
while 1:
    if findUBXthread():
        if findUCFG_SVIN():
            size = calculsizepayload()
            payload = pushpayload(size)
            if payload[37] == '0x1':
                if verb == 1:
                    print("Survey running...")
                    sys.stdout.flush()
            else:
                if verb == 1:
                    print("Survey ended")
                    sys.stdout.flush()
                break
        else:
            if verb == 1:
                print("None thread SVIN")
                sys.stdout.flush()
    else:
        if verb == 1:
            print(".", end='')
            sys.stdout.flush()

# Reading detected position
print("***** Reading detected position *****\n .= None thread UBX")
sys.stdout.flush()
usb.flushInput()
while 1:
    if findUBXthread():
        if findCFG_PVT():
            size = calculsizepayload()
            payload = pushpayload(size)
            if verb == 1:
                print("Base Position (in dd.ddddddd) ")
                sys.stdout.flush()

            meanX = int(payload[24], 16) + 256 * int(payload[25], 16) + 256 * 256 * int(payload[26],
                                                                                        16) + 256 * 256 * 256 * int(
                payload[27], 16)
            if verb > 1:
                print(meanX)
                sys.stdout.flush()
            Longitude = translateI4(meanX) / 10000000
            if verb == 1:
                print(" Longitude : ", Longitude)
                sys.stdout.flush()

            meanY = int(payload[28], 16) + 256 * int(payload[29], 16) + 256 * 256 * int(payload[30],
                                                                                        16) + 256 * 256 * 256 * int(
                payload[31], 16)
            if verb > 1:
                print(meanY)
                sys.stdout.flush()
            Latitude = translateI4(meanY) / 10000000
            if verb == 1:
                print(" Latitude : ", Latitude)
                sys.stdout.flush()

            meanZ = int(payload[32], 16) + 256 * int(payload[33], 16) + 256 * 256 * int(payload[34],
                                                                                        16) + 256 * 256 * 256 * int(
                payload[35], 16)
            Height = translateI4(meanZ) / 10000000
            if verb == 1:
                print(" Height : ", Height)
                sys.stdout.flush()
            break
        else:
            if verb == 1:
                print("None PVT thread")
                sys.stdout.flush()
    else:
        if verb == 1:
            print(".", end='')
            sys.stdout.flush()

"""
Latitude =  46.1539453
Longitude = -1.118133
Height = 71.642
"""

# Comparison with known positions in data file

""" Erase button was used, we read the positions in the file data, write them again in the file but the one which had to
 be removed """

if FlagErase == True:
    print("***** erasing position *****")
    sys.stdout.flush()
    fileIn = open(file_data, "r")
    fileOut = open(file_data, "w")
    flagPositionfunded = 0

    with fileIn, fileOut:
        for line in fileIn:
            Position = []
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
                sys.stdout.flush()
            dist = calculdistance(Lat, Lon, Latitude, Longitude)
            if verb == 1:
                print("Distance : %.2fkm." % dist)
                sys.stdout.flush()
            if dist < Dmin:
                if verb == 1:
                    print("Known position detected")
                    sys.stdout.flush()
                continue
            fileOut.write(line)
    if verb == 1:
        print("***** End of erasing *****")
        sys.stdout.flush()

file = open(file_data, "r")
flagPositionfunded = 0

if verb == 1:
    print("***** Compare position to file *****")
    sys.stdout.flush()
for line in file:
    Position = []
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
        sys.stdout.flush()
    dist = calculdistance(Lat, Lon, Latitude, Longitude)
    if verb == 1:
        print("Distance : %.2fkm." % dist)
        sys.stdout.flush()
    if dist < Dmin:
        if verb == 1:
            print("Known position detected")
            sys.stdout.flush()
        flagPositionfunded = 1
        Latknown = Lat
        Lonknown = Lon
        Heigknown = Hei
        Filknown = Position[3]
        break

file.close()
if verb == 1:
    print("***** Compare position to file *****")
    sys.stdout.flush()
    print("Find a position ? ", flagPositionfunded)
    sys.stdout.flush()

# exit()

if flagPositionfunded == 1:
    if verb == 1:
        print("Position known, prepare robot to work, sending position to beacon")
        sys.stdout.flush()

    time.sleep(1)
    ser.reset_output_buffer()
    y = -1
    while y == -1:
        configBaseFix(Latknown, Lonknown, Heigknown)
        # time.sleep(0.1)
        x = ser.readline()
        x = str(x)
        y = x.find(r"xb5b\x05\x01\x02\x00\x06q")  # ACK config TMODE3
        if verb == 1:
            print(" x = ", x)
            sys.stdout.flush()

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
            sys.stdout.flush()

            # input("Attente")
    #  Checking RTCM threads
    if verb == 1:
        print("Checking RTCM received from beacon")
        sys.stdout.flush()
    y = -1
    while y == -1:
        x = usb.readline()
        time.sleep(0.1)
        if verb == 1:
            print("Waiting RTCM : ")
            sys.stdout.flush()
        x = str(x)
        y = x.find(r"xd3\x00\x06L\xe0\x00\x88\x10\x97\xc2D\x8b\xd3\x00\x13")  # string find in an RTCM thread
        print(" x = ", x)
        sys.stdout.flush()

    if verb == 1:
        print("RTCM received : " + x)
        sys.stdout.flush()

    time.sleep(1)
    ser.reset_output_buffer()

    y = -1
    while y == -1:
        configRover()
        # time.sleep(0.1)
        x = ser.readline()
        x = str(x)
        y = x.find(r"xb5b\x05\x01\x02\x00\x06q")  # ACK config TMODE3
        if verb == 1:
            print(" x = ", x)
            sys.stdout.flush()

    time.sleep(.1)
    # enabUSBNMEA()
    if verb == 1:
        print("Ready to work")
        sys.stdout.flush()
    ################  Waiting the start of treatment function
    ################ with the file of the perimeter for this field
    # treatment(Filknown)

isRecording = True

if flagPositionfunded == 0:
    if verb == 1:
        print("Position unknown,prepare to save contour")
        sys.stdout.flush()
    time.sleep(1)
    ser.reset_output_buffer()
    y = -1
    while y == -1:
        configBaseFix(Latitude, Longitude, Height)
        # time.sleep(0.1)
        x = ser.readline()
        x = str(x)
        y = x.find(r"xb5b\x05\x01\x02\x00\x06q")  # ACK config TMODE3
        if verb == 1:
            print(" x = ", x)
            sys.stdout.flush()

    y = -1
    # input("Attente")
    while y == -1:
        # time.sleep(0.1)
        enabRTCM_OUT()
        x = ser.readline()
        x = str(x)
        y = x.find(r"xb5b\x05\x01\x02\x00\x06\x00")  # ACK config PRT
        if verb == 1:
            print(" x = ", x)
            sys.stdout.flush()

            # input("Attente")
    LatitudeBase = Latitude
    LongitudeBase = Longitude

    # Recording the line for this new base, name of the file is date and hour
    t = time.time()
    file_contour = time.ctime(t)
    text_file = open(file_data, "a")
    text_file.write(str(Latitude) + ' ' + str(Longitude) + ' ' + str(Height) + ' ' + file_contour + "\n")
    text_file.close()

    text_file = open(file_contour, "w")
    time.sleep(2)
    ser.reset_output_buffer()

    t=time.time()
    flagPVT = False
    while isRecording:
        # with open(file_contour, "w") as text_file:
        if findUBXthread():
            if findCFG_PVT():
                # text_file = open(file_contour, "a")
                print(text_file.closed)
                sys.stdout.flush()
                size = calculsizepayload()
                if verb == 1:
                    print("size", size)
                    sys.stdout.flush()
                payload = pushpayload(size)
                if verb == 1:
                    print("Beacon Position (in dd.ddddddd) ")
                    sys.stdout.flush()

                meanX = int(payload[24], 16) + 256 * int(payload[25], 16) + 256 * 256 * int(payload[26],
                                                                                            16) + 256 * 256 * 256 * int(
                    payload[27], 16)
                print(meanX)
                sys.stdout.flush()
                Longitude = translateI4(meanX) / 10000000
                if verb == 1:
                    print(" Longitude : ", Longitude)
                    sys.stdout.flush()

                meanY = int(payload[28], 16) + 256 * int(payload[29], 16) + 256 * 256 * int(payload[30],
                                                                                            16) + 256 * 256 * 256 * int(
                    payload[31], 16)
                print(meanY)
                sys.stdout.flush()
                Latitude = translateI4(meanY) / 10000000
                if verb == 1:
                    print(" Latitude : ", Latitude)
                    sys.stdout.flush()
                if ((LatitudeBase != Latitude) or (LongitudeBase != Longitude)):
                    text_file.write(str(Latitude) + ' ' + str(Longitude) + "\n")
                    flagPVT = True
                    text_file.flush()
                    t=time.time()
                # text_file.close()
                # print(text_file.closed)
            # elif findNAV_STATUS():
            #    text_file.close
            #    print(" Fin d'enregitrement, fichier fermé ?  ", text_file.closed)
            #    exit()

            else:
                if verb == 1:
                    print("None PVT thread")
                    sys.stdout.flush()

        else:
            if verb == 1:
                print("None UBX thread")
                sys.stdout.flush()
        if ((time.time()-t)>5 and (flagPVT==True)):
            while isRecording:
                t=time.time()
                if findUBXthread():
                    if findNAV_STATUS():
                        text_file.close()
                        print("File closed")
                        sys.stdout.flush()
                        isRecording = False
                if ((time.time()-t)>5):
                    t=time.time()
                    break

if not isRecording:
    saveContourOfFieldInDatatbase(LatitudeBase,LongitudeBase,file_contour)
    fileIn.close()
    fileOut.close()
usb.close()
ser.close()

