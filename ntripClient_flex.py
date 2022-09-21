import socket
import sys
import datetime
import base64
import time
import errno
import rtcm3
import ssl
from optparse import OptionParser
import serial
import random
import utility
from config import config
import adapters
from ntripbrowser import NtripBrowser, UnableToConnect, ExceededTimeoutError

version = 0.1
useragent = "NTRIP MAVProxy/%.1f" % version


class NtripError(Exception):
    def __init__(self, message, inner_exception=None):
        self.message = message
        self.inner_exception = inner_exception
        self.exception_info = sys.exc_info()

    def __str__(self):
        return self.message


class NtripClient(object):
    def __init__(self,
                 user="",
                 password=None,
                 port=2101,
                 caster="",
                 mountpoint="",
                 output="",
                 baudrate=38400,
                 v2=False,
                 lat=52.3471207,
                 long=4.94634699999,
                 height=50):

        if sys.version_info.major >= 3:
            self.base64_user = str(base64.b64encode(bytearray(user, 'ascii')), 'ascii')
        self.user = user
        self.password = password
        self.port = port
        self.caster = caster
        self.mountpoint = mountpoint
        self.output = output
        self.baudrate = baudrate
        if not self.mountpoint.startswith("/"):
            self.mountpoint = "/" + self.mountpoint
        self.socket = None
        self.v2 = v2
        self.found_header = False
        self.sent_header = False
        # RTCM3 parser
        self.rtcm3 = rtcm3.RTCM3()
        self.last_id = None
        self.lat = lat
        self.long = long
        self.height = height
        self.lastSend = time.time()

        self.__gps_server = adapters.GPSUbloxAdapterProxyServer(config.NTRIP_PROXY_SERVER_HOST, config.NTRIP_PROXY_SERVER_PORT, tick_delay=config.NTRIP_PROXY_SERVER_TICK_DELAY)

    def setPosition(self, lat, long):
        self.flagN = "N"
        self.flagE = "E"
        if long > 180:
            long = (long - 360) * -1
            self.flagE = "W"
        elif (long < 0 and long >= -180):
            long = long * -1
            self.flagE = "W"
        elif long < -180:
            long = long + 360
            self.flagE = "E"
        else:
            self.long = long
        if lat < 0:
            lat = lat * -1
            self.flagN = "S"
        self.longDeg = int(long)
        self.latDeg = int(lat)
        self.longMin = (long - self.longDeg) * 60
        self.latMin = (lat - self.latDeg) * 60

    def calcultateCheckSum(self, stringToCheck):
        xsum_calc = 0
        for char in stringToCheck:
            xsum_calc = xsum_calc ^ ord(char)
        return "%02X" % xsum_calc

    def getGGABytes(self):
        r = random.randint(-9, 9) / 100000
        self.setPosition(self.lat + r, self.long + r)
        now = datetime.datetime.utcnow()
        ggaString = "GPGGA,%02d%02d%04.2f,%02d%011.8f,%1s,%03d%011.8f,%1s,1,05,0.19,+00000,M,%5.3f,M,," % \
                    (now.hour, now.minute, now.second, self.latDeg, self.latMin, self.flagN, self.longDeg, self.longMin,
                     self.flagE, self.height)
        checksum = self.calcultateCheckSum(ggaString)
        return bytes("$%s*%s\r\n" % (ggaString, checksum), 'ascii')

    def getMountPointString(self):
        token = base64.b64encode("{}:{}".format(self.user, self.password).encode('ascii')).decode('ascii')

        if self.password != None:
            mountPointString = "GET {} HTTP/1.1\r\n".format(self.mountpoint) + \
                               "User-Agent: {}\r\n".format(useragent) + \
                               "Authorization: Basic {}\r\n".format(token)
        else:
            userstr = self.user
            if sys.version_info.major >= 3:
                userstr = self.base64_user
            mountPointString = "GET %s HTTP/1.0\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (
                self.mountpoint, useragent, userstr)

        if self.v2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString += "\r\n"
        return mountPointString

    def read(self):
        if self.socket is None:
            time.sleep(0.1)
            self.connect()
            return None

        if not self.found_header:
            if not self.sent_header:
                self.sent_header = True
                time.sleep(0.1)
                mps = self.getMountPointString()
                if sys.version_info.major >= 3:
                    mps = bytearray(mps, 'ascii')
                try:
                    self.socket.sendall(mps)
                except Exception:
                    self.socket = None  # TODO is this non closed connection?
                    return None
            try:
                casterResponse = self.socket.recv(4096)
            except ssl.SSLWantReadError:
                return None
            except IOError as e:
                if e.errno == errno.EWOULDBLOCK:
                    return None
                self.socket = None  # TODO is this non closed connection?
                casterResponse = ''
            if sys.version_info.major >= 3:
                casterResponse = str(casterResponse, 'ascii')
            header_lines = casterResponse.split("\r\n")
            for line in header_lines:
                if line == "":
                    self.found_header = True
                if line.find("SOURCETABLE") != -1:
                    raise NtripError("Mount point does not exist")
                elif line.find("401 Unauthorized") != -1:
                    raise NtripError("Unauthorized request")
                elif line.find("404 Not Found") != -1:
                    raise NtripError("Mount Point does not exist")
                elif line.find("ICY 200 OK") >= 0:
                    # Request was valid
                    self.socket.sendall(self.getGGABytes())
                elif line.find("HTTP/1.0 200 OK") >= 0:
                    # Request was valid
                    self.socket.sendall(self.getGGABytes())
                elif line.find("HTTP/1.1 200 OK") >= 0:
                    # Request was valid
                    self.socket.sendall(self.getGGABytes())
            return None
        # normal data read
        while True:
            try:
                data = self.socket.recv(1)
            except ssl.SSLWantReadError:
                return None
            except IOError as e:
                if e.errno == errno.EWOULDBLOCK:
                    return None
                self.socket.close()
                self.socket = None
                return None
            except Exception:
                self.socket.close()
                self.socket = None
                return None
            if len(data) == 0:
                self.socket.close()
                self.socket = None
                return None
            if self.rtcm3.read(data):
                self.last_id = self.rtcm3.get_packet_ID()

                if time.time() - self.lastSend >= 4 and config.SEND_LOCATION_TO_NTRIP:
                    print("Envoi de la trame GPGGA...")
                    self.socket.sendall(self.getGGABytes())
                    self.lastSend = time.time()

                return self.rtcm3.get_packet()

    def connect(self):
        '''connect to NTRIP server'''
        self.sent_header = False
        self.found_header = False
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            error_indicator = sock.connect_ex((self.caster, self.port))
        except Exception:
            return False
        if error_indicator == 0:
            sock.setblocking(0)
            self.socket = sock
            self.rtcm3.reset()
            return True
        return False

    def _D2M2(self, Lat, NS, Lon, EW):
        """Traduce NMEA format ddmmss to ddmmmm"""

        Latdd = float(Lat[:2])
        Latmmmmm = float(Lat[2:])
        Latddmmmm = Latdd + (Latmmmmm / 60.0)
        if NS == 'S':
            Latddmmmm = -Latddmmmm

        Londd = float(Lon[:3])
        Lonmmmmm = float(Lon[3:])
        Londdmmmm = Londd + (Lonmmmmm / 60.0)
        if EW == 'W':
            Londdmmmm = -Londdmmmm
        return round(Latddmmmm, 7), round(Londdmmmm, 7)

    def readAndSendLoop(self):
        ser: serial.Serial = None
        try:
            ser: serial.Serial = serial.Serial(self.output, self.baudrate, timeout=10)
        except serial.SerialException:
            NtripError(f"Error to connection to '{self.output}' at {self.baudrate} !")
            exit(1)

        # TODO check if need to send this
        # Matrame = "B5 62 06 00 14 00 03 00 00 00 00 00 00 00 00 00 00 00 23 00 03 00 00 00 00 00 43 AE"
        # ser.write(bytearray.fromhex(Matrame))

        try:
            while True:
                # ntrip
                data = self.read()
                if data is not None:
                    if self.last_id in config.RTK_ID_SEND or not len(config.RTK_ID_SEND):
                        ser.write(data)

                # broadcast gps point from ublox to all proxy clients
                if ser.in_waiting > 0:
                    try:
                        read_line = ser.readline()
                        if isinstance(read_line, bytes):
                            data = str(read_line)
                            # if len(data) == 3:
                            #    print("None GNGGA or RTCM threads")
                            if "GNGGA" in data and ",,," not in data:
                                # bad string with no position data
                                # print(data)  # debug
                                data = data.split(",")
                                lati, longi = self._D2M2(data[2], data[3], data[4], data[5])
                                point_quality = data[6]
                                # , float(data[11])  # alti
                                self.__gps_server.send_nonblocking(f"{str(lati)},{str(longi)},{str(point_quality)}")
                    except KeyboardInterrupt:
                        raise KeyboardInterrupt
                    except:
                        pass

        except KeyboardInterrupt:
            print("Fermeture des connexions...")
            # TODO will not close connections properly if something else than KBI was raised
            ser.close()
            if self.socket:
                self.socket.close()

        print("Connexions fermées.")


if __name__ == '__main__':

    ntripArgs = {}

    if utility.get_ublox_address() is not None:
        ntripArgs['output'] = utility.get_ublox_address()
    else:
        ntripArgs['output'] = config.NTRIP_OUTPUT_PORT

    ntripArgs['baudrate'] = config.NTRIP_OUTPUT_BAUDRATE

    if config.SEND_LOCATION_TO_NTRIP or config.FIND_MOUNTPOINT:
        with adapters.GPSUbloxAdapter(ntripArgs['output'], ntripArgs['baudrate'], config.GPS_POSITIONS_TO_KEEP) as gps:
            try:
                pos = gps.get_fresh_position()
                ntripArgs['lat'] = pos[0]
                ntripArgs['long'] = pos[1]
                print(f"Current latitude : {pos[0]}, longitude : {pos[1]} for send to ntrip.")
            except Exception as e:
                print("Error not found coords ! ", e)
                exit(1)

    ntripArgs['user'] = config.NTRIP_USER
    ntripArgs['password'] = config.NTRIP_PASSWORD
    ntripArgs['caster'] = config.NTRIP_CASTER
    ntripArgs['port'] = config.NTRIP_PORT
    ntripArgs['mountpoint'] = config.NTRIP_MOUNTPOINT

    if config.FIND_MOUNTPOINT:
        browser = NtripBrowser(ntripArgs['caster'], ntripArgs['port'],
                               coordinates=(ntripArgs['lat'], ntripArgs['long']),
                               maxdist=config.MAX_DISTANCE_MOUNTPOINT)
        try:
            mountpoints = browser.get_mountpoints()["str"]
            if mountpoints:
                mountpoint = mountpoints[0]
                print(
                    f"The mountpoint '{mountpoint['Mountpoint']}' is use, it's about {int(mountpoint['Distance'])} "
                    f"kilometers from us at the coordinate [lat:{mountpoint['Latitude']},"
                    f"long:{mountpoint['Longitude']}].")
                ntripArgs['mountpoint'] = mountpoint["Mountpoint"]
            else:
                print(
                    f"Not found mountpoint {config.MAX_DISTANCE_MOUNTPOINT} kilometers around, the mountpoint in"
                    f" config named '{config.NTRIP_MOUNTPOINT}' is use.")
            sys.stdout.flush()
        except (UnableToConnect, ExceededTimeoutError):
            pass

    sys.stdout.flush()
    n = NtripClient(**ntripArgs)
    n.readAndSendLoop()
