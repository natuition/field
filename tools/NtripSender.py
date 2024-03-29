"""
 NTRIP client module
 based on client from http://github.com/jcmb/NTRIP
"""

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

version = 0.1
useragent = "NTRIP MAVProxy/%.1f" % version
debug = True


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
                 port=2101,
                 caster="",
                 mountpoint="",
                 device="",
                 bauderate="",
                 lat=46,
                 lon=122,
                 height=1212,
                 host=False,
                 ssl=False,
                 V2=False,
                 ):
        if sys.version_info.major >= 3:
            user = bytearray(user, 'ascii')
        self.user = base64.b64encode(user)
        self.port = port
        self.caster = caster
        self.mountpoint = mountpoint
        self.device = device
        self.bauderate = bauderate
        if not self.mountpoint.startswith("/"):
            self.mountpoint = "/" + self.mountpoint
        self.setPosition(lat, lon)
        self.height = height
        self.ssl = ssl
        self.host = host
        self.V2 = V2
        self.socket = None
        self.found_header = False
        self.sent_header = False
        # RTCM3 parser
        self.rtcm3 = rtcm3.RTCM3()
        self.last_id = None
        if self.port == 443:
            # force SSL on port 443
            self.ssl = True

    def setPosition(self, lat, lon):
        self.flagN = "N"
        self.flagE = "E"
        if lon > 180:
            lon = (lon-360)*-1
            self.flagE = "W"
        elif lon < 0 and lon >= -180:
            lon = lon*-1
            self.flagE = "W"
        elif lon < -180:
            lon = lon+360
            self.flagE = "E"
        else:
            self.lon = lon
        if lat < 0:
            lat = lat*-1
            self.flagN = "S"
        self.lonDeg = int(lon)
        self.latDeg = int(lat)
        self.lonMin = (lon-self.lonDeg)*60
        self.latMin = (lat-self.latDeg)*60

    def getMountPointString(self):
        userstr = self.user
        if sys.version_info.major >= 3:
            userstr = str(userstr, 'ascii')
        mountPointString = "GET %s HTTP/1.0\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (
            self.mountpoint, useragent, userstr)
        if self.host or self.V2:
            hostString = "Host: %s:%i\r\n" % (self.caster, self.port)
            mountPointString += hostString
        if self.V2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString += "\r\n"
        return mountPointString

    def getGGAString(self):
        now = datetime.datetime.utcnow()
        ggaString = "GPGGA,%02d%02d%04.2f,%02d%011.8f,%1s,%03d%011.8f,%1s,1,05,0.19,+00400,M,%5.3f,M,," % (
                     now.hour, now.minute, now.second, self.latDeg, self.latMin, self.flagN,
                     self.lonDeg, self.lonMin, self.flagE, self.height)
        checksum = self.calculateCheckSum(ggaString)
        return "$%s*%s\r\n" % (ggaString, checksum)

    def calculateCheckSum(self, stringToCheck):
        xsum_calc = 0
        for char in stringToCheck:
            xsum_calc = xsum_calc ^ ord(char)
        return "%02X" % xsum_calc

    def get_ID(self):
        '''get ID of last packet'''
        return self.last_id

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
                    self.socket = None
                    return None
            try:
                casterResponse = self.socket.recv(4096)
            except ssl.SSLWantReadError:
                    return None
            except IOError as e:
                if e.errno == errno.EWOULDBLOCK:
                    return None
                self.socket = None
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
                elif line.find(" 200 OK") != -1:
                    # Request was valid
                    try:
                        gga = self.getGGAString()
                        if sys.version_info.major >= 3:
                            gga = bytearray(gga, 'ascii')
                        self.socket.sendall(gga)
                    except Exception:
                        self.socket = None
                        return None
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
                return self.rtcm3.get_packet()

    def connect(self):
        '''connect to NTRIP server'''
        self.sent_header = False
        self.found_header = False
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if self.ssl:
            sock = ssl.wrap_socket(sock)
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

    def sendToDevice(self):
        while True:
            data = self.read()

            if data is None:
                continue

            #if self.last_id in [1005,1074,1077,1084,1087,1230]:
            if debug : 
                print(self.last_id)
            ser = serial.Serial(self.device, self.bauderate)
            ser.write(data)

if __name__ == '__main__':
    usage = "NtripSender.py [options]"
    parser = OptionParser(version="1.0", usage=usage)
    parser.add_option("-u", "--user", type="string", dest="user", default="centipede", help="Le nom de l'utilisateur du caster ntrip. Par défaut : %default.")
    parser.add_option("-r", "--password", type="string", dest="password", default="centipede", help="Le mot de passe de l'utilisateur du caster ntrip. Par défaut : %default.")
    parser.add_option("-a", "--adresse", type="string", dest="caster", default="caster.centipede.fr", help="L'adresse du caster. Par défaut : %default.")
    parser.add_option("-p", "--port", type="int", dest="port", default=2101, help="Le port de l'adresse du caster. Par défaut : %default.")
    parser.add_option("-m", "--mountpoint", type="string", dest="mountpoint", default="LIENSS", help="Le 'mountpoint' du caster. Par défaut : %default.")
    parser.add_option("-d", "--device", type="string", dest="device", default="/dev/ttyTHS1", help="L'adresse serial du gps'. Par défaut : %default.")
    parser.add_option("-b", "--bauderate", type="int", dest="bauderate", default=19200, help="Le bauderate de l'adresse serie du gps'. Par défaut : %default.")
    
    parser.add_option("-t", "--latitude", type="float", dest="lat", default=50.09, help="Votre latitude. Par défaut : %default.")
    parser.add_option("-g", "--longitude", type="float", dest="lon", default=8.66, help="Votre longitude. Par défaut : %default.")
    parser.add_option("-e", "--height", type="float", dest="height", default=1200, help="Votre ellipsoid height. Par défaut : %default.")
    parser.add_option("-2", "--V2", action="store_true", dest="V2", default=False, help="Make a NTRIP V2 Connection")

    (options, args) = parser.parse_args()
    ntripArgs = {}

    ntripArgs['lat'] = options.lat
    ntripArgs['lon'] = options.lon
    ntripArgs['height'] = options.height
    

    ntripArgs['user'] = options.user
    ntripArgs['caster'] = options.caster
    ntripArgs['port'] = options.port
    ntripArgs['mountpoint'] = options.mountpoint   

    ntripArgs['device'] = options.device  
    ntripArgs['bauderate'] = options.bauderate  

    ntripClient = NtripClient(**ntripArgs)
    ntripClient.sendToDevice()