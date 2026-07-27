import base64
import datetime
import errno
import socket
import ssl
import sys
import time

import rtcm3
import serial

from logger import LoggerFactory


version = 1.0
useragent = "NTRIP Natuition/%.1f" % version


class NtripError(Exception):
    def __init__(self, message, inner_exception=None):
        self.message = message
        self.inner_exception = inner_exception
        self.exception_info = sys.exc_info()

    def __str__(self):
        return self.message


class NtripClient(object):
    def __init__(
        self,
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
        height=50,
        caster_response_decode="ascii",
        send_location_to_ntrip=True,
        rtk_id_send=None,
        ntrip_sleep_time=10,
    ):
        self.__logger = LoggerFactory.create(self.__class__.__name__)

        if sys.version_info.major >= 3:
            self.base64_user = str(
                base64.b64encode(bytearray(user, "ascii")),
                "ascii",
            )

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

        self.rtcm3 = rtcm3.RTCM3()
        self.last_id = None

        self.lat = lat
        self.long = long
        self.height = height

        self.caster_response_decode = caster_response_decode
        self.send_location_to_ntrip = send_location_to_ntrip
        self.rtk_id_send = rtk_id_send if rtk_id_send is not None else []
        self.ntrip_sleep_time = ntrip_sleep_time

        self.lastSend = time.time()

        self.__logger.info(
            "NTRIP client initialized: caster={}:{}, mountpoint={}, "
            "NTRIP v2={}, send GGA={}".format(
                self.caster,
                self.port,
                self.mountpoint,
                self.v2,
                self.send_location_to_ntrip,
            )
        )

        self.__logger.debug(
            "Initial NTRIP position: latitude={:.8f}, "
            "longitude={:.8f}, height={}".format(
                self.lat,
                self.long,
                self.height,
            )
        )

        if self.rtk_id_send:
            self.__logger.debug(
                "RTCM filters configured: {}".format(
                    self.rtk_id_send
                )
            )

    def setPosition(self, lat, long):
        self.flagN = "N"
        self.flagE = "E"

        if long > 180:
            long = (long - 360) * -1
            self.flagE = "W"

        elif long < 0 and long >= -180:
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

        self.__logger.debug(
            "NTRIP position converted to NMEA: "
            "latitude={}° {:.8f}' {}, "
            "longitude={}° {:.8f}' {}".format(
                self.latDeg,
                self.latMin,
                self.flagN,
                self.longDeg,
                self.longMin,
                self.flagE,
            )
        )

    def calcultateCheckSum(self, stringToCheck):
        xsum_calc = 0

        for char in stringToCheck:
            xsum_calc = xsum_calc ^ ord(char)

        return "%02X" % xsum_calc

    def getGGABytes(self):
        self.setPosition(self.lat, self.long)

        now = datetime.datetime.utcnow()

        ggaString = (
            "GPGGA,%02d%02d%04.2f,"
            "%02d%011.8f,%1s,"
            "%03d%011.8f,%1s,"
            "1,05,0.19,+00000,M,%5.3f,M,,"
        ) % (
            now.hour,
            now.minute,
            now.second,
            self.latDeg,
            self.latMin,
            self.flagN,
            self.longDeg,
            self.longMin,
            self.flagE,
            self.height,
        )

        checksum = self.calcultateCheckSum(ggaString)

        gga_bytes = bytes(
            "$%s*%s\r\n" % (ggaString, checksum),
            "ascii",
        )

        self.__logger.debug(
            "Generated NTRIP GGA sentence: {}".format(
                gga_bytes.decode("ascii").strip()
            )
        )

        return gga_bytes

    def getMountPointString(self):
        token = base64.b64encode(
            "{}:{}".format(
                self.user,
                self.password,
            ).encode("ascii")
        ).decode("ascii")

        if self.password is not None:
            mountPointString = (
                "GET {} HTTP/1.1\r\n".format(self.mountpoint)
                + "User-Agent: {}\r\n".format(useragent)
                + "Authorization: Basic {}\r\n".format(token)
            )

        else:
            userstr = self.user

            if sys.version_info.major >= 3:
                userstr = self.base64_user

            mountPointString = (
                "GET %s HTTP/1.0\r\n"
                "User-Agent: %s\r\n"
                "Authorization: Basic %s\r\n"
            ) % (
                self.mountpoint,
                useragent,
                userstr,
            )

        if self.v2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"

        mountPointString += "\r\n"

        return mountPointString

    def read(self):
        if self.socket is None:
            self.__logger.debug(
                "No active NTRIP socket, attempting connection."
            )

            time.sleep(0.1)

            if not self.connect():
                self.__logger.warning(
                    "Unable to connect to NTRIP caster {}:{}.".format(
                        self.caster,
                        self.port,
                    )
                )

            return None

        if not self.found_header:
            if not self.sent_header:
                self.sent_header = True

                time.sleep(0.1)

                mountpoint_request = self.getMountPointString()

                if sys.version_info.major >= 3:
                    mountpoint_request = bytearray(
                        mountpoint_request,
                        "ascii",
                    )

                try:
                    self.socket.sendall(mountpoint_request)

                    self.__logger.info(
                        "NTRIP mountpoint request sent for {}.".format(
                            self.mountpoint
                        )
                    )

                except KeyboardInterrupt:
                    raise

                except Exception as error:
                    self.__logger.warning(
                        "Failed to send NTRIP mountpoint request: "
                        "{}".format(error)
                    )

                    self.__close_socket()
                    return None

            try:
                casterResponse = self.socket.recv(4096)

            except ssl.SSLWantReadError:
                return None

            except IOError as error:
                if error.errno == errno.EWOULDBLOCK:
                    return None

                self.__logger.warning(
                    "Error while receiving NTRIP response header: "
                    "{}".format(error)
                )

                self.__close_socket()
                return None

            except Exception as error:
                self.__logger.warning(
                    "Unexpected error while receiving NTRIP header: "
                    "{}".format(error)
                )

                self.__close_socket()
                return None

            if not casterResponse:
                self.__logger.warning(
                    "NTRIP caster closed the connection "
                    "while sending the response header."
                )

                self.__close_socket()
                return None

            if sys.version_info.major >= 3:
                casterResponse = str(
                    casterResponse,
                    self.caster_response_decode,
                )

            self.__logger.debug(
                "NTRIP caster response header: {!r}".format(
                    casterResponse
                )
            )

            header_lines = casterResponse.split("\r\n")

            for line in header_lines:
                if line == "":
                    self.found_header = True

                if "SOURCETABLE" in line:
                    self.__logger.error(
                        "NTRIP mountpoint {} does not exist; "
                        "caster returned a source table.".format(
                            self.mountpoint
                        )
                    )
                    raise NtripError(
                        "Mount point does not exist"
                    )

                elif "401 Unauthorized" in line:
                    self.__logger.error(
                        "NTRIP authentication rejected for "
                        "mountpoint {}.".format(self.mountpoint)
                    )
                    raise NtripError(
                        "Unauthorized request"
                    )

                elif "404 Not Found" in line:
                    self.__logger.error(
                        "NTRIP mountpoint {} was not found.".format(
                            self.mountpoint
                        )
                    )
                    raise NtripError(
                        "Mount Point does not exist"
                    )

                elif (
                    "ICY 200 OK" in line
                    or "HTTP/1.0 200 OK" in line
                    or "HTTP/1.1 200 OK" in line
                ):
                    self.__logger.info(
                        "Connected to NTRIP mountpoint {}.".format(
                            self.mountpoint
                        )
                    )

                    if self.send_location_to_ntrip:
                        try:
                            self.socket.sendall(
                                self.getGGABytes()
                            )

                            self.lastSend = time.time()

                            self.__logger.info(
                                "Initial GPGGA sentence sent "
                                "to NTRIP caster."
                            )

                        except Exception as error:
                            self.__logger.warning(
                                "Unable to send initial GPGGA "
                                "sentence: {}".format(error)
                            )

                            self.__close_socket()

            return None

        while True:
            try:
                data = self.socket.recv(1)

            except KeyboardInterrupt:
                raise

            except ssl.SSLWantReadError:
                return None

            except IOError as error:
                if error.errno == errno.EWOULDBLOCK:
                    return None

                self.__logger.warning(
                    "NTRIP socket read error: {}".format(
                        error
                    )
                )

                self.__close_socket()
                return None

            except Exception as error:
                self.__logger.warning(
                    "Unexpected NTRIP socket error: {}".format(
                        error
                    )
                )

                self.__close_socket()
                return None

            if len(data) == 0:
                self.__logger.warning(
                    "NTRIP caster closed the connection."
                )

                self.__close_socket()
                return None

            if self.rtcm3.read(data):
                self.last_id = self.rtcm3.get_packet_ID()
                packet = self.rtcm3.get_packet()

                self.__logger.debug(
                    "RTCM packet received: id={}, size={} bytes".format(
                        self.last_id,
                        len(packet),
                    )
                )

                if (
                    time.time() - self.lastSend >= 4
                    and self.send_location_to_ntrip
                ):
                    try:
                        self.socket.sendall(
                            self.getGGABytes()
                        )

                        self.lastSend = time.time()

                        self.__logger.debug(
                            "Periodic GPGGA sentence sent "
                            "to NTRIP caster."
                        )

                    except Exception as error:
                        self.__logger.warning(
                            "Unable to send periodic GPGGA "
                            "sentence: {}".format(error)
                        )

                        self.__close_socket()
                        return None

                return packet

    def connect(self):
        """Connect to the NTRIP caster."""

        self.sent_header = False
        self.found_header = False

        self.__logger.info(
            "Connecting to NTRIP caster {}:{}...".format(
                self.caster,
                self.port,
            )
        )

        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
        )

        try:
            error_indicator = sock.connect_ex(
                (
                    self.caster,
                    self.port,
                )
            )

        except KeyboardInterrupt:
            sock.close()
            raise

        except Exception as error:
            sock.close()

            self.__logger.warning(
                "NTRIP connection attempt failed: {}".format(
                    error
                )
            )

            return False

        if error_indicator != 0:
            sock.close()

            self.__logger.warning(
                "NTRIP connection to {}:{} failed with "
                "error code {}.".format(
                    self.caster,
                    self.port,
                    error_indicator,
                )
            )

            return False

        sock.setblocking(0)

        self.socket = sock
        self.rtcm3.reset()

        self.__logger.info(
            "TCP connection established with NTRIP caster {}:{}.".format(
                self.caster,
                self.port,
            )
        )

        return True

    def __close_socket(self):
        if self.socket is None:
            return

        try:
            self.socket.close()

        except OSError as error:
            self.__logger.debug(
                "Error while closing NTRIP socket: {}".format(
                    error
                )
            )

        finally:
            self.socket = None
            self.sent_header = False
            self.found_header = False

            self.__logger.debug(
                "NTRIP socket closed and connection state reset."
            )

    def readAndSendLoop(self):
        ser = None

        try:
            self.__logger.info(
                "Opening RTCM output serial port {} at {} baud.".format(
                    self.output,
                    self.baudrate,
                )
            )

            ser = serial.Serial(
                self.output,
                self.baudrate,
                timeout=10,
            )

            self.__logger.info(
                "RTCM output serial port opened."
            )

        except serial.SerialException as error:
            self.__logger.error(
                "Unable to open serial port '{}' at {} baud: {}".format(
                    self.output,
                    self.baudrate,
                    error,
                )
            )

            raise NtripError(
                "Error connecting to '{}' at {} baud.".format(
                    self.output,
                    self.baudrate,
                ),
                error,
            )

        list_RTCM_ID_sent = []

        try:
            while True:
                data = self.read()

                if data is None:
                    continue

                in_filters_id = False

                for filter_id in self.rtk_id_send:
                    if self.last_id in filter_id:
                        in_filters_id = True

                if (
                    not in_filters_id
                    and len(self.rtk_id_send) > 0
                ):
                    self.__logger.debug(
                        "RTCM packet {} ignored by filters.".format(
                            self.last_id
                        )
                    )
                    continue

                if self.last_id not in list_RTCM_ID_sent:
                    list_RTCM_ID_sent.append(
                        self.last_id
                    )

                    self.__logger.debug(
                        "RTCM IDs sent during current cycle: {}".format(
                            list_RTCM_ID_sent
                        )
                    )

                has_filters = True

                for filter_id in self.rtk_id_send:
                    if not (
                        len(
                            set(list_RTCM_ID_sent)
                            & set(filter_id)
                        )
                        > 0
                    ):
                        has_filters = False

                written_size = ser.write(data)

                self.__logger.debug(
                    "RTCM packet {} written to serial port "
                    "({}/{} bytes).".format(
                        self.last_id,
                        written_size,
                        len(data),
                    )
                )

                if (
                    has_filters
                    and len(self.rtk_id_send) > 0
                ):
                    self.__logger.debug(
                        "All RTCM filter groups received. "
                        "Sleeping for {} seconds.".format(
                            self.ntrip_sleep_time
                        )
                    )

                    list_RTCM_ID_sent = []
                    time.sleep(self.ntrip_sleep_time)

        except KeyboardInterrupt:
            self.__logger.info(
                "NTRIP shutdown requested."
            )

        finally:
            if ser is not None and ser.is_open:
                ser.close()

                self.__logger.info(
                    "RTCM output serial port closed."
                )

            self.__close_socket()

            self.__logger.info(
                "NTRIP connections closed."
            )