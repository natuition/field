import base64
import datetime
import errno
import socket
import ssl
import sys
import time
from navigation import GPSComputing
import rtcm3

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
        ntrip_v2=False,
        caster_encoding_format="ascii",
        send_gga_to_caster=True,
        filter_rtcm_by_id=None
    ):
        self.__logger = LoggerFactory.create(self.__class__.__name__)

        if sys.version_info.major >= 3:
            self.base64_user = str(
                base64.b64encode(bytearray(user, "ascii")),
                "ascii",
            )

        self.__user = user
        self.__password = password
        self.__port = port
        self.__caster = caster
        self.__mountpoint = mountpoint

        if not self.__mountpoint.startswith("/"):
            self.__mountpoint = "/" + self.__mountpoint

        self.__caster_socket = None
        self.__ntrip_v2 = ntrip_v2
        self.__found_header = False
        self.__sent_header = False

        self.__rtcm3_decoder = rtcm3.RTCM3()
        self.__last_packet_id = None

        self.__current_lat = None
        self.__current_long = None
        self.__current_altitude = None

        self.__caster_encoding_format = caster_encoding_format
        self.__send_location_to_ntrip = send_gga_to_caster
        self.__filter_rtcm_by_id = filter_rtcm_by_id if filter_rtcm_by_id is not None else []

        self.__last_gga_send_ts = time.time()

        self.__logger.info(
            "NTRIP client initialized: caster={}:{}, mountpoint={}, "
            "NTRIP v2={}, send GGA={}".format(
                self.__caster,
                self.__port,
                self.__mountpoint,
                self.__ntrip_v2,
                self.__send_location_to_ntrip,
            )
        )

        if self.__filter_rtcm_by_id:
            self.__logger.debug(
                "RTCM filters configured: {}".format(
                    self.__filter_rtcm_by_id
                )
            )
            
    def update_position(self, latitude, longitude, altitude):
        self.__current_lat = latitude
        self.__current_long = longitude
        self.__current_altitude = altitude

    def getGGABytes(self) -> bytes:
        if self.__current_lat is None or self.__current_long is None:
            self.__logger.warning(
                "Invalid position provided to NTRIP client: "
                "latitude={}, longitude={}".format(
                    self.__current_lat,
                    self.__current_long
                )
            )
            return
        
        now = datetime.datetime.utcnow()
        gga_bytes = GPSComputing.get_gga_with_decimal_position(now, self.__current_lat, self.__current_long, self.__current_altitude)

        self.__logger.debug(
            "Generated NTRIP GGA sentence: {}".format(
                gga_bytes.decode("ascii").strip()
            )
        )

        return gga_bytes

    def getMountPointString(self):
        token = base64.b64encode(
            "{}:{}".format(
                self.__user,
                self.__password,
            ).encode("ascii")
        ).decode("ascii")

        if self.__password is not None:
            mountPointString = (
                "GET {} HTTP/1.1\r\n".format(self.__mountpoint)
                + "User-Agent: {}\r\n".format(useragent)
                + "Authorization: Basic {}\r\n".format(token)
            )

        else:
            userstr = self.__user

            if sys.version_info.major >= 3:
                userstr = self.base64_user

            mountPointString = (
                "GET %s HTTP/1.0\r\n"
                "User-Agent: %s\r\n"
                "Authorization: Basic %s\r\n"
            ) % (
                self.__mountpoint,
                useragent,
                userstr,
            )

        if self.__ntrip_v2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"

        mountPointString += "\r\n"

        return mountPointString

    def read(self):
        if self.__caster_socket is None:
            self.__logger.debug(
                "No active NTRIP socket, attempting connection."
            )

            time.sleep(0.1)

            if not self.connect():
                self.__logger.warning(
                    "Unable to connect to NTRIP caster {}:{}.".format(
                        self.__caster,
                        self.__port,
                    )
                )

            return None

        if not self.__found_header:
            if not self.__sent_header:
                self.__sent_header = True

                time.sleep(0.1)

                mountpoint_request = self.getMountPointString()

                if sys.version_info.major >= 3:
                    mountpoint_request = bytearray(
                        mountpoint_request,
                        "ascii",
                    )

                try:
                    self.__caster_socket.sendall(mountpoint_request)

                    self.__logger.info(
                        "NTRIP mountpoint request sent for {}.".format(
                            self.__mountpoint
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
                casterResponse = self.__caster_socket.recv(4096)

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
                    self.__caster_encoding_format,
                )

            self.__logger.debug(
                "NTRIP caster response header: {!r}".format(
                    casterResponse
                )
            )

            header_lines = casterResponse.split("\r\n")

            for line in header_lines:
                if line == "":
                    self.__found_header = True

                if "SOURCETABLE" in line:
                    self.__logger.error(
                        "NTRIP mountpoint {} does not exist; "
                        "caster returned a source table.".format(
                            self.__mountpoint
                        )
                    )
                    raise NtripError(
                        "Mount point does not exist"
                    )

                elif "401 Unauthorized" in line:
                    self.__logger.error(
                        "NTRIP authentication rejected for "
                        "mountpoint {}.".format(self.__mountpoint)
                    )
                    raise NtripError(
                        "Unauthorized request"
                    )

                elif "404 Not Found" in line:
                    self.__logger.error(
                        "NTRIP mountpoint {} was not found.".format(
                            self.__mountpoint
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
                            self.__mountpoint
                        )
                    )

                    if self.__send_location_to_ntrip:
                        try:
                            self.__caster_socket.sendall(
                                self.getGGABytes()
                            )

                            self.__last_gga_send_ts = time.time()

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
                data = self.__caster_socket.recv(1)

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

            if self.__rtcm3_decoder.read(data):
                self.__last_packet_id = self.__rtcm3_decoder.get_packet_ID()
                packet = self.__rtcm3_decoder.get_packet()

                self.__logger.debug(
                    "RTCM packet received: id={}, size={} bytes".format(
                        self.__last_packet_id,
                        len(packet),
                    )
                )

                if (
                    time.time() - self.__last_gga_send_ts >= 4
                    and self.__send_location_to_ntrip
                ):
                    try:
                        self.__caster_socket.sendall(
                            self.getGGABytes()
                        )

                        self.__last_gga_send_ts = time.time()

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

        self.__sent_header = False
        self.__found_header = False

        self.__logger.info(
            "Connecting to NTRIP caster {}:{}...".format(
                self.__caster,
                self.__port,
            )
        )

        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
        )

        try:
            error_indicator = sock.connect_ex(
                (
                    self.__caster,
                    self.__port,
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
                    self.__caster,
                    self.__port,
                    error_indicator,
                )
            )

            return False

        sock.setblocking(0)

        self.__caster_socket = sock
        self.__rtcm3_decoder.reset()

        self.__logger.info(
            "TCP connection established with NTRIP caster {}:{}.".format(
                self.__caster,
                self.__port,
            )
        )

        return True

    def __close_socket(self):
        if self.__caster_socket is None:
            return

        try:
            self.__caster_socket.close()

        except OSError as error:
            self.__logger.debug(
                "Error while closing NTRIP socket: {}".format(
                    error
                )
            )

        finally:
            self.__caster_socket = None
            self.__sent_header = False
            self.__found_header = False

            self.__logger.debug(
                "NTRIP socket closed and connection state reset."
            )