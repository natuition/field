import base64
import datetime
import errno
import select
import socket
import ssl
import sys
import threading
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
    """Non-blocking NTRIP client designed to be polled regularly.

    ``read()`` never waits for the TCP connection, socket I/O, a reconnect
    delay, or DNS resolution. It advances an internal state machine and
    returns either one complete RTCM packet or ``None``.
    """

    DATA_TIMEOUT_SECONDS = 3.0
    CONNECT_TIMEOUT_SECONDS = 3.0
    DNS_TIMEOUT_SECONDS = 3.0
    RECONNECT_DELAY_SECONDS = 1.0
    GGA_SEND_INTERVAL_SECONDS = 1.0
    SOCKET_READ_SIZE = 4096
    MAX_HEADER_SIZE = 64 * 1024

    STATE_DISCONNECTED = "disconnected"
    STATE_RESOLVING = "resolving"
    STATE_CONNECTING = "connecting"
    STATE_CONNECTED = "connected"

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
        filter_rtcm_by_id=None,
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

        self.__ntrip_v2 = ntrip_v2
        self.__caster_encoding_format = caster_encoding_format
        self.__send_location_to_ntrip = send_gga_to_caster
        self.__filter_rtcm_by_id = (
            filter_rtcm_by_id if filter_rtcm_by_id is not None else []
        )

        self.__caster_socket = None
        self.__state = self.STATE_DISCONNECTED
        self.__resolver_generation += 1
        self.__connect_started_ts = None
        self.__next_connect_attempt_ts = 0.0

        self.__resolver_thread = None
        self.__resolver_lock = threading.Lock()
        self.__resolver_generation = 0
        self.__resolver_started_ts = None
        self.__resolver_result = None
        self.__resolver_error = None
        self.__resolved_addresses = []
        self.__resolved_address_index = 0

        self.__found_header = False
        self.__sent_header = False
        self.__header_buffer = bytearray()
        self.__receive_buffer = bytearray()
        self.__send_buffer = bytearray()

        self.__rtcm3_decoder = rtcm3.RTCM3()
        self.__last_packet_id = None

        self.__current_lat = None
        self.__current_long = None
        self.__current_altitude = None

        self.__last_gga_send_ts = time.monotonic()
        self.__last_data_received_ts = time.monotonic()

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

    @property
    def socket(self):
        """Expose the active socket for compatibility with existing cleanup."""
        return self.__caster_socket

    def close(self):
        """Close the client immediately without scheduling an automatic delay."""
        self.__close_socket(schedule_reconnect=False)

    def update_position(self, latitude, longitude, altitude=0):
        self.__current_lat = latitude
        self.__current_long = longitude
        self.__current_altitude = altitude

    def get_last_packet_id(self):
        return self.__last_packet_id

    def getGGABytes(self):
        if self.__current_lat is None or self.__current_long is None:
            self.__logger.warning(
                "Invalid position provided to NTRIP client: "
                "latitude={}, longitude={}".format(
                    self.__current_lat,
                    self.__current_long,
                )
            )
            return None

        now = datetime.datetime.utcnow()
        gga_bytes = GPSComputing.get_gga_with_decimal_position(
            now,
            self.__current_lat,
            self.__current_long,
            self.__current_altitude,
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
                self.__user,
                self.__password,
            ).encode("ascii")
        ).decode("ascii")

        if self.__password is not None:
            mountpoint_string = (
                "GET {} HTTP/1.1\r\n".format(self.__mountpoint)
                + "User-Agent: {}\r\n".format(useragent)
                + "Authorization: Basic {}\r\n".format(token)
            )
        else:
            user_string = self.__user

            if sys.version_info.major >= 3:
                user_string = self.base64_user

            mountpoint_string = (
                "GET %s HTTP/1.0\r\n"
                "User-Agent: %s\r\n"
                "Authorization: Basic %s\r\n"
            ) % (
                self.__mountpoint,
                useragent,
                user_string,
            )

        if self.__ntrip_v2:
            mountpoint_string += "Ntrip-Version: Ntrip/2.0\r\n"

        mountpoint_string += "\r\n"
        return mountpoint_string

    def read(self):
        """Advance the NTRIP state machine and return at most one RTCM packet."""
        current_ts = time.monotonic()

        if self.__state == self.STATE_DISCONNECTED:
            if current_ts >= self.__next_connect_attempt_ts:
                self.connect()
            return None

        if self.__state == self.STATE_RESOLVING:
            self.__poll_dns_resolution(current_ts)
            return None

        if self.__state == self.STATE_CONNECTING:
            self.__poll_tcp_connection(current_ts)
            return None

        if self.__state != self.STATE_CONNECTED:
            return None

        if not self.__sent_header:
            mountpoint_request = self.getMountPointString().encode("ascii")
            self.__queue_data_to_send(mountpoint_request)
            self.__sent_header = True
            self.__logger.info(
                "NTRIP mountpoint request queued for {}.".format(
                    self.__mountpoint
                )
            )

        self.__queue_periodic_gga_if_needed(current_ts)
        self.__flush_send_buffer()

        if self.__state != self.STATE_CONNECTED:
            return None

        self.__receive_available_data(current_ts)

        if self.__state != self.STATE_CONNECTED:
            return None

        if not self.__found_header:
            self.__process_response_header()

            if self.__state != self.STATE_CONNECTED or not self.__found_header:
                return None

        if (
            current_ts - self.__last_data_received_ts
            > self.DATA_TIMEOUT_SECONDS
        ):
            self.__logger.warning(
                "No NTRIP data received for more than {:.1f} seconds; "
                "reconnecting.".format(self.DATA_TIMEOUT_SECONDS)
            )
            self.__close_socket()
            return None

        self.__flush_send_buffer()
        return self.__extract_one_rtcm_packet()

    def connect(self):
        """Start a non-blocking DNS resolution/connection attempt."""
        current_ts = time.monotonic()

        if self.__state != self.STATE_DISCONNECTED:
            return True

        if current_ts < self.__next_connect_attempt_ts:
            return False

        self.__reset_protocol_state()
        self.__start_dns_resolution(current_ts)
        return True

    def __start_dns_resolution(self, current_ts):
        self.__state = self.STATE_RESOLVING
        self.__resolver_started_ts = current_ts

        with self.__resolver_lock:
            self.__resolver_result = None
            self.__resolver_error = None

        self.__logger.info(
            "Resolving NTRIP caster {}:{}...".format(
                self.__caster,
                self.__port,
            )
        )

        self.__resolver_generation += 1
        resolver_generation = self.__resolver_generation

        self.__resolver_thread = threading.Thread(
            target=self.__resolve_caster,
            args=(resolver_generation,),
            name="NtripDnsResolver",
        )
        self.__resolver_thread.daemon = True
        self.__resolver_thread.start()

    def __resolve_caster(self, resolver_generation):
        try:
            addresses = socket.getaddrinfo(
                self.__caster,
                self.__port,
                socket.AF_UNSPEC,
                socket.SOCK_STREAM,
            )

            unique_addresses = []
            seen = set()

            for family, socktype, protocol, _, sockaddr in addresses:
                key = (family, socktype, protocol, sockaddr)
                if key in seen:
                    continue
                seen.add(key)
                unique_addresses.append(
                    (family, socktype, protocol, sockaddr)
                )

            with self.__resolver_lock:
                if resolver_generation == self.__resolver_generation:
                    self.__resolver_result = unique_addresses

        except Exception as error:
            with self.__resolver_lock:
                if resolver_generation == self.__resolver_generation:
                    self.__resolver_error = error

    def __poll_dns_resolution(self, current_ts):
        with self.__resolver_lock:
            result = self.__resolver_result
            error = self.__resolver_error

        if error is not None:
            self.__logger.warning(
                "Unable to resolve NTRIP caster {}: {}".format(
                    self.__caster,
                    error,
                )
            )
            self.__close_socket()
            return

        if result is not None:
            if not result:
                self.__logger.warning(
                    "DNS resolution returned no address for {}.".format(
                        self.__caster
                    )
                )
                self.__close_socket()
                return

            self.__resolved_addresses = result
            self.__resolved_address_index = 0
            self.__start_next_tcp_connection(current_ts)
            return

        if (
            self.__resolver_started_ts is not None
            and current_ts - self.__resolver_started_ts
            > self.DNS_TIMEOUT_SECONDS
        ):
            self.__logger.warning(
                "DNS resolution for {} exceeded {:.1f} seconds; "
                "scheduling another attempt.".format(
                    self.__caster,
                    self.DNS_TIMEOUT_SECONDS,
                )
            )
            # The daemon resolver may still finish later, but its result is
            # detached from this connection cycle when state is reset.
            self.__close_socket()

    def __start_next_tcp_connection(self, current_ts):
        self.__close_active_socket_only()

        while self.__resolved_address_index < len(self.__resolved_addresses):
            family, socktype, protocol, sockaddr = self.__resolved_addresses[
                self.__resolved_address_index
            ]
            self.__resolved_address_index += 1

            sock = socket.socket(family, socktype, protocol)
            sock.setblocking(False)

            try:
                error_indicator = sock.connect_ex(sockaddr)
            except KeyboardInterrupt:
                sock.close()
                raise
            except Exception as error:
                sock.close()
                self.__logger.warning(
                    "Unable to start NTRIP TCP connection to {}: {}".format(
                        sockaddr,
                        error,
                    )
                )
                continue

            in_progress_errors = (
                0,
                errno.EINPROGRESS,
                errno.EWOULDBLOCK,
                errno.EALREADY,
                errno.EINTR,
            )

            if error_indicator not in in_progress_errors:
                sock.close()
                self.__logger.warning(
                    "NTRIP connection to {} failed immediately with "
                    "error code {}.".format(sockaddr, error_indicator)
                )
                continue

            self.__caster_socket = sock
            self.__connect_started_ts = current_ts

            if error_indicator == 0:
                self.__on_tcp_connected()
            else:
                self.__state = self.STATE_CONNECTING
                self.__logger.info(
                    "Connecting asynchronously to NTRIP caster at {}...".format(
                        sockaddr
                    )
                )
            return

        self.__logger.warning(
            "Unable to connect to any resolved address for {}:{}.".format(
                self.__caster,
                self.__port,
            )
        )
        self.__close_socket()

    def __poll_tcp_connection(self, current_ts):
        if self.__caster_socket is None:
            self.__close_socket()
            return

        if (
            self.__connect_started_ts is not None
            and current_ts - self.__connect_started_ts
            > self.CONNECT_TIMEOUT_SECONDS
        ):
            self.__logger.warning(
                "NTRIP TCP connection attempt exceeded {:.1f} seconds.".format(
                    self.CONNECT_TIMEOUT_SECONDS
                )
            )
            self.__start_next_tcp_connection(current_ts)
            return

        try:
            _, writable, exceptional = select.select(
                [],
                [self.__caster_socket],
                [self.__caster_socket],
                0,
            )
        except Exception as error:
            self.__logger.warning(
                "Unable to poll NTRIP TCP connection: {}".format(error)
            )
            self.__start_next_tcp_connection(current_ts)
            return

        if exceptional:
            self.__logger.warning(
                "NTRIP socket entered an exceptional state while connecting."
            )
            self.__start_next_tcp_connection(current_ts)
            return

        if not writable:
            return

        socket_error = self.__caster_socket.getsockopt(
            socket.SOL_SOCKET,
            socket.SO_ERROR,
        )

        if socket_error != 0:
            self.__logger.warning(
                "NTRIP TCP connection failed with error code {}.".format(
                    socket_error
                )
            )
            self.__start_next_tcp_connection(current_ts)
            return

        self.__on_tcp_connected()

    def __on_tcp_connected(self):
        self.__state = self.STATE_CONNECTED
        self.__connect_started_ts = None
        self.__last_data_received_ts = time.monotonic()
        self.__rtcm3_decoder.reset()

        self.__logger.info(
            "TCP connection established with NTRIP caster {}:{}.".format(
                self.__caster,
                self.__port,
            )
        )

    def __queue_data_to_send(self, data):
        if data:
            self.__send_buffer.extend(data)

    def __flush_send_buffer(self):
        if self.__caster_socket is None or not self.__send_buffer:
            return

        try:
            sent_size = self.__caster_socket.send(self.__send_buffer)
        except KeyboardInterrupt:
            raise
        except (ssl.SSLWantWriteError, ssl.SSLWantReadError):
            return
        except IOError as error:
            if error.errno in (
                errno.EWOULDBLOCK,
                errno.EAGAIN,
                errno.EINPROGRESS,
            ):
                return

            self.__logger.warning(
                "NTRIP socket send error: {}".format(error)
            )
            self.__close_socket()
            return
        except Exception as error:
            self.__logger.warning(
                "Unexpected NTRIP send error: {}".format(error)
            )
            self.__close_socket()
            return

        if sent_size == 0:
            self.__logger.warning(
                "NTRIP caster closed the connection while sending data."
            )
            self.__close_socket()
            return

        del self.__send_buffer[:sent_size]

    def __receive_available_data(self, current_ts):
        if self.__caster_socket is None:
            return

        try:
            data = self.__caster_socket.recv(self.SOCKET_READ_SIZE)
        except KeyboardInterrupt:
            raise
        except ssl.SSLWantReadError:
            return
        except IOError as error:
            if error.errno in (
                errno.EWOULDBLOCK,
                errno.EAGAIN,
            ):
                return

            self.__logger.warning(
                "NTRIP socket read error: {}".format(error)
            )
            self.__close_socket()
            return
        except Exception as error:
            self.__logger.warning(
                "Unexpected NTRIP socket error: {}".format(error)
            )
            self.__close_socket()
            return

        if not data:
            self.__logger.warning(
                "NTRIP caster closed the connection."
            )
            self.__close_socket()
            return

        self.__last_data_received_ts = current_ts

        if self.__found_header:
            self.__receive_buffer.extend(data)
        else:
            self.__header_buffer.extend(data)

            if len(self.__header_buffer) > self.MAX_HEADER_SIZE:
                self.__logger.warning(
                    "NTRIP response header exceeded {} bytes.".format(
                        self.MAX_HEADER_SIZE
                    )
                )
                self.__close_socket()

    def __process_response_header(self):
        separator = b"\r\n\r\n"
        separator_index = self.__header_buffer.find(separator)

        # Some old NTRIP v1 casters may terminate their status with \r\n only.
        if separator_index < 0:
            if self.__header_buffer.startswith(b"ICY 200 OK\r\n"):
                separator_index = len(b"ICY 200 OK\r\n") - len(separator)
            else:
                return

        if separator_index >= 0:
            header_end = separator_index + len(separator)
        else:
            header_end = len(b"ICY 200 OK\r\n")

        header_bytes = bytes(self.__header_buffer[:header_end])
        trailing_data = self.__header_buffer[header_end:]
        self.__header_buffer = bytearray()

        try:
            header = header_bytes.decode(
                self.__caster_encoding_format,
                errors="replace",
            )
        except TypeError:
            # Compatibility with unusual codecs on older Python versions.
            header = header_bytes.decode(self.__caster_encoding_format)

        self.__logger.debug(
            "NTRIP caster response header: {!r}".format(header)
        )

        if "SOURCETABLE" in header:
            self.__logger.error(
                "NTRIP mountpoint {} does not exist; caster returned "
                "a source table.".format(self.__mountpoint)
            )
            self.__close_socket()
            raise NtripError("Mount point does not exist")

        if "401 Unauthorized" in header:
            self.__logger.error(
                "NTRIP authentication rejected for mountpoint {}.".format(
                    self.__mountpoint
                )
            )
            self.__close_socket()
            raise NtripError("Unauthorized request")

        if "404 Not Found" in header:
            self.__logger.error(
                "NTRIP mountpoint {} was not found.".format(
                    self.__mountpoint
                )
            )
            self.__close_socket()
            raise NtripError("Mount point does not exist")

        valid_response = (
            "ICY 200 OK" in header
            or "HTTP/1.0 200 OK" in header
            or "HTTP/1.1 200 OK" in header
        )

        if not valid_response:
            self.__logger.warning(
                "Unexpected NTRIP response header: {!r}".format(header)
            )
            self.__close_socket()
            return

        self.__found_header = True
        self.__last_data_received_ts = time.monotonic()
        self.__receive_buffer.extend(trailing_data)

        self.__logger.info(
            "Connected to NTRIP mountpoint {}.".format(
                self.__mountpoint
            )
        )

        if self.__send_location_to_ntrip:
            self.__queue_gga("Initial")

    def __queue_periodic_gga_if_needed(self, current_ts):
        if not self.__found_header or not self.__send_location_to_ntrip:
            return

        if (
            current_ts - self.__last_gga_send_ts
            < self.GGA_SEND_INTERVAL_SECONDS
        ):
            return

        self.__queue_gga("Periodic")

    def __queue_gga(self, label):
        gga_bytes = self.getGGABytes()
        if not gga_bytes:
            return

        self.__queue_data_to_send(gga_bytes)
        self.__last_gga_send_ts = time.monotonic()

        self.__logger.debug(
            "{} GPGGA sentence queued for NTRIP caster.".format(label)
        )

    def __extract_one_rtcm_packet(self):
        while self.__receive_buffer:
            current_byte = bytes(self.__receive_buffer[:1])
            del self.__receive_buffer[:1]

            if not self.__rtcm3_decoder.read(current_byte):
                continue

            self.__last_packet_id = self.__rtcm3_decoder.get_packet_ID()
            packet = self.__rtcm3_decoder.get_packet()

            self.__logger.debug(
                "RTCM packet received: id={}, size={} bytes".format(
                    self.__last_packet_id,
                    len(packet),
                )
            )

            # Keep legacy behavior: the caller may decide how filters are used.
            return packet

        return None

    def __reset_protocol_state(self):
        self.__sent_header = False
        self.__found_header = False
        self.__header_buffer = bytearray()
        self.__receive_buffer = bytearray()
        self.__send_buffer = bytearray()
        self.__rtcm3_decoder.reset()

    def __close_active_socket_only(self):
        if self.__caster_socket is None:
            return

        try:
            self.__caster_socket.close()
        except OSError as error:
            self.__logger.debug(
                "Error while closing NTRIP socket: {}".format(error)
            )
        finally:
            self.__caster_socket = None

    def __close_socket(self, schedule_reconnect=True):
        self.__close_active_socket_only()

        self.__state = self.STATE_DISCONNECTED
        self.__resolver_generation += 1
        self.__connect_started_ts = None
        self.__resolver_started_ts = None
        self.__resolver_thread = None
        self.__resolved_addresses = []
        self.__resolved_address_index = 0

        with self.__resolver_lock:
            self.__resolver_result = None
            self.__resolver_error = None

        self.__reset_protocol_state()

        if schedule_reconnect:
            self.__next_connect_attempt_ts = (
                time.monotonic() + self.RECONNECT_DELAY_SECONDS
            )
        else:
            self.__next_connect_attempt_ts = float("inf")

        self.__logger.debug(
            "NTRIP socket closed and connection state reset."
        )
