import argparse
import datetime
import json
import time

import serial
import zmq

import utility
from config import config
from gnss.constants import IPC_ENDPOINT, TOPIC
from logger import LoggerFactory
from navigation import GNSSPoint
from gnss.ntrip_client import NtripClient, NtripError


class GNSSPublisher:
    def __init__(self, endpoint=IPC_ENDPOINT, topic=TOPIC):
        self.__endpoint = endpoint
        self.__topic = topic

        self.__context = zmq.Context()
        self.__publisher = self.__context.socket(zmq.PUB)
        self.__publisher.setsockopt(zmq.LINGER, 0)
        self.__publisher.bind(self.__endpoint)

        self.__logger = LoggerFactory.create(self.__class__.__name__)

        serial_port = utility.get_ublox_address()

        if not serial_port:
            raise RuntimeError(
                "Unable to find the u-blox serial port"
            )

        self.__gps_serial = serial.Serial(
            serial_port,
            config.NTRIP_OUTPUT_BAUDRATE,
            timeout=0.05,
            write_timeout=1.0,
        )

        self.__ntrip_client = None
        self.__sent_rtcm_ids = []
        self.__closed = False

    def publish(self, point: GNSSPoint) -> None:
        if not isinstance(point, GNSSPoint):
            raise TypeError(
                "point must be a GNSSPoint, got {} instead".format(
                    type(point).__name__
                )
            )

        payload = json.dumps(
            point.as_dict,
            separators=(",", ":"),
        ).encode("utf-8")

        self.__publisher.send_multipart([
            self.__topic.encode("utf-8"),
            payload,
        ])

    def __enter__(self):
        return self
    
    def close(self):
        if self.__closed:
            return

        self.__closed = True

        if self.__ntrip_client is not None:
            ntrip_socket = getattr(
                self.__ntrip_client,
                "socket",
                None,
            )

            if ntrip_socket is not None:
                try:
                    ntrip_socket.close()
                except OSError as error:
                    self.__logger.warning(
                        "Error while closing NTRIP socket: {}".format(
                            error
                        )
                    )

        if (
            self.__gps_serial is not None
            and self.__gps_serial.is_open
        ):
            try:
                self.__gps_serial.close()
            except serial.SerialException as error:
                self.__logger.warning(
                    "Error while closing serial port: {}".format(
                        error
                    )
                )

        if self.__publisher is not None:
            self.__publisher.close()

        if self.__context is not None:
            self.__context.term()

        self.__logger.info("GNSS publisher stopped")

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    @staticmethod
    def nmea_coordinate_to_decimal(value, hemisphere):
        """Convert a NMEA ddmm.mmmm/dddmm.mmmm coordinate to decimal degrees."""
        if not value:
            raise ValueError("Empty NMEA coordinate")

        raw = float(value)
        degrees = int(raw // 100)
        minutes = raw - degrees * 100
        coordinate = degrees + minutes / 60.0

        if hemisphere in ("S", "W"):
            coordinate = -coordinate

        return coordinate

    @staticmethod
    def nmea_time_to_timestamp(value, receiving_ts):
        """Build today's UTC timestamp from the GGA hhmmss.sss field."""
        if not value:
            return receiving_ts

        hours = int(value[0:2])
        minutes = int(value[2:4])
        seconds_float = float(value[4:])
        seconds = int(seconds_float)
        microseconds = int(round((seconds_float - seconds) * 1_000_000))

        if microseconds == 1_000_000:
            seconds += 1
            microseconds = 0

        receiving_date = datetime.datetime.fromtimestamp(
            receiving_ts,
            tz=datetime.timezone.utc,
        ).date()

        creation_datetime = datetime.datetime(
            receiving_date.year,
            receiving_date.month,
            receiving_date.day,
            hours,
            minutes,
            seconds,
            microseconds,
            tzinfo=datetime.timezone.utc,
        )

        creation_ts = creation_datetime.timestamp()

        # Around UTC midnight, the GGA time can belong to the adjacent day.
        if creation_ts - receiving_ts > 12 * 3600:
            creation_ts -= 24 * 3600
        elif receiving_ts - creation_ts > 12 * 3600:
            creation_ts += 24 * 3600

        return creation_ts

    @staticmethod
    def parse_gga(line, receiving_ts):
        """Parse a GGA sentence and return a GNSSPoint, or None for another sentence."""
        if not line.startswith(("$GPGGA,", "$GNGGA,")):
            return None

        sentence = line.split("*", 1)[0]
        fields = sentence.split(",")

        if len(fields) < 10:
            raise ValueError("Incomplete GGA sentence: {}".format(line))

        latitude = GNSSPublisher.nmea_coordinate_to_decimal(fields[2], fields[3])
        longitude = GNSSPublisher.nmea_coordinate_to_decimal(fields[4], fields[5])
        quality = int(fields[6])
        if quality == 0:
            return None
        creation_ts = GNSSPublisher.nmea_time_to_timestamp(fields[1], receiving_ts)

        return GNSSPoint(
            latitude=latitude,
            longitude=longitude,
            quality=quality,
            creation_ts=creation_ts,
            receiving_ts=receiving_ts,
        )

    @staticmethod
    def create_ntrip_client(latitude, longitude):
        return NtripClient(
            user=config.NTRIP_USER,
            password=config.NTRIP_PASSWORD,
            caster=config.NTRIP_CASTER,
            port=config.NTRIP_PORT,
            mountpoint=config.NTRIP_MOUNTPOINT,
            lat=latitude,
            long=longitude,
            caster_response_decode=config.CASTER_RESPONSE_DECODE,
            send_location_to_ntrip=config.SEND_LOCATION_TO_NTRIP,
            rtk_id_send=config.RTK_ID_SEND,
            ntrip_sleep_time=config.NTRIP_SLEEP_TIME,
        )

    @staticmethod
    def rtcm_id_is_allowed(rtcm_id):
        if not config.RTK_ID_SEND:
            return True

        return any(rtcm_id in filter_ids for filter_ids in config.RTK_ID_SEND)
    
    def __must_pause_ntrip(self):
        if not config.RTK_ID_SEND:
            return False

        return all(
            bool(set(self.__sent_rtcm_ids) & set(filter_ids))
            for filter_ids in config.RTK_ID_SEND
        )
        
    def __update_ntrip_position(self, point):
        if self.__ntrip_client is None:
            self.__ntrip_client = GNSSPublisher.create_ntrip_client(
                point.latitude,
                point.longitude,
            )

            self.__logger.info(
                "NTRIP client initialized at latitude={:.8f}, "
                "longitude={:.8f}".format(
                    point.latitude,
                    point.longitude,
                )
            )

            return

        self.__ntrip_client.lat = point.latitude
        self.__ntrip_client.long = point.longitude
        
    def __read_and_publish_position(self):
        raw_line = self.__gps_serial.readline()

        if not raw_line:
            return

        receiving_ts = time.time()

        try:
            line = raw_line.decode("ascii", errors="ignore").strip()
            point = GNSSPublisher.parse_gga(line, receiving_ts)
        except (ValueError, IndexError) as error:
            self.__logger.debug(
                "Invalid NMEA sentence: {}".format(error)
            )
            return

        if point is None:
            return

        self.publish(point)

        self.__logger.debug(
            "Position sent: latitude={:.8f}, longitude={:.8f}, "
            "quality={}, creation_ts={}, receiving_ts={}".format(
                point.latitude,
                point.longitude,
                point.quality,
                point.creation_ts,
                point.receiving_ts,
            )
        )

        self.__update_ntrip_position(point)
        
    def __read_and_send_rtcm_correction(self):
        if self.__ntrip_client is None:
            return

        correction = self.__ntrip_client.read()

        if correction is None:
            return

        rtcm_id = self.__ntrip_client.last_id

        if not self.rtcm_id_is_allowed(rtcm_id):
            return

        if rtcm_id not in self.__sent_rtcm_ids:
            self.__sent_rtcm_ids.append(rtcm_id)

            self.__logger.info(
                "RTCM IDs received: {}".format(
                    self.__sent_rtcm_ids
                )
            )

        written_size = self.__gps_serial.write(correction)
        self.__gps_serial.flush()

        self.__logger.debug(
            "RTCM correction {} sent to F9P "
            "({}/{} bytes)".format(
                rtcm_id,
                written_size,
                len(correction),
            )
        )

        if self.__must_pause_ntrip():
            self.__sent_rtcm_ids = []
            time.sleep(config.NTRIP_SLEEP_TIME)

    def run(self):
        try:
            while True:
                self.__read_and_publish_position()
                self.__read_and_send_rtcm_correction()

        except KeyboardInterrupt:
            self.__logger.info(
                "GNSS publisher shutdown requested"
            )

        except NtripError as error:
            self.__logger.error(
                "NTRIP error: {}".format(error)
            )

        except serial.SerialException as error:
            self.__logger.error(
                "GNSS serial error: {}".format(error)
            )

        except OSError as error:
            self.__logger.error(
                "GNSS/NTRIP I/O error: {}".format(error)
            )

        finally:
            self.close()


def parse_args():
    parser = argparse.ArgumentParser(description="GNSS publisher runtime")
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Logger level (default: INFO)",
    )
    return parser.parse_args()



def main():
    args = parse_args()
    LoggerFactory.set_level(args.log_level)
    logger = LoggerFactory.create("GNSSPublisher-Runtime")

    publisher = GNSSPublisher()

    logger.info("GNSS publisher started")
    logger.info("Endpoint: {}".format(IPC_ENDPOINT))
    logger.info("Topic: {}".format(TOPIC))

    publisher.run()


if __name__ == "__main__":
    main()
