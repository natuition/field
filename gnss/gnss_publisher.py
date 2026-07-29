import argparse
import json
import time

import serial
import zmq

import utility
from config import config
from gnss.constants import IPC_ENDPOINT, TOPIC
from logger import LoggerFactory
from navigation import GNSSPoint, GPSComputing
from gnss.ntrip_client import NtripClient, NtripError


class GNSSPublisher:
    def __init__(self, endpoint=IPC_ENDPOINT, topic=TOPIC):
        self.__endpoint = endpoint
        self.__topic = topic

        self.__zmq_context = zmq.Context()
        self.__publisher = self.__zmq_context.socket(zmq.PUB)
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
        
        self.__position_count = 0
        self.__position_quality_counts = {}
        self.__last_position_stats_ts = time.monotonic()
        self.__position_stats_interval = 5.0

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

        if self.__zmq_context is not None:
            self.__zmq_context.term()

        self.__logger.info("GNSS publisher stopped")

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    @staticmethod
    def __create_ntrip_client(latitude, longitude):
        return NtripClient(
            user=config.NTRIP_USER,
            password=config.NTRIP_PASSWORD,
            caster=config.NTRIP_CASTER,
            port=config.NTRIP_PORT,
            mountpoint=config.NTRIP_MOUNTPOINT,
            lat=latitude,
            long=longitude,
            caster_encoding_format=config.CASTER_RESPONSE_DECODE,
            send_gga_to_caster=config.SEND_LOCATION_TO_NTRIP,
            filter_rtcm_by_id=config.RTK_ID_SEND,
            ntrip_sleep_time=config.NTRIP_SLEEP_TIME,
        )
    
    def __log_position_statistics_if_needed(self):
        current_ts = time.monotonic()
        elapsed = current_ts - self.__last_position_stats_ts

        if elapsed < self.__position_stats_interval:
            return

        qualities = ", ".join(
            "{}={}".format(quality, count)
            for quality, count in sorted(
                self.__position_quality_counts.items()
            )
        )

        if not qualities:
            qualities = "none"

        self.__logger.info(
            "{} GNSS position(s) sent during the last {:.1f} seconds. "
            "Qualities: {}".format(
                self.__position_count,
                elapsed,
                qualities,
            )
        )

        self.__position_count = 0
        self.__position_quality_counts = {}
        self.__last_position_stats_ts = current_ts
        
    def __update_ntrip_position(self, point):
        if self.__ntrip_client is None:
            self.__ntrip_client = GNSSPublisher.__create_ntrip_client(
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

        self.__ntrip_client.__current_lat = point.latitude
        self.__ntrip_client.__current_long = point.longitude
        
    def __read_and_publish_position(self):
        raw_line = self.__gps_serial.readline()

        if not raw_line:
            return

        receiving_ts = time.time()

        try:
            line = raw_line.decode("ascii", errors="ignore").strip()
            point = GPSComputing.parse_gga(line, receiving_ts)
        except (ValueError, IndexError) as error:
            self.__logger.debug(
                "Invalid NMEA sentence: {}".format(error)
            )
            return

        if point is None:
            return

        self.publish(point)
        self.__position_count += 1

        self.__position_quality_counts[point.quality] = (
            self.__position_quality_counts.get(point.quality, 0) + 1
        )

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

        rtcm_id = self.__ntrip_client.__last_packet_id

        if rtcm_id not in self.__sent_rtcm_ids:
            self.__sent_rtcm_ids.append(rtcm_id)

            self.__logger.debug(
                "RTCM IDs received: {}".format(
                    self.__sent_rtcm_ids
                )
            )

        written_size = self.__gps_serial.write(correction)
        self.__gps_serial.flush()

        self.__logger.debug(
            "RTCM correction {} sent to GNSS receiver"
            "({}/{} bytes)".format(
                rtcm_id,
                written_size,
                len(correction),
            )
        )

    def run(self):
        try:
            while True:
                self.__read_and_publish_position()
                self.__read_and_send_rtcm_correction()
                self.__log_position_statistics_if_needed()

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
