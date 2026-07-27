import argparse
import json
import random
import time

import zmq

from gnss.constants import IPC_ENDPOINT, TOPIC
from navigation import GNSSPoint

from logger import LoggerFactory


class GNSSPublisher:
    def __init__(self, endpoint=IPC_ENDPOINT, topic=TOPIC):
        self.__endpoint = endpoint
        self.__topic = topic

        self.__context = zmq.Context()
        self.__publisher = self.__context.socket(zmq.PUB)

        self.__publisher.setsockopt(zmq.LINGER, 0)

        self.__publisher.bind(self.__endpoint)

    def publish(self, point: GNSSPoint)-> None:
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

    def close(self):
        self.__publisher.close()
        self.__context.term()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        
def create_random_gps_point():
    # Random position around La Rochelle.
    latitude = random.uniform(46.1400, 46.1800)
    longitude = random.uniform(-1.1800, -1.1200)

    timestamp = time.time()

    return GNSSPoint(
        latitude=latitude,
        longitude=longitude,
        quality=random.choice(
            [
                "1",
                "5",
                "4",
            ]
        ),
        creation_ts=timestamp-(random.uniform(0.001, 0.01)),
        receiving_ts=timestamp,
    )


def parse_args():
    parser = argparse.ArgumentParser(description="GNSS subscriber runtime")
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

    try:
        while True:
            point = create_random_gps_point()

            publisher.publish(point)

            logger.debug(
                "Position sent: "
                "latitude={:.8f}, longitude={:.8f}, quality={}".format(
                    point.latitude,
                    point.longitude,
                    point.quality,
                )
            )

            time.sleep(0.25)

    except KeyboardInterrupt:
        logger.info("GNSS publisher shutdown requested")

    finally:
        publisher.close()
        logger.info("GNSS publisher stopped")


if __name__ == "__main__":
    main()