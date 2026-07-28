import argparse
import json
import threading
import time
from typing import Optional

import zmq

from gnss.constants import IPC_ENDPOINT, TOPIC
from navigation import GNSSPoint
from logger import LoggerFactory


class GNSSSubscriber:
    def __init__(
        self,
        endpoint=IPC_ENDPOINT,
        topic=TOPIC,
        poll_timeout_ms=200,
    ):
        self.__endpoint = endpoint
        self.__topic = topic
        self.__poll_timeout_ms = poll_timeout_ms

        self.__logger = LoggerFactory.create(self.__class__.__name__)

        self.__latest_point: Optional[GNSSPoint] = None
        self.__point_lock = threading.RLock()

        self.__running = threading.Event()
        self.__thread: Optional[threading.Thread] = None

    @property
    def is_running(self):
        return self.__running.is_set()

    def get_last_position_v2(self) -> Optional[GNSSPoint]:
        with self.__point_lock:
            return self.__latest_point

    def start(self):
        if self.__running.is_set():
            return

        self.__running.set()

        self.__thread = threading.Thread(
            target=self.__subscriber_loop,
            name="GNSS-ZMQ-Subscriber",
        )

        self.__thread.daemon = True
        self.__thread.start()

    def stop(self, timeout=2.0):
        self.__running.clear()

        thread = self.__thread

        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout)

        self.__thread = None

    def __subscriber_loop(self):
        context = zmq.Context()
        subscriber = context.socket(zmq.SUB)

        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.setsockopt_string(zmq.SUBSCRIBE, self.__topic)

        poller = zmq.Poller()

        try:
            subscriber.connect(self.__endpoint)
            poller.register(subscriber, zmq.POLLIN)

            self.__logger.info(
                "GNSS subscriber connected to {}".format(
                    self.__endpoint
                )
            )

            self.__logger.info(
                "Subscribed to topic: {}".format(
                    self.__topic
                )
            )

            while self.__running.is_set():
                events = dict(
                    poller.poll(self.__poll_timeout_ms)
                )

                if subscriber not in events:
                    continue

                if not events[subscriber] & zmq.POLLIN:
                    continue

                topic_bytes, payload_bytes = subscriber.recv_multipart()

                received_topic = topic_bytes.decode("utf-8")

                if received_topic != self.__topic:
                    self.__logger.warning(
                        "Received message with unexpected topic: {}".format(
                            received_topic
                        )
                    )
                    continue

                point = self.__deserialize_point(payload_bytes)

                with self.__point_lock:
                    self.__latest_point = point

        except zmq.ContextTerminated:
            pass

        except zmq.ZMQError as error:
            if self.__running.is_set():
                self.__logger.error(
                    "ZeroMQ error in GNSS subscriber: {}".format(
                        error
                    )
                )

        except Exception as error:
            if self.__running.is_set():
                self.__logger.exception(
                    "Unexpected error in GNSS subscriber: {}".format(
                        error
                    )
                )

        finally:
            self.__running.clear()

            try:
                poller.unregister(subscriber)
            except (KeyError, zmq.ZMQError):
                pass

            subscriber.close()
            context.term()

            self.__logger.info("GNSS subscriber stopped")

    @staticmethod
    def __deserialize_point(payload: bytes) -> GNSSPoint:
        try:
            data = json.loads(payload.decode("utf-8"))

        except (UnicodeDecodeError, ValueError) as error:
            raise ValueError(
                "Invalid GNSSPoint JSON payload: {}".format(
                    error
                )
            )

        return GNSSPoint.from_dict(data)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

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
    logger = LoggerFactory.create("GNSSSubscriber-Runtime")
    
    subscriber = GNSSSubscriber()
    logger.info("Starting GNSS subscriber runtime")
    logger.info("Endpoint: {}".format(IPC_ENDPOINT))
    logger.info("Topic: {}".format(TOPIC))
    subscriber.start()

    try:
        sum_delay = 0.0
        delay_count = 0

        last_point = None
        last_average_print_ts = time.monotonic()

        while True:
            point = subscriber.get_last_position_v2()

            if point != last_point:
                last_point = point

                current_ts = time.time()
                delay = current_ts - point.receiving_ts

                sum_delay += delay
                delay_count += 1

                logger.debug(
                    "New GNSS position at current_ts={} : "
                    "latitude={:.20f}, longitude={:.20f}, quality={}, "
                    "creation_ts={}, receiving_ts={}, delay={:.2f} ms".format(
                        current_ts,
                        point.latitude,
                        point.longitude,
                        point.quality,
                        point.creation_ts,
                        point.receiving_ts,
                        delay * 1000.0,
                    )
                )

            if time.monotonic() - last_average_print_ts >= 5.0:
                if delay_count > 0:
                    average_delay = sum_delay / delay_count

                    logger.info(
                        "Average delay over 5 seconds: "
                        "{:.6f} s ({:.2f} ms) over {} unique samples".format(
                            average_delay,
                            average_delay * 1000.0,
                            delay_count,
                        )
                    )
                else:
                    logger.warning("No new GNSS sample received over 5 seconds")

                sum_delay = 0.0
                delay_count = 0
                last_average_print_ts = time.monotonic()

            time.sleep(0.01)

    except KeyboardInterrupt:
        logger.info("GNSS subscriber shutdown requested")

    finally:
        subscriber.stop()
        logger.info("GNSS subscriber runtime stopped")


if __name__ == "__main__":
    main()