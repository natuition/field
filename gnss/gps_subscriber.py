import json
import threading
import time
from typing import Optional

import zmq

from gnss.constants import IPC_ENDPOINT, TOPIC
from navigation import GPSPoint
from logger import LoggerFactory


class GPSSubscriber:
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

        self.__latest_point: Optional[GPSPoint] = None
        self.__point_lock = threading.RLock()

        self.__running = threading.Event()
        self.__thread: Optional[threading.Thread] = None

    @property
    def is_running(self):
        return self.__running.is_set()

    def get_last_position_v2(self) -> Optional[GPSPoint]:
        with self.__point_lock:
            return self.__latest_point

    def start(self):
        if self.__running.is_set():
            return

        self.__running.set()

        self.__thread = threading.Thread(
            target=self.__subscriber_loop,
            name="GPS-ZMQ-Subscriber",
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
                "GPS subscriber connected to {}".format(
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
                    "ZeroMQ error in GPS subscriber: {}".format(
                        error
                    )
                )

        except Exception as error:
            if self.__running.is_set():
                self.__logger.exception(
                    "Unexpected error in GPS subscriber: {}".format(
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

            self.__logger.info("GPS subscriber stopped")

    @staticmethod
    def __deserialize_point(payload: bytes) -> GPSPoint:
        try:
            data = json.loads(payload.decode("utf-8"))

        except (UnicodeDecodeError, ValueError) as error:
            raise ValueError(
                "Invalid GPSPoint JSON payload: {}".format(
                    error
                )
            )

        return GPSPoint.from_dict(data)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()


def main():
    LoggerFactory.set_level("DEBUG")
    logger = LoggerFactory.create("GPSSubscriber-Runtime")
    
    subscriber = GPSSubscriber()
    logger.info("Starting GPS subscriber runtime")
    logger.info("Endpoint: {}".format(IPC_ENDPOINT))
    logger.info("Topic: {}".format(TOPIC))
    subscriber.start()

    try:
        sum_delay = 0.0
        delay_count = 0

        last_average_print_ts = time.monotonic()

        while True:
            point = subscriber.get_last_position_v2()
            current_ts = time.time()

            if point is None:
                logger.debug("No GPS position received yet")
            else:
                delay = current_ts - point.receiving_ts

                sum_delay += delay
                delay_count += 1

                logger.debug(
                    "Latest GPS position at current_ts={} : "
                    "latitude={:.8f}, longitude={:.8f}, quality={}, "
                    "creation_ts={}, receiving_ts={}".format(
                        current_ts,
                        point.latitude,
                        point.longitude,
                        point.quality,
                        point.creation_ts,
                        point.receiving_ts,
                    )
                )

            if time.monotonic() - last_average_print_ts >= 5.0:
                if delay_count > 0:
                    average_delay = sum_delay / delay_count

                    print(
                        "Average delay over 5 seconds: "
                        "{:.6f} s ({:.2f} ms) over {} samples".format(
                            average_delay,
                            average_delay * 1000.0,
                            delay_count,
                        )
                    )
                else:
                    print("Average delay over 5 seconds: no GPS samples")

                sum_delay = 0.0
                delay_count = 0
                last_average_print_ts = time.monotonic()

            time.sleep(0.01)

    except KeyboardInterrupt:
        logger.info("GPS subscriber shutdown requested")

    finally:
        subscriber.stop()
        logger.info("GPS subscriber runtime stopped")


if __name__ == "__main__":
    main()