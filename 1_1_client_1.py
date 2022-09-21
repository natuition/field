import socket
import threading
import time
import adapters
from config import config

def main():
    gps = None
    try:
        gps = adapters.GPSUbloxAdapterProxyClient(config.NTRIP_PROXY_SERVER_HOST, config.NTRIP_PROXY_SERVER_PORT)
        prev_read_t = time.time()
        while True:
            point = gps.get_fresh_position()
            read_t = time.time()
            print(point, round(read_t - prev_read_t, 3))
            prev_read_t = read_t
    except KeyboardInterrupt:
        pass
    finally:
        if gps:
            gps.close()


if __name__ == '__main__':
    main()
