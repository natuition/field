"""Script to check hardware and connections availability"""

# TODO add config loading stability later
from config import config
import adapters
import utility
import time
import os
import datetime
import pytz
import serial
import serial.tools.list_ports
import traceback

# SETTINGS
SERVICE_RESTART_COMMAND = ""  # "sudo systemctl restart THIS_SERVICE_NAME.service"
LOGS_DIR = "pre_check_logs/"
# list of any amount of hosts, first successful connection will confirm connection immediately and cause to stop pinging
HOSTS_TO_PING = ["google.com", "youtube.com", "microsoft.com", "github.com"]

ADDRESSES_TRIES_MAX = 5
ADDRESSES_RETRIES_DELAY = 1

SMOOTHIE_CONN_TIMEOUT = 5
SMOOTHIE_CONN_TRIES_MAX = 2
SMOOTHIE_CONN_RETRIES_DELAY = 1

VESC_CONN_TIMEOUT = 5
VESC_CONN_TRIES_MAX = 2
VESC_CONN_RETRIES_DELAY = 1

VESC_ADAPTER_TRIES_MAX = 2
VESC_ADAPTER_RETRIES_DELAY = 1

UBLOX_NTRIP_CONN_TIMEOUT = 5
UBLOX_NTRIP_CONN_TRIES_MAX = 2
UBLOX_NTRIP_CONN_RETRIES_DELAY = 1

UBLOX_GPS_CONN_TIMEOUT = 5
UBLOX_GPS_CONN_TRIES_MAX = 2
UBLOX_GPS_CONN_RETRIES_DELAY = 1

GPS_ADAPTER_TRIES_MAX = 2
GPS_ADAPTER_RETRIES_DELAY = 1


def restart_this_service():
    # os.system(SERVICE_RESTART_COMMAND)
    pass


def restart_this_computer():
    raise NotImplementedError("this function is not implemented yet")


def is_internet_conn_available():
    """Return True if any of hosts is reachable, return False if none of them reachable"""

    for host in HOSTS_TO_PING:
        if os.system(f"ping -c 1 {host}") == 0:
            return True
    return False


def is_camera_available():
    """Return True if camera is detected, return False otherwise"""

    return os.system("ls /dev/video0") == 0


def get_cur_time_formatted():
    return datetime.datetime.now(pytz.timezone('Europe/Berlin')).strftime("%d-%m-%Y_%H-%M-%S")


def get_hardware_addresses():
    equipment_ports = dict()
    for port, desc, other in sorted(serial.tools.list_ports.comports()):
        if "Smoothie" in desc:
            equipment_ports["smoothie"] = port
        if "ChibiOS/RT" in desc:
            equipment_ports["vesc"] = port
        if "u-blox" in desc:
            equipment_ports["ublox"] = port
    return equipment_ports


def main():
    # make log directory and initialize logger
    global LOGS_DIR
    if not LOGS_DIR.endswith("/") and not LOGS_DIR.endswith("\\"):
        LOGS_DIR += "/"
    utility.create_directories(LOGS_DIR)
    logger = utility.Logger(LOGS_DIR + f"{get_cur_time_formatted()}.log")

    try:
        # check internet connection
        if not is_internet_conn_available():
            # msg = f"Internet connection: FAIL\nRestarting jetson due to failed internet connection test."
            msg = f"Internet connection: FAIL"
            logger.write_and_flush(msg + "\n")
            # restart_this_computer()
        else:
            msg = f"Internet connection: OK"
            logger.write_and_flush(msg + "\n")

        # check for camera availability
        if is_camera_available():
            msg = f"Camera: OK"
            logger.write_and_flush(msg + "\n")
        else:
            msg = f"Camera: FAIL"
            logger.write_and_flush(msg + "\n")

        # look for hardware availability (get hardware addresses)
        smoothie_address = None
        vesc_address = None
        ublox_ntrip_address = None
        addr_reading_count = 0
        while addr_reading_count < ADDRESSES_TRIES_MAX:
            hardware_addresses = get_hardware_addresses()

            msg = f"At {addr_reading_count + 1} attempt got these addresses: {str(hardware_addresses)}"
            logger.write_and_flush(msg + "\n")

            if "vesc" in hardware_addresses:
                vesc_address = hardware_addresses["vesc"]

            if config.SMOOTHIE_BACKEND == 1:
                smoothie_address = config.SMOOTHIE_HOST
            elif "smoothie" in hardware_addresses:
                smoothie_address = hardware_addresses["smoothie"]

            # get ublox address (USB is auto-detected, used for Ntrip; Uart is in config, used for ublox)
            if "ublox" in hardware_addresses:
                ublox_ntrip_address = hardware_addresses["ublox"]

            if vesc_address is not None and smoothie_address is not None and ublox_ntrip_address is not None:
                msg = f"Vesc address: OK\nSmoothie address: OK\nUblox Ntrip address: OK"
                logger.write_and_flush(msg + "\n")
                break

            addr_reading_count += 1
            time.sleep(ADDRESSES_RETRIES_DELAY)
        else:
            if vesc_address is None:
                msg = f"Vesc address: FAIL"
                logger.write_and_flush(msg + "\n")
            if smoothie_address is None:
                msg = f"Smoothie address: FAIL"
                logger.write_and_flush(msg + "\n")
            if ublox_ntrip_address is None:
                msg = f"Ublox address: FAIL"
                logger.write_and_flush(msg + "\n")
            msg = f"Restarting pre_check.py service due to hardware addresses (see above) auto detection fail."
            logger.write_and_flush(msg + "\n")
            restart_this_service()

        # check smoothie low level connection
        smoothie_conn_try_count = 0
        while smoothie_conn_try_count < SMOOTHIE_CONN_TRIES_MAX:
            if config.SMOOTHIE_BACKEND == 2:
                try:
                    with serial.Serial(
                            smoothie_address,
                            config.SMOOTHIE_BAUDRATE,
                            timeout=SMOOTHIE_CONN_TIMEOUT) as smoothie_ser_conn:
                        time.sleep(0.5)
                        smoothie_ser_conn.flushInput()
                        smoothie_ser_conn.flushOutput()
                        smoothie_ser_conn.write(b"G91\n")
                        res = smoothie_ser_conn.readline().decode()
                        if res == adapters.SmoothieAdapter.RESPONSE_OK:
                            msg = f"Smoothie connection: OK"
                            logger.write_and_flush(msg + "\n")
                            break
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except:
                    msg = f"At {smoothie_conn_try_count + 1} attempt exception occurred during establishing " \
                          f"connection to smoothie:\n{traceback.format_exc()}"
                    logger.write_and_flush(msg + "\n")
            else:
                msg = f"Smoothie direct connection test currently is not supporting " \
                      f"config.SMOOTHIE_BACKEND={config.SMOOTHIE_BACKEND} mode. Skipping this test."
                logger.write_and_flush(msg + "\n")
                break

            smoothie_conn_try_count += 1
            time.sleep(SMOOTHIE_CONN_RETRIES_DELAY)
        else:
            msg = f"Smoothie connection: FAIL\nRestarting service due to smoothie connection fail."
            logger.write_and_flush(msg + "\n")
            restart_this_service()

        # check vesc low level connection
        vesc_conn_try_count = 0
        while vesc_conn_try_count < VESC_CONN_TRIES_MAX:
            try:
                with serial.Serial(vesc_address, config.VESC_BAUDRATE, timeout=VESC_CONN_TIMEOUT) as vesc_ser_conn:
                    time.sleep(0.5)
                    vesc_ser_conn.flushInput()
                    vesc_ser_conn.flushOutput()
                    msg = f"Vesc connection: OK"
                    logger.write_and_flush(msg + "\n")
                    break
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                msg = f"At {vesc_conn_try_count + 1} attempt exception occurred during establishing connection to " \
                      f"vesc:\n{traceback.format_exc()}"
                logger.write_and_flush(msg + "\n")

            vesc_conn_try_count += 1
            time.sleep(VESC_CONN_RETRIES_DELAY)
        else:
            msg = f"Vesc connection: FAIL\nRestarting service due to vesc connection fail."
            logger.write_and_flush(msg + "\n")
            restart_this_service()

        # check ublox ntrip low level connection
        ublox_ntrip_conn_try_count = 0
        while ublox_ntrip_conn_try_count < UBLOX_NTRIP_CONN_TRIES_MAX:
            try:
                with serial.Serial(
                        ublox_ntrip_address,
                        config.NTRIP_OUTPUT_BAUDRATE,
                        timeout=UBLOX_NTRIP_CONN_TIMEOUT) as ublox_ntrip_ser_conn:
                    time.sleep(0.5)
                    ublox_ntrip_ser_conn.flushInput()
                    ublox_ntrip_ser_conn.flushOutput()
                    msg = f"Ublox ntrip connection: OK"
                    logger.write_and_flush(msg + "\n")
                    break
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                msg = f"At {ublox_ntrip_conn_try_count + 1} attempt exception occurred during establishing " \
                      f"connection to ublox ntrip:\n{traceback.format_exc()}"
                logger.write_and_flush(msg + "\n")

            ublox_ntrip_conn_try_count += 1
            time.sleep(UBLOX_NTRIP_CONN_RETRIES_DELAY)
        else:
            msg = f"Ublox ntrip connection: FAIL\nRestarting service due to ublox ntrip connection fail."
            logger.write_and_flush(msg + "\n")
            restart_this_service()

        # check ublox gps low level connection
        ublox_gps_conn_try_count = 0
        while ublox_gps_conn_try_count < UBLOX_GPS_CONN_TRIES_MAX:
            try:
                with serial.Serial(
                        config.GPS_PORT,
                        config.GPS_BAUDRATE,
                        timeout=UBLOX_GPS_CONN_TIMEOUT) as ublox_gps_ser_conn:
                    time.sleep(0.5)
                    ublox_gps_ser_conn.flushInput()
                    ublox_gps_ser_conn.flushOutput()
                    msg = f"Ublox gps connection: OK"
                    logger.write_and_flush(msg + "\n")
                    break
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                msg = f"At {ublox_gps_conn_try_count + 1} attempt exception occurred during establishing " \
                      f"connection to gps ntrip:\n{traceback.format_exc()}"
                logger.write_and_flush(msg + "\n")

            ublox_gps_conn_try_count += 1
            time.sleep(UBLOX_GPS_CONN_RETRIES_DELAY)
        else:
            msg = f"Ublox gps connection: FAIL\nRestarting service due to ublox gps connection fail."
            logger.write_and_flush(msg + "\n")
            restart_this_service()

        # check GPS adapter
        gps_adapter_try_count = 0
        while gps_adapter_try_count < GPS_ADAPTER_TRIES_MAX:
            try:
                with adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, 10) as gps:
                    time.sleep(2)
                    position = gps.get_last_position_v2_non_blocking()
                    position_str = str(position) if position is None else str(position.as_old_list)

                    if position is not None:
                        msg = f"GPS adapter: OK"
                        logger.write_and_flush(msg + "\n")
                        break
                    else:
                        msg = f"At {gps_adapter_try_count + 1} attempt got position: {position_str}"
                        logger.write_and_flush(msg + "\n")
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                msg = f"At {gps_adapter_try_count + 1} attempt exception occurred during creation or using " \
                      f"gps adapter:\n{traceback.format_exc()}"
                logger.write_and_flush(msg + "\n")

            gps_adapter_try_count += 1
            time.sleep(GPS_ADAPTER_RETRIES_DELAY)
        else:
            msg = f"GPS adapter: FAIL\nRestarting service due to gps adapter fail."
            logger.write_and_flush(msg + "\n")
            restart_this_service()

        # check vesc adapter
        report_field_names = [
            'temp_fet_filtered',
            'temp_motor_filtered',
            'avg_motor_current',
            'avg_input_current',
            'rpm',
            'input_voltage']
        vesc_adapter_try_count = 0
        while vesc_adapter_try_count < VESC_ADAPTER_TRIES_MAX:
            try:
                with adapters.VescAdapterV4(
                        vesc_address,
                        config.VESC_BAUDRATE,
                        config.VESC_ALIVE_FREQ,
                        config.VESC_CHECK_FREQ,
                        config.VESC_STOPPER_CHECK_FREQ) as vesc_engine:
                    sensors_data = vesc_engine.get_sensors_data(report_field_names, vesc_engine.PROPULSION_KEY)
                    time.sleep(0.5)
                    if sensors_data is not None:
                        msg = f"Vesc adapter: OK (got sensors data: {str(sensors_data)})"
                        logger.write_and_flush(msg + "\n")
                        break
                    else:
                        msg = f"At {vesc_adapter_try_count + 1} attempt got vesc sensors data: {str(sensors_data)}"
                        logger.write_and_flush(msg + "\n")
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                msg = f"At {vesc_adapter_try_count + 1} attempt exception occurred during creation or using " \
                      f"vesc adapter:\n{traceback.format_exc()}"
                logger.write_and_flush(msg + "\n")

            vesc_adapter_try_count += 1
            time.sleep(VESC_ADAPTER_RETRIES_DELAY)
        else:
            msg = f"Vesc adapter: FAIL\nRestarting service due to vesc adapter fail."
            logger.write_and_flush(msg + "\n")
            restart_this_service()

        msg = f"All tests are OK"
        logger.write_and_flush(msg + "\n")
    except KeyboardInterrupt:
        msg = f"Stopped by KBI"
        logger.write_and_flush(msg + "\n")
    finally:
        logger.close()


if __name__ == '__main__':
    main()
