import application
from config import config
import threading
import telnetlib
from urllib.parse import quote, unquote
import subprocess
import grp
import fileinput
import os
import pwd
import utility
import json
import adapters
import time
import sys
sys.path.append('../')
from navigation import NavigationV3
from navigation import GPSComputing


def voltage_thread_tf(voltage_thread_alive, vesc_engine: adapters.VescAdapterV4, socketio, input_voltage, logger: utility.Logger, recreate_vesc_callback):
    """
    Thread function to monitor VESC input voltage and handle bumping events.
    This function continuously checks the VESC input voltage and emits updates to the socketio server.
    If the voltage drops below a certain threshold, it emits "Bumper" to the socketio server.
    If the voltage returns to normal, it emits "Reseting" to the socketio server and attempts to reset the VESC using a lifeline.
    When the VESC is reset, it tries to reconnect to the VESC.

    Args:
        voltage_thread_alive (bool): Boolean that indicates if the voltage thread should continue running.
        vesc_engine (adapters.VescAdapterV4): The VESC adapter instance to get sensor data from.
        socketio: The socketio instance to emit updates to the UI.
        input_voltage (dict): Dictionary to store the latest input voltage.
        logger (utility.Logger): Logger instance for logging messages.
        recreate_vesc_callback (function): Callback function to recreate the VESC connection after a bump event.
    """
    vesc_data = None
    isBumped = False
    while voltage_thread_alive():
        if vesc_engine is not None:
            try:
                vesc_data = vesc_engine.get_sensors_data(["input_voltage"], vesc_engine.PROPULSION_KEY)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                break

        if vesc_data is not None:
            if voltage_thread_alive():
                vesc_voltage = vesc_data.get("input_voltage", None)
                if vesc_voltage is not None:
                    if vesc_voltage < 12.0:
                        msg = f"[Voltage thread] -> Bumped, vesc voltage is {vesc_voltage}V."
                        logger.write_and_flush(msg + "\n")
                        isBumped = True
                        sendBumperInfo(socketio, "Bumper")

                    elif isBumped and vesc_voltage >= 12.0:
                        msg = f"[Voltage thread] -> Unbumped, vesc voltage is {vesc_voltage}V, resetting VESC with lifeline."
                        logger.write_and_flush(msg + "\n")
                        isBumped = False
                        sendBumperInfo(socketio, "Reseting")
                        utility.life_line_reset()                 
                    else:
                        sendInputVoltage(socketio, vesc_data["input_voltage"])
                        input_voltage["input_voltage"] = vesc_data["input_voltage"]
        time.sleep(0.3)


def sendBumperInfo(socketio, bumper_info: str):
    socketio.emit('update', bumper_info, namespace='/voltage', broadcast=True)


def sendInputVoltage(socketio, input_voltage):
    try:
        input_voltage = round(float(input_voltage) * 2) / 2
    except ValueError:
        pass
    socketio.emit('update', input_voltage, namespace='/voltage', broadcast=True)
    



def send_last_pos_thread_tf(send_last_pos_thread_alive, socketio, logger: utility.Logger):
    with adapters.GPSUbloxAdapterWithoutThread(config.GPS_PORT, config.GPS_BAUDRATE, 1) as gps:
        while send_last_pos_thread_alive():
            lastPos = gps.get_fresh_position()
            if config.ALLOW_GPS_BAD_QUALITY_NTRIP_RESTART and lastPos[2]!='4':
                NavigationV3.restart_ntrip_service(logger)
            socketio.emit('updatePath', json.dumps([[[lastPos[1], lastPos[0]]], lastPos[2]]), namespace='/map', broadcast=True)
            socketio.emit('updateGPSQuality', lastPos[2], namespace='/gps', broadcast=True)


def initVesc(logger: utility.Logger):
    for i in range(3):
        if i==2:
            msg = "Couldn't get vesc's USB address, stopping attempt to unlock with lifeline."
            logger.write_and_flush(msg + "\n")
            raise Exception(msg)
        smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
        if "vesc" in smoothie_vesc_addr:
            vesc_address = smoothie_vesc_addr["vesc"]
            msg = f"Finding vesc's USB address at '{vesc_address}'."
            logger.write_and_flush(msg + "\n")
            print(msg)
            break
        else:
            msg = "Couldn't get vesc's USB address, attempt to unlock with lifeline"
            logger.write_and_flush(msg + "\n")
            print(msg)
            utility.life_line_reset()
    
    time.sleep(5)

    vesc_engine = adapters.VescAdapterV4(vesc_address,
                                         config.VESC_BAUDRATE,
                                         config.VESC_ALIVE_FREQ,
                                         config.VESC_CHECK_FREQ,
                                         config.VESC_STOPPER_CHECK_FREQ,
                                         logger)
    vesc_engine.set_target_rpm(0, vesc_engine.PROPULSION_KEY)
    vesc_engine.set_time_to_move(
        config.VESC_MOVING_TIME, vesc_engine.PROPULSION_KEY)
    return vesc_engine


def timeout_sm_th(event, logger):
    time.sleep(10)
    if not event.is_set():
        msg = "[Timeout sm] Couldn't get SmoothieAdapter!"
        logger.write_and_flush(msg + "\n")
        raise Exception(msg)


def initSmoothie(logger: utility.Logger):
    smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
    if config.SMOOTHIE_BACKEND == 1:
        smoothie_address = config.SMOOTHIE_HOST
    else:
        if "smoothie" in smoothie_vesc_addr:
            smoothie_address = smoothie_vesc_addr["smoothie"]
        else:
            msg = "Couldn't get smoothie's USB address!"
            logger.write_and_flush(msg + "\n")
            raise Exception(msg)

    event = threading.Event()
    smoothie_creation_thread = threading.Thread(
        target=timeout_sm_th, args=(event, logger))
    smoothie_creation_thread.start()
    smoothie = adapters.SmoothieAdapter(smoothie_address)
    event.set()

    return smoothie


def save_gps_coordinates(points: list, file_name):
    with open(file_name, "w") as file:
        for point in points:
            str_point = str(point[0]) + " " + str(point[1]) + "\n"
            file.write(str_point)
    user = pwd.getpwnam('violette')
    os.chown(file_name, user.pw_uid, user.pw_gid)


def changeConfigValue(path: str, value):
    with fileinput.FileInput("../config/config.py", inplace=True, backup='.bak') as file:
        found_key = False

        for line in file:
            # skip comments
            if line.startswith("#"):
                print(line, end='')
                continue

            # if key name is strictly equal
            elements = line.split("=")
            if len(elements) > 0 and elements[0].strip() == path:
                print(path + " = " + str(value), end='\n')
                found_key = True
            else:
                print(line, end='')

        # if key is absent - add it to the end of the file
        if not found_key:
            print(path + " = " + str(value), end='\n')

    uid = pwd.getpwnam("violette").pw_uid
    gid = grp.getgrnam("violette").gr_gid
    os.chown("../config/config.py", uid, gid)


def startMain():
    mainSP = subprocess.Popen("python3 main.py", stdin=subprocess.PIPE, cwd=os.getcwd().split("/uiWebRobot")[0],
                              shell=True, preexec_fn=os.setsid)
    return mainSP


def startLiveCam():
    camSP = subprocess.Popen("python3 serveurCamLive.py", stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, cwd=os.getcwd().split(
        "/uiWebRobot")[0], shell=True,
        preexec_fn=os.setsid)
    return camSP


def updateFields(field_name):
    field_name_quote = quote(field_name, safe="", encoding='utf-8')
    
    cmd = "ln -sf 'fields/" + field_name_quote + ".txt' ../field.txt"
    os.system(cmd)

    with open("../field.txt") as file:
        points = file.readlines()

    coords = list()
    for coord in points:
        coord = coord.replace("\n", "").split(" ")
        coords.append([float(coord[1]), float(coord[0])])
    coords.append(coords[0])

    other_fields = get_other_field()

    return coords, other_fields, field_name

def load_field_list(dir_path):
        field_list = []
        for file in os.listdir(dir_path):
            if file.endswith(".txt"):
                #if file != "tmp.txt":
                field_list.append(
                    unquote(file.split(".txt")[0], encoding='utf-8'))
        return field_list

def get_other_field():
        link_path = os.path.realpath("../field.txt")
        current_field = (link_path.split("/")[-1]).split(".")[0]
        field_list = load_field_list("../fields")
        if len(field_list) >= 2:
            coords_other = []
            for field_name in field_list:
                if field_name != unquote(current_field, encoding='utf-8') and field_name != "tmp.txt":
                    with open("../fields/" + quote(field_name, safe="", encoding='utf-8') + ".txt", encoding='utf-8') as file:
                        points = file.readlines()

                    coords = list()
                    for coord in points:
                        coord = coord.replace("\n", "").split(" ")
                        coords.append([float(coord[1]), float(coord[0])])
                    coords.append(coords[0])
                    coords_other.append(coords)
            return coords_other
        return list()


def is_valid_field_file(file_path : str, logger: utility.Logger):
    """
    Check if a field file is valid.

    Args:
        file_path (str): The path to the file to check.

    Returns:
        bool: True if the file is valid, False otherwise.
    """
    # Check if the file exists
    if not os.path.exists(file_path):
        msg = f"[Field file validator] -> Field validation, the file does not exist ({file_path})."
        logger.write(msg + "\n")
        print(msg)
        return False
    try:
        # Check if the file contains the right number of points
        coords_list = utility.load_coordinates(file_path)
        if (len(coords_list) not in [4,2]):
            msg = f"[Field file validator] -> Field validation, the file does not have the right number of line ({len(coords_list)})."
            logger.write(msg + "\n")
            print(msg)
            return False
        
        # Check distance between two consecutive points
        if config.CHECK_MINIMUM_SIZE_FIELD:
            nav = GPSComputing()
            for i in range(len(coords_list) - 1):
                if nav.get_distance(coords_list[i], coords_list[i+1]) <  (config.MINIMUM_SIZE_FIELD * 1000):
                    msg = f"[Field file validator] -> Field validation, the file have a field to small, not saving it."
                    logger.write(msg + "\n")
                    print(msg)
                    return False

    except ValueError as e:
        msg = f"[Field file validator] -> Failed to load field {file_path} due to ValueError (file is likely corrupted)."
        logger.write(msg + "\n")
        print(msg)
        return False
    
    return True