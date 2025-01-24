import sys
sys.path.append('../')

from config import config

from navigation import NavigationV3
from navigation import GPSComputing

from uiWebRobot.state_machine import Events

from typing import Tuple, Dict, List
from flask_socketio import SocketIO
import socketio
import threading
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


def voltage_thread_tf(voltage_thread_alive: bool, vesc_engine: adapters.VescAdapterV4, socketio: SocketIO, input_voltage: Dict[str, str]) -> None:
    """
        Function for getting and sending the voltage to ui.
        
        Args:
            - voltage_thread_alive
            - vesc_engine
            - socketio : socket connected with ui
            - input_voltage
    """
    #last_update = 0
    vesc_data = None
    while voltage_thread_alive():
        if voltage_thread_alive():
            if vesc_engine is not None:
                try:
                    vesc_data = vesc_engine.get_sensors_data(
                        ["input_voltage"], vesc_engine.PROPULSION_KEY)
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except:
                    break
                if vesc_data is not None and voltage_thread_alive():
                    #last_update = time.time()
                    sendInputVoltage(socketio, vesc_data["input_voltage"])
                    input_voltage["input_voltage"] = vesc_data["input_voltage"]
        time.sleep(1)


def sendInputVoltage(socketio: SocketIO, input_voltage: Dict[str, str]) -> None:
    """
        Function for sending the voltage to ui.
        
        Args:
            - socketio : socket connected with ui
            - input_voltage
    """
    try:
        input_voltage = round(float(input_voltage) * 2) / 2
    except ValueError:
        pass
    socketio.emit('update', input_voltage,
                  namespace='/voltage', broadcast=True)


def send_last_pos_thread_tf(send_last_pos_thread_alive: bool, socketio: SocketIO, logger: utility.Logger) -> None:
    """
        Function for sending the last position to ui for the map.
        
        Args:
            - send_last_pos_thread_alive
            - socketio : socket connected with ui
            - logger
    """
    with adapters.GPSUbloxAdapterWithoutThread(config.GPS_PORT, config.GPS_BAUDRATE, 1) as gps:
        while send_last_pos_thread_alive():
            lastPos = gps.get_fresh_position()
            if config.ALLOW_GPS_BAD_QUALITY_NTRIP_RESTART and lastPos[2]!='4':
                NavigationV3.restart_ntrip_service(logger)
            socketio.emit('updatePath', json.dumps([[[lastPos[1], lastPos[0]]], lastPos[2]]), namespace='/map', broadcast=True)
            socketio.emit('updateGPSQuality', lastPos[2], namespace='/gps', broadcast=True)


def initVesc(logger: utility.Logger) -> adapters.VescAdapterV4 :
    """
        Function for initializing a VESC.\n
        
        Args:
            logger

        Returns:
            VescAdapterV4: the initialized VESC.
    """
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


def timeout_sm_th(event: Events, logger: utility.Logger) -> None:
    time.sleep(10)
    if not event.is_set():
        msg = "[Timeout sm] Couldn't get SmoothieAdapter!"
        logger.write_and_flush(msg + "\n")
        raise Exception(msg)


def initSmoothie(logger: utility.Logger) -> adapters.SmoothieAdapter:
    """
        Function for initializing a Smoothie.
        
        Args:
            logger

        Returns:
            SmoothieAdapter: the initialized Smoothie.
    """
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


def save_gps_coordinates(points: List[List[float]], file_name: str) -> None:
    """
        Function for saving GPS coordinates in a file.
        
        Args:
            - points: list of coordinates.
            - file_name: name of the file in which the cooridinates will be saved.
    """
    with open(file_name, "w") as file:
        for point in points:
            str_point = str(point[0]) + " " + str(point[1]) + "\n"
            file.write(str_point)
    user = pwd.getpwnam('violette')
    os.chown(file_name, user.pw_uid, user.pw_gid)


def changeConfigValue(path: str, value: str) -> None:
    """
        Function for changing a value in the config file.
        
        Args:
            - path: name of the variable in the config file.
            - value: new value.
    """
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
    """
        Function for starting the main program.
    """
    mainSP = subprocess.Popen("python3 main.py", stdin=subprocess.PIPE, cwd=os.getcwd().split("/uiWebRobot")[0],
                              shell=True, preexec_fn=os.setsid)
    return mainSP


def startLiveCam():
    """
        Function for starting the live camera.
    """
    camSP = subprocess.Popen("python3 serveurCamLive.py", stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, cwd=os.getcwd().split(
        "/uiWebRobot")[0], shell=True,
        preexec_fn=os.setsid)
    return camSP


def updateFields(field_name: str) -> Tuple[List[float], list, str]:
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


def load_field_list(dir_path) -> List[str]:
    field_list = []
    for file in os.listdir(dir_path):
        if file.endswith(".txt"):
            #if file != "tmp.txt":
            field_list.append(
                unquote(file.split(".txt")[0], encoding='utf-8'))
    return field_list


def get_other_field() -> list:
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


def change_state(event : Events) -> None :
    """
        Function for changing state.
        
        Args:
            - event: event of the next state
    """
    io = socketio.Client()
    io.connect(url="http://localhost:80", namespaces="/server")
    io.emit(event="data", data={"type": str(event)}, namespace="/server")


def is_valid_field_file(file_path : str, logger: utility.Logger) -> bool:
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


def get_ui_language() -> Tuple[dict, str]:
    """
        Function for getting ui languages and current language.

        Returns: 
            - dict: ui languages
            - str: current language
    """
    with open("ui_language.json", "r", encoding='utf-8') as read_file:
            ui_languages = json.load(read_file)
    ui_language = config.UI_LANGUAGE
    if ui_language not in ui_languages["Supported Language"]:
        ui_language = "en"
    return ui_languages, ui_language