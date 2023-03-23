import sys
sys.path.append('../')

import time
import adapters
import json
import utility
import pwd
import os
import fileinput
import grp
import subprocess
from urllib.parse import quote, unquote
import telnetlib
import threading

from config import config
import application

def voltage_thread_tf(voltage_thread_alive, vesc_engine: adapters.VescAdapterV4, socketio, input_voltage):
    last_update = 0
    vesc_data = None
    while voltage_thread_alive():
        if time.time() - last_update > 5 and voltage_thread_alive():
            if vesc_engine is not None:
                try:
                    vesc_data = vesc_engine.get_sensors_data(["input_voltage"], vesc_engine.PROPULSION_KEY)
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except:
                    break
                if vesc_data is not None and voltage_thread_alive():
                    last_update = time.time()
                    sendInputVoltage(socketio, vesc_data["input_voltage"])
                    input_voltage["input_voltage"] = vesc_data["input_voltage"]
        time.sleep(1)

def sendInputVoltage(socketio, input_voltage):
    try:
        input_voltage = round(float(input_voltage) * 2) / 2
    except ValueError:
        pass
    socketio.emit('update', input_voltage, namespace='/voltage', broadcast=True)

def send_last_pos_thread_tf(send_last_pos_thread_alive, socketio):
    with adapters.GPSUbloxAdapterWithoutThread(config.GPS_PORT, config.GPS_BAUDRATE, 1) as gps:
        while send_last_pos_thread_alive():
            lastPos = gps.get_fresh_position()
            socketio.emit('updatePath', json.dumps([[[lastPos[1], lastPos[0]]], lastPos[2]]), namespace='/map', broadcast=True)

def initVesc(logger: utility.Logger):
    smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
    if "vesc" in smoothie_vesc_addr:
        vesc_address = smoothie_vesc_addr["vesc"]
    else:
        msg = "Couldn't get vesc's USB address!"
        logger.write_and_flush(msg + "\n")
        print(msg)
        exit(1)
    vesc_engine = adapters.VescAdapterV4(vesc_address,
                                         config.VESC_BAUDRATE,
                                         config.VESC_ALIVE_FREQ,
                                         config.VESC_CHECK_FREQ,
                                         config.VESC_STOPPER_CHECK_FREQ)
    vesc_engine.set_target_rpm(0, vesc_engine.PROPULSION_KEY)
    vesc_engine.set_time_to_move(config.VESC_MOVING_TIME, vesc_engine.PROPULSION_KEY)
    return vesc_engine

def restart_ui_sm_init_error_th(event):
    time.sleep(10)
    if not event.is_set():
        tn = telnetlib.Telnet("192.168.9.101")
        tn.write(b"G28 XY\n")
        tn.read_some()
        tn.write(b"exit\n")
        tn.read_all()
        tn.close()
        os.system("sudo systemctl restart UI.service")

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
            print(msg)
            exit(1)

    event = threading.Event()
    smoothie_creation_thread = threading.Thread(target=restart_ui_sm_init_error_th, args=(event,))
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
    camSP = subprocess.Popen("python3 serveurCamLive.py", stderr=subprocess.DEVNULL, stdin=subprocess.PIPE,
                             stdout=subprocess.DEVNULL, cwd=os.getcwd().split("/uiWebRobot")[0], shell=True,
                             preexec_fn=os.setsid)
    return camSP


def updateFields(field_name):
    field_name = quote(field_name, safe="", encoding='utf-8')

    cmd = "ln -sf 'fields/" + field_name + ".txt' ../field.txt"

    os.system(cmd)

    with open("../field.txt") as file:
        points = file.readlines()

    coords = list()
    for coord in points:
        coord = coord.replace("\n", "").split(" ")
        coords.append([float(coord[1]), float(coord[0])])
    coords.append(coords[0])

    other_fields = application.UIWebRobot.get_other_field()
    current_field_name = subprocess.run(["readlink", "../field.txt"], stdout=subprocess.PIPE).stdout.decode(
        'utf-8').replace("fields/", "")[:-5]

    return coords, other_fields, unquote(current_field_name)