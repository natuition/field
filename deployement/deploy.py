from tkinter import E
from flask import Flask, send_from_directory, make_response, render_template
from flask_socketio import SocketIO, emit
from engineio.payload import Payload
import re
import fileinput
import pwd
import grp
import os
import logging
import time
import serial
import json
from flask_cors import CORS

import importlib
import sys
sys.path.append('../')

module_config = importlib.import_module("..config", 'config.config')
config_vars = {k: v for k, v in module_config.__dict__.items() if not k.startswith('_')}
config = type("config", (object, ), config_vars)

import adapters
import utility
from cameraCalibration import CameraCalibration

app = Flask(__name__)
CORS(app)
app.config['DEBUG'] = False
app.logger.disabled = True
log = logging.getLogger('werkzeug')
log.disabled = True

Payload.max_decode_packets = 500
socketio = SocketIO(app, async_mode=None, logger=False, engineio_logger=False)

smoothie: adapters.SmoothieAdapter= None
cameraCalibration: CameraCalibration = None

@app.route("/show_pdf/<filename>")
def show_pdf(filename):
    if not "path" in send_from_directory.__code__.co_varnames:
        response=make_response(send_from_directory(app.static_folder,filename=f'pdf/{filename}.pdf'))
    else:
        response=make_response(send_from_directory(app.static_folder,path=f'pdf/{filename}.pdf'))
    response.headers['Content-Type'] = 'application/pdf'
    response.headers['Content-Disposition'] = f'inline; filename={filename}.pdf'
    return response

@app.route("/")
def vesc_foc():
    return render_template('vesc_foc.html')

@app.route("/x_y_dir")
def x_y_dir():
    global smoothie
    print(smoothie, in_reset)
    if smoothie is None and not in_reset:
        smoothie = adapters.SmoothieAdapter(utility.get_smoothie_vesc_addresses()["smoothie"], calibration_at_init=False)
    print(smoothie)
    return render_template('x_y_dir.html', A_MAX=config.A_MAX, Y_MAX=config.Y_MAX, X_MAX=config.X_MAX, IN_RESET=json.dumps(in_reset))

@app.route("/camera_focus")
def camera_focus():
    global cameraCalibration
    if cameraCalibration is None:
        cameraCalibration = CameraCalibration()
        cameraCalibration.focus_adjustment_step()
    return render_template('camera_focus.html')

@app.route("/camera_crop_picture")
def camera_crop_picture():
    global cameraCalibration
    if cameraCalibration is None:
        cameraCalibration = CameraCalibration()
    else:
        try:
            cameraCalibration.focus_adjustment_step_validate()
        except:
            pass
    return render_template('camera_crop_picture.html')

@app.route("/camera_target_detection")
def camera_target_detection():
    return render_template('camera_target_detection.html')

@socketio.on('run_round_detection', namespace='/server')
def on_run_round_detection(data):
    global cameraCalibration
    if cameraCalibration is None:
        cameraCalibration = CameraCalibration()
    if data["run_detection"]:
        cameraCalibration.step_crop_picture()
        with open('./scene_center.jpg', 'rb') as f:
            image_data = f.read()
        socketio.emit('image', {'image_data': image_data}, namespace='/server', broadcast=True)

@socketio.on('run_target_detection', namespace='/server')
def on_run_target_detection(data):
    global cameraCalibration
    if cameraCalibration is None:
        cameraCalibration = CameraCalibration()
    if data["run_detection"]:
        res = cameraCalibration.offset_calibration_step_detect()
        with open('./target_detection.jpg', 'rb') as f:
            image_data = f.read()
        socketio.emit('image', {'image_data': image_data, "res": res}, namespace='/server', broadcast=True)

@socketio.on('x_y_dir', namespace='/server')
def on_x_y_dir(data):
    global smoothie
    global in_reset
    if "x" in data and not in_reset:
        smoothie.custom_move_for(X_F=config.X_F_MAX, X=data["x"])
        print(f"x:{data['x']}")
    if "y" in data and not in_reset:
        smoothie.custom_move_for( Y_F=config.Y_F_MAX, Y=data["y"])
        print(f"y:{data['y']}")
    if "a" in data and not in_reset:
        smoothie.custom_move_for(A_F=config.A_F_MAX, A=data["a"])
        print(f"a:{data['a']}")
    if "inv" in data and not in_reset:
        translate_axis_grec = {"x":"alpha_dir_pin","y":"beta_dir_pin","a":"delta_dir_pin"}
        for axis, invert in data["inv"].items():
            if invert:
                cmd = f"config-get sd {translate_axis_grec[axis.lower()]}"
                #print(cmd)
                smoothie.get_connector().write(cmd)
                response = smoothie.get_connector().read_some()
                matches = re.findall(f"{translate_axis_grec[axis.lower()]} is set to (.*?)\n", response)
                if matches:
                    value = matches[0][:-1]
                    if "!" in value:
                        value = value[:-1]
                    else:
                        value = value+"!"
                    cmd = f"config-set sd {translate_axis_grec[axis.lower()]} {value}"
                    #print(cmd)
                    smoothie.get_connector().write(cmd)
                    response = smoothie.get_connector().read_some()
                    #print(response.replace("\n",""))
        in_reset = True
        smoothie.get_connector().write("reset")
        response = smoothie.get_connector().read_some()
        #print(response.replace("\n",""))
        smoothie.disconnect()
        smoothie = None
        print("Reset wait 30 sec")
        time.sleep(30)
        print("Search smoothie...")
        while True:
            print(utility.get_smoothie_vesc_addresses())
            if "smoothie" in utility.get_smoothie_vesc_addresses():
                if smoothie is None:
                    print("Smoothie find.")
                    in_reset = False
                    time.sleep(2)
                    socketio.emit('reload', {"reload": True} , namespace='/server', broadcast=True)
                    break
            else:
                time.sleep(0.5)
        in_reset = False

def changeConfigValue(path: str, value):
    with fileinput.FileInput("../config/config.py", inplace=True, backup='.bak') as file:
        for line in file:
            if path in line:
                print(path + " = " + str(value), end='\n')
            else:
                print(line, end='')
    uid = pwd.getpwnam("violette").pw_uid
    gid = grp.getgrnam("violette").gr_gid
    os.chown("../config/config.py", uid, gid)

if __name__ == "__main__":
    global in_reset
    in_reset = False
    app.run(host="0.0.0.0",port="80",debug=True, use_reloader=False)