from tkinter import E
from flask import Flask, send_from_directory, make_response, render_template
from flask_socketio import SocketIO, emit
from engineio.payload import Payload
import re
import fileinput
import pwd
import grp
import os

import sys
sys.path.append('../')
from config import config
import adapters
import utility

app = Flask(__name__)

Payload.max_decode_packets = 500
socketio = SocketIO(app, async_mode=None, logger=False, engineio_logger=False)

smoothie: adapters.SmoothieAdapter= None

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
    if smoothie is None:
        smoothie = adapters.SmoothieAdapter(utility.get_smoothie_vesc_addresses()["smoothie"])
    return render_template('x_y_dir.html', A_MAX=config.A_MAX, Y_MAX=config.Y_MAX, X_MAX=config.X_MAX)

@socketio.on('x_y_dir', namespace='/server')
def on_x_y_dir(data):
    global smoothie
    if smoothie is None:
        smoothie = adapters.SmoothieAdapter(utility.get_smoothie_vesc_addresses()["smoothie"])
    if "x" in data:
        smoothie.custom_move_for(X_F=config.X_F_MAX, X=data["x"])
        print(f"x:{data['x']}")
    if "y" in data:
        smoothie.custom_move_for( Y_F=config.Y_F_MAX, Y=data["y"])
        print(f"y:{data['y']}")
    if "a" in data:
        smoothie.custom_move_for(A_F=config.A_F_MAX, A=data["a"])
        print(f"a:{data['a']}")
    if "inv" in data:
        axis_to_config = {"X":config.X_COEFFICIENT_TO_MM,"Y":config.Y_COEFFICIENT_TO_MM,"A":config.A_COEFFICIENT_TO_MM}
        for axis, invert in data["inv"].items():
            if invert:
                changeConfigValue(f"{axis}_COEFFICIENT_TO_MM",axis_to_config[axis])

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
    app.run(host="0.0.0.0",port="80",debug=True, use_reloader=False)