from flask_socketio import SocketIO, emit
from flask import Flask, render_template, url_for, copy_current_request_context, redirect, request
from random import random
from time import sleep
from threading import Thread, Event
import os
import fileinput
import pwd
import grp
import subprocess
import time

import sys
sys.path.append('../')
import connectors
import adapters
from config import config
import utility


app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
app.config['DEBUG'] = True

socketio = SocketIO(app, async_mode=None, logger=True, engineio_logger=True)


# ---------------------------------------------------------
# -------------------- utils variables --------------------
# ---------------------------------------------------------

XP = "G0 X20 F4000"
XN = "G0 X-20 F4000"
YP = "G0 Y20 F4000"
YN = "G0 Y-20 F4000"
DP = "G0 A20 F4000"
DN = "G0 A-20 F4000"
ES = "M119"
RPM = 5000
MOVING_TIME = 5
XYD_URL = 'xyd.html'
VESC_Z_URL = 'vesc_z.html'
TEST_Z = "G0 Z20 F1950"

d = utility.get_current_time().split(" ")[0].split("-")
h = utility.get_current_time().split(" ")[1].split("-")
Date = f"{d[0]}/{d[1]}/{d[2]} à {h[0]}h{h[1]}min{h[2]}s"

LOG = {'DATE': Date, 'PIC': 'KO', 'ROBOT_SN': 'KO', 'UI_LANGUAGE': 'KO', 'NTRIP_USER': 'KO', 'NTRIP_PASSWORD' : 'KO', 'NTRIP_CASTER': 'KO', 'CAMERA': 'KO', 'G91': 'KO', XP: 'KO', XN: 'KO', YP: 'KO', YN: 'KO', DP: 'KO', DN: 'KO', 'ESX': 'KO', 'ESY': 'KO', 'ESZ': 'KO', 'VESC_PR_APP': 'KO', 'VESC_PR_FOC': 'KO', 'MOTOR_PR': 'KO', 'VESC_Z_APP': 'KO', 'VESC_Z_FOC': 'KO', 'MOTOR_Z': 'KO', 'GPS_CONFIG': 'KO', 'GPS_TEST': 'KO', 'GPS_TEST_NTRIP': 'KO'}

LOG_PRINT = False

camSP = None

# --------------------------------------------------------
# -------------------- utils function --------------------
# --------------------------------------------------------

def changeConfigValue(path: str, value):
    LOG[path]= value
    if LOG_PRINT:
        print(LOG)
    with fileinput.FileInput("../config/config.py", inplace=True, backup='.bak') as file:
        for line in file:
            if path in line:
                print(path + " = " + '"' +str(value) + '"', end='\n')
            else:
                print(line, end='')
    os.chown("../config/config.py", -1, -1)

def startLiveCam():
    camSP = subprocess.Popen("python3 serveurCamLive.py False", stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE, cwd=os.getcwd().split("/deployement")[0], shell=True, preexec_fn=os.setsid)
    return camSP

def test_smoothie(smoothie, command, url):
    smoothie.write(command)
    LOG[command] = "OK"
    if LOG_PRINT:
        print(LOG)
    return command + ' ' +str(smoothie.read_some())

def testes(smoothie, es):
    if es == 'x':
      index = 6
    elif es == 'y':
      index = 14
    elif es =='z':
      index = 22
    while True:
      smoothie.write(ES)
      s = smoothie.read_some()
      if s=='':
        s = smoothie.read_some()
      elif s[0]=="o":
        s = smoothie.read_some()
      if s[index] == "1":
        while True:
          smoothie.write(ES)
          s = smoothie.read_some()
          if s=='':
            s = smoothie.read_some()
          elif s[0]=="o":
            s = smoothie.read_some()
          if s[index] == "0":
            return "End Stop "+es +" OK"
          time.sleep(0.1)

# --------------------------------------------------------
# -------------------- route function --------------------
# --------------------------------------------------------

@app.route('/', methods=['POST', 'GET'])
def init():
    if request.method == 'POST':
        LOG['PIC']= request.form['Tech']
        changeConfigValue("ROBOT_SN", request.form['SN'])
        changeConfigValue("UI_LANGUAGE", request.form['language'])
        changeConfigValue("NTRIP_USER", request.form['ntripuser'])
        changeConfigValue("NTRIP_PASSWORD", request.form['ntrippswd'])
        changeConfigValue("NTRIP_CASTER", request.form['ntripcaster'])
        return redirect(url_for('cam'))
    else:
        return render_template('init.html')

@app.route('/cam', methods=['POST', 'GET'])
def cam():
    global camSP
    if request.method == 'GET':
        os.system("sudo systemctl restart nvargus-daemon")
        camSP=startLiveCam()
        return render_template('cam.html')
    else:
        LOG['CAMERA']= 'OK'
        if LOG_PRINT:
            print(LOG)
        os.killpg(os.getpgid(camSP.pid), signal.SIGINT)
        camSP.wait()
        return redirect(url_for('xyd'))

@app.route('/xyd', methods=['POST', 'GET'])
def xyd():
    with connectors.SmoothieV11SerialConnector(utility.get_smoothie_vesc_addresses()["smoothie"], config.SMOOTHIE_BAUDRATE) as smoothie:
      smoothie.write("G91")
      smoothie.read_some()
      smoothie.write("G91")
      LOG['G91'] = 'OK'
      if LOG_PRINT:
        print(LOG)
      if request.method == 'GET':
          return render_template('xyd.html', answer=str(utility.get_smoothie_vesc_addresses()["smoothie"]))
      else:
          if request.form['x'] == 'next':
            return redirect(url_for('vesc_pr'))

@app.route('/vesc_pr', methods=['POST', 'GET'])
def vesc_pr():
    if request.method == 'GET':
        return render_template('vesc_pr.html')
    else:
        if request.form['x'] == 'test_motor':
              with adapters.VescAdapter(config.VESC_RPM_SLOW, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                              config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE) as vesc_engine:
                vesc_engine.set_rpm(RPM)
                vesc_engine.set_moving_time(MOVING_TIME)
                vesc_engine.start_moving()
                vesc_engine.wait_for_stop()
              return render_template('vesc_pr.html', answer='test_motor command sent')
        elif request.form['x'] == 'next':
            return redirect(url_for('vesc_z'))

@app.route('/vesc_z', methods=['POST', 'GET'])
def vesc_z():
    with connectors.SmoothieV11SerialConnector(utility.get_smoothie_vesc_addresses()["smoothie"], config.SMOOTHIE_BAUDRATE) as smoothie:
        smoothie.write("G91")
        smoothie.read_some()
        smoothie.write("G91")
        smoothie.read_some()
        if request.method == 'GET':
            return render_template('vesc_z.html')
        else:
            if request.form['x'] == 'test_motor':
                return test_smoothie(smoothie, TEST_Z, VESC_Z_URL)        
                return render_template('vesc_pr.html', answer='test_motor command sent')
            elif request.form['x'] == 'next':
                return redirect(url_for('gps'))

@app.route('/gps', methods=['POST', 'GET'])
def gps():
    if request.method == 'GET':
        return render_template('gps.html')
    else:
        if request.form['x'] == 'test_gps':
          with adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP) as gps:
            return render_template('gps.html', answer = gps.get_fresh_position())
        return redirect(url_for('final'))

@app.route('/final', methods=['POST', 'GET'])
def final():
    if request.method == 'GET':
        if LOG_PRINT:
            print(LOG)
        return render_template('final.html', answer = LOG)
    else:
        return redirect(url_for('final'))

# -----------------------------------------------------------
# -------------------- socketio function --------------------
# -----------------------------------------------------------

@socketio.on('connect', namespace='/server')
def test_connect():
    print('Client connected')

@socketio.on('disconnect', namespace='/server')
def test_disconnect():
    print('Client disconnected')

@socketio.on('data', namespace='/server')
def on_socket_data(data):
    print(data)
    with connectors.SmoothieV11SerialConnector(utility.get_smoothie_vesc_addresses()["smoothie"], config.SMOOTHIE_BAUDRATE) as smoothie:
        smoothie.write("G91")
        smoothie.read_some()
        smoothie.write("G91")
        LOG['G91'] = 'OK'
        if LOG_PRINT:
            print(LOG)
        if data == "x+":
            to_emit = test_smoothie(smoothie, XP, XYD_URL)
        elif data == "x-": 
            to_emit = test_smoothie(smoothie, XN, XYD_URL),
        elif data == "y+": 
            to_emit = test_smoothie(smoothie, YP, XYD_URL),
        elif data == "y-": 
            to_emit = test_smoothie(smoothie, YN, XYD_URL),
        elif data == "d+": 
            to_emit = test_smoothie(smoothie, DP, XYD_URL),
        elif data == "d-": 
            to_emit = test_smoothie(smoothie, DN, XYD_URL),
        elif data == "esx": 
            to_emit = testes(smoothie, "x"),
        elif data == "esy": 
            to_emit = testes(smoothie, "y"),
        elif data == "esz": 
            to_emit = testes(smoothie, "z")
        socketio.emit("smoothie_return", to_emit, namespace='/server')

# --------------------------------------------------------


if __name__ == '__main__':
    socketio.run(app, host="0.0.0.0", port=80, debug=False)
