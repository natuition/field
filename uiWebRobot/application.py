import sys
sys.path.append('../')
from config import config
from flask_socketio import SocketIO, emit
from engineio.payload import Payload
from werkzeug.exceptions import HTTPException
from flask import Flask, render_template,make_response,send_from_directory, request
import os
import logging
import json
import stateMachine
from state import Events
import subprocess
import my_states
import os
from urllib.parse import quote, unquote

__author__ = 'Vincent LAMBERT'

app = Flask(__name__)
app.config['DEBUG'] = False
app.logger.disabled = True
log = logging.getLogger('werkzeug')
log.disabled = True
Payload.max_decode_packets = 500
socketio = SocketIO(app, async_mode=None, logger=False, engineio_logger=False)

def init():
    global ui_languages
    global stateMachine
    with open("ui_language.json", "r", encoding='utf-8') as read_file:
        ui_languages = json.load(read_file)    
    stateMachine = stateMachine.StateMachine(socketio)

def load_coordinates(file_path):
    positions_list = []
    try:
        with open(file_path) as file:
            for line in file:
                if line != "":
                    coords = list(map(float, line.split(" ")))
                    positions_list.append([coords[0],coords[1]])
    except OSError as e:
        return None
    return positions_list

def load_ai_list(dir_path):
    ia_list = []
    for file in os.listdir(dir_path):
        if file.endswith(".conf"):
            ia_list.append(file.split(".conf")[0])
    return ia_list

def load_field_list(dir_path):
    field_list = []
    for file in os.listdir(dir_path):
        if file.endswith(".txt"):
            field_list.append(unquote(file.split(".txt")[0]))
    return field_list

def get_other_field():
    current_field = subprocess.run(["readlink","../field.txt"], stdout=subprocess.PIPE).stdout.decode('utf-8').replace("fields/", "")[:-5]
    field_list = load_field_list("../fields")
    if len(field_list)>=2:
        coords_other = []
        for field_name in field_list:
            if field_name != unquote(current_field):
                with open("../fields/"+quote(field_name,safe="")+".txt") as file:
                    points = file.readlines()
                
                coords = list()
                for coord in points:
                    coord = coord.replace("\n","").split(" ")
                    coords.append([float(coord[1]),float(coord[0])])
                coords.append(coords[0])
                coords_other.append(coords)
        return coords_other
    return list()

def formattingFieldPointsForSend(corners):
    coords = list()

    for coord in corners:
        coords.append([coord[1],coord[0]])

    coords.append(coords[0])

    return coords

@socketio.on('data', namespace='/server')
def on_socket_data(data):
    if "type" in data: 
        if data["type"] == "joystick" and str(stateMachine.currentState) in ["WaitWorkingState","CreateFieldState"]:
            stateMachine.on_socket_data(data)
        elif data["type"] == "field":
            stateMachine.on_event(Events.CREATE_FIELD)
            stateMachine.on_socket_data(data)
        elif data["type"] == "field_name":
            stateMachine.on_socket_data(data)
            stateMachine.on_event(Events.VALIDATE_FIELD_NAME)
        elif data["type"] == "validerZone":
            data["client_id"] = request.sid
            stateMachine.on_socket_data(data)
            stateMachine.on_event(Events.VALIDATE_FIELD)
        elif data["type"] == "start":
            if data["audit"]:
                stateMachine.on_event(Events.START_AUDIT)
            else:
                stateMachine.on_event(Events.START_MAIN)
        elif data["type"] == "continue":
            if data["audit"]:
                stateMachine.on_event(Events.CONTINUE_AUDIT)
            else:
                stateMachine.on_event(Events.CONTINUE_MAIN)
        elif data["type"] == "stop":
            stateMachine.on_event(Events.STOP)
        elif data["type"] == "allChecked":
            stateMachine.on_socket_data(data)
            stateMachine.on_event(Events.LIST_VALIDATION)
        elif data["type"] == "wheel":
            stateMachine.on_event(Events.WHEEL)
        elif data["type"] == "modifyZone":
            stateMachine.on_socket_data(data)
        elif data["type"] == "getField":
            stateMachine.on_socket_data(data)
        elif data["type"] == "getStats":
            stateMachine.on_socket_data(data)
        elif data["type"] == "removeField":
            if isinstance(stateMachine.currentState,my_states.WaitWorkingState):
                stateMachine.on_socket_data(data)


@socketio.on('data', namespace='/broadcast')
def on_socket_broadcast(data):
    if data["type"] == "audit":
        if data["audit"]:
            stateMachine.on_event(Events.AUDIT_ENABLE)
        else:
            stateMachine.on_event(Events.AUDIT_DISABLE)
    emit(data["type"], data, broadcast=True)

@socketio.on('disconnect')
def on_disconnect():
    if str(stateMachine.currentState) in ["WaitWorkingState","CreateFieldState"]:
        stateMachine.on_socket_data({"type": "joystick", "x" : 0 , "y" : 0})

@app.route('/')
def index():
    #ui_language = "fr"
    ui_language = config.UI_LANGUAGE
    if ui_language not in ui_languages["Supported Language"]:
        ui_language = "en"
    sn = config.ROBOT_SN
    #sn = "SNXXX"
    statusOfUIObject = stateMachine.getStatusOfControls()

    IA_list = load_ai_list("../yolo")
    Field_list = load_field_list("../fields")

    if not Field_list:
        Field_list = None
        current_field = None
    else:
        Field_list.sort(key=str.casefold)
        current_field = subprocess.run(["readlink","../field.txt"], stdout=subprocess.PIPE).stdout.decode('utf-8').replace("fields/", "")[:-5]
        current_field = unquote(current_field)

    if str(stateMachine.currentState) == "ErrorState":
        render_template("Error.html",sn=sn, error_message=ui_languages["Error_500"][ui_language]), 500

    return render_template('UIRobot.html',sn=sn, statusOfUIObject=statusOfUIObject, ui_languages=ui_languages, ui_language=ui_language, Field_list=Field_list, current_field=current_field, IA_list=IA_list)    

@app.route('/map')
def maps():
    myCoords=[0,0]
    field = stateMachine.getField()
    if field is None:
        field = load_coordinates("../field.txt")
    if field is None:
        return render_template('map.html', myCoords=myCoords)
    else:
        coords_other = get_other_field()
        coords_field = formattingFieldPointsForSend(field)
        if coords_other:
            return render_template('map.html', coords_field=coords_field, myCoords=myCoords, coords_other=coords_other)
        else:
            return render_template('map.html', coords_field=coords_field, myCoords=myCoords)

@app.route('/offline.html')
def offline():
    sn = config.ROBOT_SN
    ui_language = config.UI_LANGUAGE
    if ui_language not in ui_languages["Supported Language"]:
        ui_language = "en"
    return render_template('offline.html',sn=sn, ui_languages=ui_languages, ui_language=ui_language)

@app.route('/styles.css')
def style():
    response=make_response(send_from_directory('static',filename='css/style.css'))
    response.headers['Content-Type'] = 'text/css'
    return response

@app.errorhandler(Exception)
def handle_exception(e):
    # pass through HTTP errors
    if isinstance(e, HTTPException):
        return e

    # now you're handling non-HTTP exceptions only
    print(e)
    stateMachine.on_event(Events.ERROR)
    sn = config.ROBOT_SN
    ui_language = config.UI_LANGUAGE
    if ui_language not in ui_languages["Supported Language"]:
        ui_language = "en"
    return render_template("Error.html",sn=sn, error_message=ui_languages["Error_500"][ui_language]), 500

@app.route('/sw.js')
def worker():
    response=make_response(send_from_directory('static',filename='js/offline_worker.js'))
    response.headers['Content-Type'] = 'application/javascript'
    return response

@app.route('/js/socket.io.min.js')
def socket_io_min():
    response=make_response(send_from_directory('static',filename='js/socket.io.min.js'))
    response.headers['Content-Type'] = 'application/javascript'
    return response

if __name__ == "__main__":
    init()
    app.run(host="0.0.0.0",port="80",debug=True, use_reloader=False)