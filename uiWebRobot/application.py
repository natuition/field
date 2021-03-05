import sys
sys.path.append('../')
from config import config
from flask_socketio import SocketIO, emit
from engineio.payload import Payload
from werkzeug.exceptions import HTTPException
from flask import Flask, render_template, url_for, copy_current_request_context,make_response,send_from_directory, Response
import time
import logging
import json
import stateMachine
from state import Events

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
    with open(file_path) as file:
        for line in file:
            if line != "":
                coords = list(map(float, line.split(" ")))
                positions_list.append([coords[0],coords[1]])
    return positions_list

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
        elif data["type"] == "validerZone":
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
            stateMachine.on_event(Events.LIST_VALIDATION)
        elif data["type"] == "wheel":
            stateMachine.on_event(Events.WHEEL)
        elif data["type"] == "modifyZone":
            stateMachine.on_socket_data(data)
        elif data["type"] == "getStats":
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
    sn = config.ROBOT_SN
    #sn = "SNXXX"
    statusOfUIObject = stateMachine.getStatusOfControls()

    if str(stateMachine.currentState) == "ErrorState":
        return render_template("500.html",sn=sn), 500

    return render_template('UIRobot.html',sn=sn, statusOfUIObject=statusOfUIObject, ui_languages=ui_languages, ui_language=ui_language)    

@app.route('/map')
def maps():
    myCoords=[0,0]
    field = stateMachine.getField()
    if field is None:
        field = load_coordinates("../field.txt")
    return render_template('map.html', coords=formattingFieldPointsForSend(field), myCoords=myCoords)

@app.errorhandler(Exception)
def handle_exception(e):
    # pass through HTTP errors
    if isinstance(e, HTTPException):
        return e

    # now you're handling non-HTTP exceptions only
    print(e)
    stateMachine.on_event(Events.ERROR)
    sn = config.ROBOT_SN
    return render_template("500.html",sn=sn), 500

@app.route('/js/worker.js')
def worker():
    response=make_response(send_from_directory('static',filename='js/worker.js'))
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