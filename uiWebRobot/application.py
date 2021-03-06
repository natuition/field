import sys
sys.path.append("../.")

import VescInteractiveAdapter
import adapters
import navigation
import utility
import manageConfig

from flask_socketio import SocketIO, emit
from flask import Flask, render_template, url_for, copy_current_request_context,make_response,send_from_directory
from threading import Thread, Event
import queue
import time
import logging
import subprocess
import signal
import json

__author__ = 'Vincent LAMBERT'

app = Flask(__name__)
app.config['DEBUG'] = False
app.logger.disabled = True
log = logging.getLogger('werkzeug')
log.disabled = True
socketio = SocketIO(app, async_mode=None, logger=False, engineio_logger=False)

path_field = "../field.txt"
path_layers = "static/layers/"

def sendHistoryOfVoltage(voltages: list):
    socketio.emit('newHistoryVoltage', voltages, namespace='/voltage')

def sendNewVoltage(voltage: dict):
    historyOfVoltages.append(voltage)
    socketio.emit('newVoltage', voltage, namespace='/voltage')

def getCorners():
    corners = list()
    with open(path_field, "r") as f:
        for line in f.readlines():
            corners.append(line.replace("\n","").split(" "))
    return corners

def getCenter():
    corners = list()
    with open(path_field, "r") as f:
        for line in f.readlines():
            corners.append(line.replace("\n","").split(" "))
    return [((float(corners[0][0])+float(corners[1][0])+float(corners[2][0])+float(corners[3][0]))/4)  , ((float(corners[0][1])+float(corners[1][1])+float(corners[2][1])+float(corners[3][1]))/4)]

def setStatusOfUIObject(arrowPropulsion=True,arrowDirection=True,fieldB=True,startB=True,continueB=True,stopB=None):
        statusOfUIObject["arrowsPropulsionStatus"] = arrowPropulsion
        statusOfUIObject["arrowsDirectionStatus"] = arrowDirection             
        statusOfUIObject["fieldButton"] = fieldB
        statusOfUIObject["startButton"] = startB
        statusOfUIObject["continueButton"] = continueB
        statusOfUIObject["stopButton"] = stopB
        #print(statusOfUIObject)

def initVesc():
    global vesc_engine
    configList = config.getValues(["VESC_RPM_UI", "VESC_ALIVE_FREQ", "VESC_CHECK_FREQ", "VESC_BAUDRATE"])
    vesc_engine =  VescInteractiveAdapter.VescInteractiveAdapter(configList["VESC_RPM_UI"],configList["VESC_ALIVE_FREQ"],configList["VESC_CHECK_FREQ"],vesc_address,configList["VESC_BAUDRATE"])
    vesc_engine.set_rpm(config.getValue("VESC_RPM_UI"))

def initSmoothie():
    global smoothie
    smoothie =  adapters.SmoothieAdapter(smoothie_address)

def initVescAndSmoothie():
    initVesc()
    initSmoothie()   
    
def disconnectVescAndSmoothie():
    vesc_engine.disconnect()
    smoothie.disconnect()

def emergency_field_defining_start(gps: adapters.GPSUbloxAdapter, nav: navigation.GPSComputing, field_size: int):
    global vesc_emergency
    global first_point_emergency
    global field_size_emergency

    field_size_emergency = field_size
    configList = config.getValues(["VESC_RPM_UI", "VESC_MOVING_TIME", "VESC_ALIVE_FREQ", "VESC_CHECK_FREQ", "VESC_PORT", "VESC_BAUDRATE"])
    vesc_emergency = adapters.VescAdapter(configList["VESC_RPM_UI"], configList["VESC_MOVING_TIME"], configList["VESC_ALIVE_FREQ"], configList["VESC_CHECK_FREQ"], configList["VESC_PORT"], configList["VESC_BAUDRATE"])
    
    msg = "Using emergency field creation..."
    print(msg)

    first_point_emergency = gps.get_last_position()

    msg = "Moving forward..."
    print(msg)
    vesc_emergency.start_moving()

def emergency_field_defining_end(gps: adapters.GPSUbloxAdapter, nav: navigation.GPSComputing):
    
    msg = "Stop moving forward..."
    print(msg)

    vesc_emergency.stop_moving()

    msg = "Getting point A..."
    print(msg)
    time.sleep(1)
    
    A = first_point_emergency
    B = gps.get_last_position()

    length_field = field_size_emergency
    width_field = nav.get_distance(A,B)

    msg = "Computing rest points... field_size : {} (length) / {} (width)."
    print(msg.format(length_field,width_field))

    C = nav.get_coordinate(B, A, 90, length_field)
    D = nav.get_coordinate(C, B, 90, width_field)

    msg = "Saving field.txt file..."
    print(msg)
    field = [B, C, D, A]
    save_gps_coordinates(field, "../field.txt")
    return field
    
def save_gps_coordinates(points: list, file_name):
    with open(file_name, "w") as file:
        for point in points:
            str_point = str(point[0]) + " " + str(point[1]) + "\n"
            file.write(str_point)
            
def startMain():
    print("Start main.py...")
    global mainSP
    mainSP = subprocess.Popen(["python3","main.py"], cwd="/home/violette/field")#, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)

@app.before_first_request
def initAll():
    app.logger.info("Init before first request")
    global vesc_engine
    global mainSP
    global config
    global vesc_address
    global smoothie_address
    global historyOfVoltages
    global statusOfUIObject
    global socketio
    global robot_sn
    global ui_languages
    with open("ui_language.json", "r") as read_file:
        ui_languages = json.load(read_file)
    config = manageConfig.ManageConfig("../config/config.py")
    robot_sn = config.getValue("ROBOT_SN")
    historyOfVoltages = list()
    statusOfUIObject = {
        "arrowsPropulsionStatus": True, #True or False
        "arrowsDirectionStatus": True, #True or False
        "fieldButton": True, #True or False or charging or None
        "startButton": True, #True or False or charging or None
        "continueButton": True, #True or False or charging or None
        "stopButton": None #True or charging
    }
    # get smoothie and vesc addresses
    smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
    if "vesc" in smoothie_vesc_addr:
        vesc_address = smoothie_vesc_addr["vesc"]
    else:
        msg = "Couldn't get vesc's USB address!"
        print(msg)
        exit(1)
    if config.getValue("SMOOTHIE_BACKEND") == 1:
        smoothie_address = config.getValue("SMOOTHIE_HOST")
    else:
        if "smoothie" in smoothie_vesc_addr:
            smoothie_address = smoothie_vesc_addr["smoothie"]
        else:
            msg = "Couldn't get smoothie's USB address!"
            print(msg)
            exit(1)
    initVescAndSmoothie()

@app.route('/')
def index():
    if(statusOfUIObject["arrowsPropulsionStatus"]):
        arrowsPropulsionStatus = "arrowStop"
    else:
        arrowsPropulsionStatus = "arrowOff"
        
    if(statusOfUIObject["arrowsDirectionStatus"]):
        arrowsDirectionStatus = "arrowStop"
    else:
        arrowsDirectionStatus = "arrowOff"
    ui_language = "en"
    config_ui_language = config.getValue("UI_LANGUAGE")
    if config_ui_language in ui_languages["Supported Language"]:
        ui_language = config.getValue("UI_LANGUAGE")
    return render_template('UIRobot.html',sn=robot_sn, arrowsPropulsionStatus=arrowsPropulsionStatus, arrowsDirectionStatus=arrowsDirectionStatus, fieldButton=statusOfUIObject["fieldButton"], startButton=statusOfUIObject["startButton"], continueButton=statusOfUIObject["continueButton"], stopButton=statusOfUIObject["stopButton"], ui_languages=ui_languages, ui_language=ui_language)    

@app.route('/voltage')
def volatge():
    return render_template('voltage.html')

@app.route('/map')
def accueil():

    headerBaliseFile = 'var json_field_balise_3 = {"type": "FeatureCollection","name": "field_balise_3","crs": { "type": "name", "properties": { "name": "urn:ogc:def:crs:OGC:1.3:CRS84" } },"features": ['
    headerCornerFile = 'var json_field_corner_4 = {"type": "FeatureCollection","name": "field_corner_4","crs": { "type": "name", "properties": { "name": "urn:ogc:def:crs:OGC:1.3:CRS84" } },"features": ['
    headerContourFile = 'var json_field_contour_2 = {"type": "FeatureCollection","name": "field_contour_2","crs": { "type": "name", "properties": { "name": "urn:ogc:def:crs:OGC:1.3:CRS84" } },"features": ['
    endFile= ']}'

    with open(path_layers+"field_balise_3.js", "w") as f:
        f.write(headerBaliseFile)
        f.write(endFile)

    with open(path_layers+"field_corner_4.js", "w") as f:
        f.write(headerCornerFile)
        for corner in getCorners():
            f.write('{ "type": "Feature", "properties": { "id": 0.0, "iscorner": "True", "robot": "SN005", "isbase": "False" }, "geometry": { "type": "Point", "coordinates": [ '+corner[1]+','+corner[0]+'] } },')
        f.write(endFile)

    with open(path_layers+"field_contour_2.js", "w") as f:
        f.write(headerContourFile)
        f.write(endFile)

    return render_template('map.html', titre="Map !")

@socketio.on('connect', namespace='/voltage')
def test_connect():
    sendHistoryOfVoltage(historyOfVoltages)

@socketio.on('connect', namespace='/coords')
def connect():
    socketio.emit('centerOfMap', getCenter(), namespace='/coords')

@socketio.on('direction', namespace='/navigation')
def handle_dir(status):
    #print(status)
    socketio.emit('direction', status, namespace='/navigation', broadcast=True)
    #Ask the smoothie to spin the wheels
    if status["direction"] == "arrow_left":
        smoothie.nav_turn_wheels_for(5,4000)
    elif status["direction"] == "arrow_right":
        smoothie.nav_turn_wheels_for(-5,4000)
    time.sleep(0.25)
    status["status"]=False
    socketio.emit('direction', status, namespace='/navigation', broadcast=True)

@socketio.on('propulsion', namespace='/navigation')
def handle_propulsion(status):
    socketio.emit('propulsion', status, namespace='/navigation', broadcast=True)
    if status["propulsion"] == "arrow_up" and status["status"]:
        vesc_engine.start_moving_forward()
    elif status["propulsion"] == "arrow_down" and status["status"]:
        vesc_engine.start_moving_backward()
    elif not status["status"]:
        vesc_engine.stop_moving()

@socketio.on('alive', namespace='/navigation')
def handle_alive(status):
    vesc_engine.alive()

@socketio.on('field', namespace='/button')
def handle_field(status):
    #print("field :",status)
    if status["status"] == "pushed":
        setStatusOfUIObject(arrowPropulsion=False,arrowDirection=True,fieldB="charging",startB=False,continueB=False,stopB=None)
        socketio.emit('field', status, namespace='/button', broadcast=True)
        #Emmergency start process
        disconnectVescAndSmoothie()

        nav = navigation.GPSComputing()
        configList = config.getValues(["GPS_PORT", "GPS_BAUDRATE", "GPS_POSITIONS_TO_KEEP"])
        with adapters.GPSUbloxAdapter(configList["GPS_PORT"], configList["GPS_BAUDRATE"], configList["GPS_POSITIONS_TO_KEEP"]) as gps:
                emergency_field_defining_start(gps,nav,(int(status["value"])*1000))

        initSmoothie()
        #Emmergency start process
        status["status"]  = "inRun"
        setStatusOfUIObject(arrowPropulsion=False,arrowDirection=True,fieldB=None,startB=False,continueB=False,stopB=None)
        socketio.emit('field', status, namespace='/button', broadcast=True)
    

@socketio.on('start', namespace='/button')
def handle_start(status):
    #print("start :",status["status"])
    if status["status"] == "pushed":
        setStatusOfUIObject(arrowPropulsion=False,arrowDirection=False,fieldB=False,startB="charging",continueB=False,stopB=None)
        socketio.emit('start', status, namespace='/button', broadcast=True)
        disconnectVescAndSmoothie()
        #Start main
        if config.getValue("CONTINUE_PREVIOUS_PATH"):
            config.setValue("CONTINUE_PREVIOUS_PATH",False)
        startMain()
        #Start main
        print("Waiting 10 sec for main start..")
        time.sleep(10)
    status["status"]  = "finish"
    setStatusOfUIObject(arrowPropulsion=False,arrowDirection=False,fieldB=False,startB=None,continueB=False,stopB=True)
    socketio.emit('start', status, namespace='/button', broadcast=True)

@socketio.on('continue', namespace='/button')
def handle_continue(status):
    #print("continue :",status["status"])
    if status["status"] == "pushed":
        setStatusOfUIObject(arrowPropulsion=False,arrowDirection=False,fieldB=False,startB=False,continueB="charging",stopB=None)
        socketio.emit('continue', status, namespace='/button', broadcast=True)
        disconnectVescAndSmoothie()
        #Continue main
        if not config.getValue("CONTINUE_PREVIOUS_PATH"):
            config.setValue("CONTINUE_PREVIOUS_PATH",True)
        startMain()
        #Continue main
        print("Waiting 10 sec for main start..")
        time.sleep(10)
        print("Main are start")
    status["status"]  = "finish"
    setStatusOfUIObject(arrowPropulsion=False,arrowDirection=False,fieldB=False,startB=False,continueB=None,stopB=True)
    socketio.emit('continue', status, namespace='/button', broadcast=True)

@socketio.on('stop', namespace='/button')
def handle_stop(status):
    #print("stop :",status["status"])
    if status["status"] == "pushed":
        if statusOfUIObject["fieldButton"] == None:
            setStatusOfUIObject(arrowPropulsion=False,arrowDirection=True,fieldB=statusOfUIObject["fieldButton"],startB=statusOfUIObject["startButton"],continueB=statusOfUIObject["continueButton"],stopB="charging")
            status["status"] = "pushedF"
            socketio.emit('stop', status, namespace='/button', broadcast=True)
            #Emmergency end process
            smoothie.disconnect()

            nav = navigation.GPSComputing()
            configList = config.getValues(["GPS_PORT", "GPS_BAUDRATE", "GPS_POSITIONS_TO_KEEP"])
            with adapters.GPSUbloxAdapter(configList["GPS_PORT"], configList["GPS_BAUDRATE"], configList["GPS_POSITIONS_TO_KEEP"]) as gps:
                emergency_field_defining_end(gps,nav)

            initVescAndSmoothie()
            #Emmergency end process
            status["status"]  = "finish"
            setStatusOfUIObject(continueB=False)
            socketio.emit('field', status, namespace='/button', broadcast=True)
        else: 
            setStatusOfUIObject(arrowPropulsion=False,arrowDirection=False,fieldB=statusOfUIObject["fieldButton"],startB=statusOfUIObject["startButton"],continueB=statusOfUIObject["continueButton"],stopB="charging")
            socketio.emit('stop', status, namespace='/button', broadcast=True)
            #Stop main
            mainSP.send_signal(signal.SIGINT)
            time.sleep(2)
            setStatusOfUIObject(arrowPropulsion=True,arrowDirection=True,fieldB=True,startB=True,continueB=True,stopB=None)
            initVescAndSmoothie()

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
    app.logger.info("Start of application.py")
    app.run(host="0.0.0.0",port="80",debug=True)
   
