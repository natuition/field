import pwd
import grp
import os
import signal
import sys
sys.path.append('../')
from config import config
from state import State
from state import Events
import adapters
import navigation
import utility, datacollection
import time
import subprocess
import stateMachine
from flask_socketio import SocketIO, send
import fileinput
import json
import serial
import threading
import pathlib
import re
from datetime import datetime, timezone
import posix_ipc
from application import get_other_field, load_field_list
from urllib.parse import quote, unquote

#This state were robot is start, this state corresponds when the ui reminds the points to check before launching the robot.
class CheckState(State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger):
        self.socketio = socketio
        self.logger = logger
        
        try:
            msg = f"[{self.__class__.__name__}] -> startLiveCam"
            self.logger.write_and_flush(msg+"\n")
            print(msg)
            self.cam = startLiveCam()
        except:
            self.on_event(Events.ERROR)
        
        self.statusOfUIObject = {
            "GPSWork" : False
        }

        self.field = None

        #self.thread = threading.Thread(target=checkHaveGPS, args=[self.socketio,self.statusOfUIObject], daemon=True)
        #self.thread.start()

    def on_event(self, event):
        if event == Events.LIST_VALIDATION:
            self.cam.send_signal(signal.SIGINT)
            self.cam.send_signal(signal.SIGINT)
            self.cam.wait()
            os.system("sudo systemctl restart nvargus-daemon")
            if config.NTRIP:
                os.system("sudo systemctl restart ntripClient.service")
            return WaitWorkingState(self.socketio, self.logger, False)
        else:
            try:
                self.cam.send_signal(signal.SIGINT)
                self.cam.wait()
            except:
                pass
            return ErrorState(self.socketio, self.logger)

    def on_socket_data(self, data):
        if data["type"] == 'allChecked':
            with open("../yolo/"+data["strategy"]+".conf") as file:
                for line in file.readlines():
                    content = line.split("#")[0]
                    content = re.sub('[^0-9a-zA-Z._="/]+', '', content)
                    if content:
                        changeConfigValue(content.split("=")[0],content.split("=")[1])
            return self
        else:
            return ErrorState(self.socketio, self.logger)


    def getStatusOfControls(self):
        return self.statusOfUIObject
    
    def getField(self):
        return self.field


#This state corresponds when the robot is waiting to work, during this state we can control it with the joystick.
class WaitWorkingState(State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger, createField: bool):
        self.socketio = socketio
        self.logger = logger
        try:
            #msg = f"[{self.__class__.__name__}] -> initGPS"
            #self.logger.write_and_flush(msg+"\n")
            #print(msg)
            #self.gps = adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP)
            msg = f"[{self.__class__.__name__}] -> initVesc"
            self.logger.write_and_flush(msg+"\n")
            print(msg)
            self.vesc_engine = initVesc(self.logger)
            self.vesc_engine.start_moving()

            msg = f"[{self.__class__.__name__}] -> initSmoothie"
            self.logger.write_and_flush(msg+"\n")
            print(msg)
            self.smoothie = initSmoothie(self.logger)
        except:
            self.on_event(Events.ERROR)


        self.lastValueX = 0
        self.lastValueY = 0

        self.statusOfUIObject = {
            "joystick": True, #True or False
            "fieldButton": True, #True or False or charging or None
            "startButton": True, #True or False or charging or None
            "continueButton": True, #True or False or charging or None
            "stopButton": None, #True or charging or None
            "wheelButton": True, #True or False
            "audit": False, #True or False or use or not-use
            "slider": 25, #Int for slider value
            "removeFieldButton": True #True or False     
        }

        if createField:
            self.statusOfUIObject["continueButton"] = False

        self.socketio.emit('checklist', {"status": "refresh"}, namespace='/server', broadcast=True)

        self.field = None

        #lastPos = self.gps.get_last_position()
        #socketio.emit('newPos', json.dumps([lastPos[1],lastPos[0]]), namespace='/map')

    def on_event(self, event):
        if event == Events.CREATE_FIELD:
            self.statusOfUIObject["fieldButton"] = "charging"
            self.statusOfUIObject["startButton"] = False
            self.statusOfUIObject["continueButton"] = False
            self.statusOfUIObject["joystick"] = False
            self.statusOfUIObject["audit"] = 'disable'
            self.smoothie.disconnect()
            self.vesc_engine.disconnect()
            #self.gps.disconnect()
            return CreateFieldState(self.socketio, self.logger)
        elif event in [Events.START_MAIN,Events.START_AUDIT]:
            self.statusOfUIObject["startButton"] = "charging"
            self.statusOfUIObject["fieldButton"] = False
            self.statusOfUIObject["continueButton"] = False
            self.statusOfUIObject["joystick"] = False
            if event == Events.START_MAIN:
                self.statusOfUIObject["audit"] = 'not-use'
            elif event == Events.START_AUDIT:
                self.statusOfUIObject["audit"] = 'use'
            self.smoothie.disconnect()
            self.vesc_engine.disconnect()
            #self.gps.disconnect()
            if event == Events.START_AUDIT:
                return StartingState(self.socketio, self.logger, True)
            return StartingState(self.socketio, self.logger)
        elif event in [Events.CONTINUE_MAIN,Events.CONTINUE_AUDIT]:
            self.statusOfUIObject["continueButton"] = "charging"
            self.statusOfUIObject["startButton"] = False
            self.statusOfUIObject["fieldButton"] = False
            self.statusOfUIObject["joystick"] = False
            if event == Events.CONTINUE_MAIN:
                self.statusOfUIObject["audit"] = 'not-use'
            elif event == Events.CONTINUE_AUDIT:
                self.statusOfUIObject["audit"] = 'use'
            self.smoothie.disconnect()
            self.vesc_engine.disconnect()
            #self.gps.disconnect()
            if event == Events.CONTINUE_AUDIT:
                return ResumeState(self.socketio, self.logger, True)
            return ResumeState(self.socketio, self.logger)
        elif event == Events.WHEEL:
            self.smoothie.freewheels()
            return self
        elif event == Events.AUDIT_ENABLE:
            self.statusOfUIObject["audit"] = True
            return self
        elif event == Events.AUDIT_DISABLE:
            self.statusOfUIObject["audit"] = False
            return self
        else:
            try:
                self.smoothie.disconnect()
                self.vesc_engine.disconnect()
                #self.gps.disconnect()
            except:
                pass
            return ErrorState(self.socketio, self.logger)
    
    def on_socket_data(self, data):
        #print(f"[{self.__class__.__name__}] -> Move '"+str(data["x"])+"','"+str(data["y"])+"'.")
        if data["type"] == 'joystick':
            x = int(data["x"])
            if x < 0:
                x *= -(config.A_MIN/100)
            if x > 0:
                x *= config.A_MAX/100
            y = int(data["y"])
            if self.lastValueX != x:
                self.smoothie.custom_move_to(A_F=config.A_F_UI, A=x)
                self.lastValueX = x
            if self.lastValueY != y:
                if y > 15 or y < -15:
                    y = (y/100) * (config.VESC_RPM_UI*0.9) + (config.VESC_RPM_UI/10)
                else:
                    y = 0
                self.vesc_engine.apply_rpm(y)
                self.lastValueY = y
                
        if data["type"] == 'getField':
            
            coords, other_fields, current_field_name = updateFields(data["field_name"])
            fields_list = load_field_list("../fields")
            self.socketio.emit('newField', json.dumps({"field" : coords, "other_fields" : other_fields, "current_field_name" : current_field_name, "fields_list": fields_list}), namespace='/map')

        if data["type"] == 'removeField':

            os.remove("../fields/"+quote(data["field_name"],safe="")+".txt")
            fields_list = load_field_list("../fields")

            if len(fields_list) > 0:
                os.system("ln -sf 'fields/"+quote(fields_list[0],safe="")+".txt' ../field.txt")
                coords, other_fields, current_field_name = updateFields(fields_list[0])
            else:
                coords, other_fields, current_field_name = list(), list(), ""

            self.socketio.emit('newField', json.dumps({"field" : coords, "other_fields" : other_fields, "current_field_name" : current_field_name, "fields_list": fields_list}), namespace='/map')

        return self

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field


#This state corresponds when the robot is generating the work area.
class CreateFieldState(State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger):
        self.socketio = socketio
        self.logger = logger
        self.socketio.emit('field', {"status": "pushed"}, namespace='/button', broadcast=True)
        try:
            msg = f"[{self.__class__.__name__}] -> initSmoothie"
            self.logger.write_and_flush(msg+"\n")
            print(msg)
            self.smoothie = initSmoothie(self.logger)

            msg = f"[{self.__class__.__name__}] -> initGPS"
            self.logger.write_and_flush(msg+"\n")
            print(msg)
            self.gps = adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP)

            msg = f"[{self.__class__.__name__}] -> initGPSComputing"
            self.logger.write_and_flush(msg+"\n")
            print(msg)
            self.nav = navigation.GPSComputing()
        except:
            self.on_event(Events.ERROR)

        self.statusOfUIObject = {
            "joystick": True, #True or False
            "fieldButton": "charging", #True or False or charging or validate or None
            "startButton": False, #True or False or charging or None
            "continueButton": False, #True or False or charging or None
            "stopButton": True, #True or charging or None
            "wheelButton": False, #True or False
            "audit": 'disable', #True or False or use or not-use or disable
            "slider": 25, #Int for slider value
            "removeFieldButton": False #True or False  
        }

        self.fieldCreator = FieldCreator(self.logger, self.gps, self.nav, self.smoothie, self.socketio)

        self.field = None
        self.manoeuvre = False

        try:
            self.notificationQueue = posix_ipc.MessageQueue(config.QUEUE_NAME_UI_NOTIFICATION)
        except:
            self.notificationQueue = None
    
    def on_event(self, event):
        if event == Events.STOP:
            self.socketio.emit('stop', {"status": "pushed"}, namespace='/button', broadcast=True)
            self.statusOfUIObject["fieldButton"] = None
            self.statusOfUIObject["stopButton"] = "charging"

            try:
                self.fieldCreator.setSecondPoint()
            except TimeoutError:
                if self.notificationQueue is not None:
                    self.notificationQueue.send(json.dumps({"message_name": "No_GPS_for_field"}))     
                self.smoothie.disconnect()
                self.gps.disconnect() 
                return WaitWorkingState(self.socketio, self.logger, False) 

            self.field = self.fieldCreator.calculateField()
            if not config.TWO_POINTS_FOR_CREATE_FIELD and not config.FORWARD_BACKWARD_PATH:
                self.manoeuvre = True
                self.fieldCreator.manoeuvre()
                self.manoeuvre = False
            self.statusOfUIObject["stopButton"] = None
            self.statusOfUIObject["fieldButton"] = "validate"
            self.socketio.emit('field', {"status": "finish"}, namespace='/button', broadcast=True)
            self.smoothie.disconnect()
            self.gps.disconnect()
            return self
        elif event == Events.VALIDATE_FIELD:
            return self
        elif event == Events.VALIDATE_FIELD_NAME:
            self.socketio.emit('field', {"status": "validate"}, namespace='/button', broadcast=True)
            return WaitWorkingState(self.socketio, self.logger, True)
        elif event == Events.WHEEL:
            self.smoothie.freewheels()
            return self
        else:
            try:
                self.smoothie.disconnect()
                self.gps.disconnect()
            except:
                pass
            return ErrorState(self.socketio, self.logger)   

    def on_socket_data(self, data):
        if data["type"] == "joystick":
            if self.statusOfUIObject["fieldButton"] != "validate" and not self.manoeuvre:
                x = int(data["x"])/2
                if x < 0:
                    x *= -(config.A_MIN/100)
                if x > 0:
                    x *= config.A_MAX/100
                #print(f"[{self.__class__.__name__}] -> Move '{x}'.")
                self.smoothie.custom_move_to(A_F=config.A_F_UI, A=x)
        elif data["type"] == "field":
            msg = f"[{self.__class__.__name__}] -> Slider value : {data['value']}."
            self.logger.write_and_flush(msg+"\n")
            print(msg)
            self.statusOfUIObject["slider"] = int(data["value"])
            self.fieldCreator.setFieldSize(int(data["value"])*1000)

            try:
                self.fieldCreator.setFirstPoint()
                self.socketio.emit('field', {"status": "inRun"}, namespace='/button', broadcast=True)
                self.statusOfUIObject["fieldButton"] = None
            except TimeoutError:
                if self.notificationQueue is not None:
                    self.notificationQueue.send(json.dumps({"message_name": "No_GPS_for_field"}))  
                self.smoothie.disconnect()
                self.gps.disconnect()
                return WaitWorkingState(self.socketio, self.logger, False)

        elif data["type"] == "modifyZone":
            msg = f"[{self.__class__.__name__}] -> Slider value : {data['value']}."
            self.logger.write_and_flush(msg+"\n")
            print(msg)
            self.statusOfUIObject["slider"] = int(data["value"])
            self.fieldCreator.setFieldSize(int(data["value"])*1000)
            self.field = self.fieldCreator.calculateField()
        elif data["type"] == "validerZone":
            msg = f"[{self.__class__.__name__}] -> Slider value final : {data['value']}."
            self.logger.write_and_flush(msg+"\n")
            print(msg)
            self.statusOfUIObject["slider"] = int(data["value"])
            self.fieldCreator.setFieldSize(int(data["value"])*1000)
            self.field = self.fieldCreator.calculateField()
            self.socketio.emit('field', {"status": "validate_name"}, namespace='/button', room=data["client_id"])
        elif data["type"] == "field_name":
            self.statusOfUIObject["fieldButton"] = "charging"
            field_name = self.fieldCreator.saveField("../fields/",data["name"]+".txt")

            fields_list = load_field_list("../fields")

            if len(fields_list) > 0:
                os.system("ln -sf 'fields/"+quote(fields_list[0],safe="")+".txt' ../field.txt")
                coords, other_fields, current_field_name = updateFields(field_name)
            else:
                coords, other_fields, current_field_name = list(), list(), ""

            self.socketio.emit('newField', json.dumps({"field" : coords, "other_fields" : other_fields, "current_field_name" : current_field_name, "fields_list": fields_list}), namespace='/map')
        return self

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field


#This state corresponds when the robot configures it to start from zero the work.
class StartingState(State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger, isAudit = False):
        self.socketio = socketio
        self.logger = logger
        self.isAudit = isAudit
        self.socketio.emit('start', {"status": "pushed"}, namespace='/button', broadcast=True)
        msg = f"[{self.__class__.__name__}] -> Edit fichier config (CONTINUE_PREVIOUS_PATH:{False},AUDIT_MODE:{isAudit})"
        self.logger.write_and_flush(msg+"\n")
        print(msg) 
        changeConfigValue("CONTINUE_PREVIOUS_PATH", False)
        changeConfigValue("AUDIT_MODE", isAudit)

        self.statusOfUIObject = {
            "joystick": False, #True or False
            "fieldButton": False, #True or False or charging or None
            "startButton": "charging", #True or False or charging or None
            "continueButton": False, #True or False or charging or None
            "stopButton": None, #True or charging or None
            "wheelButton": False, #True or False
            "audit": isAudit, #True or False or use or not-use
            "slider": 25, #Int for slider value
            "removeFieldButton": False #True or False  
        }

        if isAudit:
            self.statusOfUIObject["audit"] = "use"
        else:
            self.statusOfUIObject["audit"] = "not-use"

        self.field = None
    
    def on_event(self, event):
        if event == Events.CONFIG_IS_SET:
            self.statusOfUIObject["startButton"] = None
            self.statusOfUIObject["stopButton"] = True
            return WorkingState(self.socketio, self.logger, self.isAudit, False)
        else:
            return ErrorState(self.socketio, self.logger)

    def on_socket_data(self, data):
        return ErrorState(self.socketio, self.logger)

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field


#This state corresponds when the robot configures it to continue the last job.
class ResumeState(State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger, isAudit = False):
        self.socketio = socketio
        self.logger = logger
        self.isAudit = isAudit
        self.socketio.emit('continue', {"status": "pushed"}, namespace='/button', broadcast=True)
        msg = f"[{self.__class__.__name__}] -> Edit fichier config (CONTINUE_PREVIOUS_PATH:{True},AUDIT_MODE:{isAudit})"
        self.logger.write_and_flush(msg+"\n")
        print(msg) 
        changeConfigValue("CONTINUE_PREVIOUS_PATH", True)
        changeConfigValue("AUDIT_MODE", isAudit)

        self.statusOfUIObject = {
            "joystick": False, #True or False
            "fieldButton": False, #True or False or charging or None
            "startButton": False, #True or False or charging or None
            "continueButton": "charging", #True or False or charging or None
            "stopButton": None, #True or charging or None
            "wheelButton": False, #True or False
            "audit": isAudit, #True or False or use or not-use
            "slider": 25, #Int for slider value
            "removeFieldButton": False #True or False  
        }

        if isAudit:
            self.statusOfUIObject["audit"] = "use"
        else:
            self.statusOfUIObject["audit"] = "not-use"

        self.field = None
    
    def on_event(self, event):
        if event == Events.CONFIG_IS_SET:
            self.statusOfUIObject["continueButton"] = None
            self.statusOfUIObject["stopButton"] = True
            return WorkingState(self.socketio, self.logger, self.isAudit, True)
        else:
            return ErrorState(self.socketio, self.logger)

    def on_socket_data(self, data):
        return ErrorState(self.socketio, self.logger)

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field
        

#This state corresponds when the robot is working.
class WorkingState(State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger, isAudit: bool, isResume: bool):
        self.socketio = socketio
        self.logger = logger
        self.isAudit = isAudit
        self.allPath = []
        self.isResume = isResume
        self.detected_plants = dict()
        self.extracted_plants = dict()

        msg = f"Audit mode enable : {isAudit}"
        self.logger.write_and_flush(msg+"\n")
        print(msg)

        msg = f"[{self.__class__.__name__}] -> Création queue de message ui main"
        self.logger.write_and_flush(msg+"\n")
        print(msg)

        try:
            posix_ipc.unlink_message_queue(config.QUEUE_NAME_UI_MAIN)
        except:
            pass

        self.msgQueue = posix_ipc.MessageQueue(config.QUEUE_NAME_UI_MAIN, posix_ipc.O_CREX)
        self._main_msg_thread_alive = True
        self._main_msg_thread = threading.Thread(target=self._main_msg_thread_tf, daemon=True)
        self._main_msg_thread.start()

        self.statusOfUIObject = {
            "joystick": False, #True or False
            "fieldButton": False, #True or False or charging or None
            "startButton": False, #True or False or charging or None
            "continueButton": False, #True or False or charging or None
            "stopButton": True, #True or charging or None
            "wheelButton": False , #True or False
            "audit": isAudit, #True or False or use or not-use
            "slider": 25, #Int for slider value
            "removeFieldButton": False #True or False  
        }

        if isAudit:
            self.statusOfUIObject["audit"] = "use"
        else:
            self.statusOfUIObject["audit"] = "not-use"

        if self.isResume:
            self.statusOfUIObject["continueButton"] = None
        else:
            self.statusOfUIObject["startButton"] = None

        self.field = None
        self.lastGpsQuality = "1"

        msg = f"[{self.__class__.__name__}] -> Lancement main"
        self.logger.write_and_flush(msg+"\n")
        print(msg)
        self.main = startMain()
        self.timeStartMain = datetime.now(timezone.utc)
        
    
    def on_event(self, event):
        if event == Events.STOP:
            self.socketio.emit('stop', {"status": "pushed"}, namespace='/button', broadcast=True)
            self.statusOfUIObject["stopButton"] = "charging"
            self.main.send_signal(signal.SIGINT)
            self.main.wait()
            os.system("sudo systemctl restart nvargus-daemon")
            self._main_msg_thread_alive = False
            self.socketio.emit('stop', {"status": "finish"}, namespace='/button', broadcast=True)
            if self.isResume:
                self.statusOfUIObject["continueButton"] = True
            else:
                self.statusOfUIObject["startButton"] = True
            self.statusOfUIObject["stopButton"] = None
            return WaitWorkingState(self.socketio, self.logger, False)
        else:
            self._main_msg_thread_alive = False
            return ErrorState(self.socketio, self.logger)
    
    def on_socket_data(self, data):
        if data["type"] == "getStats":
            self.sendLastStatistics()
            return self
        else:
            self._main_msg_thread_alive = False
            return ErrorState(self.socketio, self.logger)

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field

    def sendLastStatistics(self):
        data = dict()
        data["weeds"] = self.extracted_plants
        data["time"] = self.timeStartMain.isoformat()
        self.socketio.emit('statistics', data, namespace='/server', broadcast=True)

    def _main_msg_thread_tf(self): 
        while self._main_msg_thread_alive:
            msg = self.msgQueue.receive()
            data = json.loads(msg[0])
            if "start" in data:
                if data["start"]:
                    msg = f"[{self.__class__.__name__}] -> Main lancé !"
                    self.logger.write_and_flush(msg+"\n")
                    print(msg)
                    self.socketio.emit('startMain', {"status": "finish", "audit": self.isAudit}, namespace='/button', broadcast=True)
            elif "datacollector" in data:
                self.detected_plants = data["datacollector"][0]
                self.extracted_plants = data["datacollector"][1]
                self.sendLastStatistics()
            elif "last_gps" in data:
                data = json.loads(msg[0])["last_gps"]
                self.allPath.append([data[1],data[0]])
                if self.lastGpsQuality != data[2]:
                    self.lastGpsQuality = data[2]
                self.socketio.emit('updatePath', json.dumps(self.allPath), namespace='/map')

#This state corresponds when the robot has an error.
class ErrorState(State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger):
        self.socketio = socketio
        self.logger = logger
        msg = f"[{self.__class__.__name__}] -> Error"
        self.logger.write_and_flush(msg+"\n")
        print(msg)

        self.statusOfUIObject = {
            "joystick": False, #True or False
            "fieldButton": False, #True or False or charging or None
            "startButton": False, #True or False or charging or None
            "continueButton": False, #True or False or charging or None
            "stopButton": None, #True or charging or None
            "wheelButton": False, #True or False
            "audit": False, #True or False or use or not-use
            "slider": 25, #Int for slider value
            "removeFieldButton": False #True or False  
        }

        self.field = None

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field

###### Class for all state/ functions ######

class FieldCreator:

    def __init__(self, logger: utility.Logger, gps: adapters.GPSUbloxAdapter, nav: navigation.GPSComputing, smoothie: adapters.SmoothieAdapter, socketio: SocketIO):
        self.A = [0,0]
        self.B = [0,0]
        self.C = [0,0]
        self.D = [0,0]
        self.length_field = 0
        self.field = []
        self.logger = logger
        self.gps = gps
        self.nav = nav
        self.smoothie = smoothie
        self.socketio = socketio

    def setFirstPoint(self):
        msg = f"[{self.__class__.__name__}] -> Getting point A..."
        self.logger.write_and_flush(msg+"\n")
        print(msg)
        self.A = utility.average_point(self.gps,None,self.nav,self.logger)

        self.socketio.emit('newPos', json.dumps([self.A[1],self.A[0]]), namespace='/map')

        self.vesc_emergency = initVesc(self.logger)
        msg = f"[{self.__class__.__name__}] -> Moving forward..."
        self.logger.write_and_flush(msg+"\n")
        print(msg)
        self.vesc_emergency.apply_rpm(config.VESC_RPM_UI)
        self.vesc_emergency.start_moving()

    def setSecondPoint(self):
        msg = f"[{self.__class__.__name__}] -> Stop moving forward..."
        self.logger.write_and_flush(msg+"\n")
        print(msg)

        self.vesc_emergency.stop_moving()

        msg = f"[{self.__class__.__name__}] -> Getting point B..."
        self.logger.write_and_flush(msg+"\n")
        print(msg)
        self.B = utility.average_point(self.gps,None,self.nav,self.logger)

        self.socketio.emit('newPos', json.dumps([self.B[1],self.B[0]]), namespace='/map')
    
    def setFieldSize(self, size: int):
        self.length_field = size
    
    def calculateField(self):
        width_field = self.nav.get_distance(self.A,self.B)

        msg = f"[{self.__class__.__name__}] -> Field_size : {round(self.length_field/1000,2)} (length) / {round(width_field/1000,2)} (width)."
        self.logger.write_and_flush(msg+"\n")
        print(msg)

        if not config.TWO_POINTS_FOR_CREATE_FIELD:
            self.C = self.nav.get_coordinate(self.B, self.A, -90, self.length_field)
            self.D = self.nav.get_coordinate(self.C, self.B, -90, width_field)
            self.field = [self.B, self.C, self.D, self.A]
        else:
            self.field = [self.B, self.A]
     
        other_fields = get_other_field()
        current_field_name = subprocess.run(["readlink","../field.txt"], stdout=subprocess.PIPE).stdout.decode('utf-8').replace("fields/", "")[:-5]

        self.socketio.emit('newField', json.dumps({"field" : self.formattingFieldPointsForSend(), "other_fields" : other_fields, "current_field_name" : unquote(current_field_name)}), namespace='/map')

        return self.field
    
    def formattingFieldPointsForSend(self):
        coords = list()

        for coord in self.field:
            coords.append([coord[1],coord[0]])

        if len(self.field) == 4:
            coords.append(coords[0])

        return coords

    def saveField(self, fieldPath: str , fieldName: str):
        cpt = 1
        fieldName = quote(fieldName,safe="")
        if(os.path.exists(fieldPath+fieldName)):
            while os.path.exists(f"{fieldPath+fieldName[:-4]}_{cpt}.txt"):
                cpt+=1
            fieldName = f"{fieldName[:-4]}_{cpt}.txt"
        path = fieldPath + fieldName
        msg = f"[{self.__class__.__name__}] -> Save field in {path}..."
        self.logger.write_and_flush(msg+"\n")
        print(msg)
        save_gps_coordinates(self.field, path)
        return unquote(fieldName[:-4])

    def manoeuvre(self):
        self.vesc_emergency.apply_rpm(-config.VESC_RPM_UI)
        self.vesc_emergency.start_moving()
        time.sleep(6)
        self.vesc_emergency.stop_moving()

        self.smoothie.custom_move_to(A_F=config.A_F_UI, A=config.A_MIN)
        self.smoothie.wait_for_all_actions_done()

        self.vesc_emergency.apply_rpm(config.VESC_RPM_UI)
        self.vesc_emergency.start_moving()
        time.sleep(8)
        self.vesc_emergency.stop_moving()

        self.smoothie.custom_move_to(A_F=config.A_F_UI, A=0)
        self.smoothie.wait_for_all_actions_done()


###### Function for all state ######


def initVesc(logger: utility.Logger):
    smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
    if "vesc" in smoothie_vesc_addr:
        vesc_address = smoothie_vesc_addr["vesc"]
    else:
        msg = "Couldn't get vesc's USB address!"
        logger.write_and_flush(msg+"\n")
        print(msg)
        exit(1)
    vesc_engine = adapters.VescAdapter(0, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ, config.VESC_CHECK_FREQ, vesc_address, config.VESC_BAUDRATE)
    return vesc_engine

def initSmoothie(logger: utility.Logger):
    smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
    if config.SMOOTHIE_BACKEND == 1:
        smoothie_address = config.SMOOTHIE_HOST
    else:
        if "smoothie" in smoothie_vesc_addr:
            smoothie_address = smoothie_vesc_addr["smoothie"]
        else:
            msg = "Couldn't get smoothie's USB address!"
            logger.write_and_flush(msg+"\n")
            print(msg)
            exit(1)
    smoothie = adapters.SmoothieAdapter(smoothie_address)
    return smoothie

def save_gps_coordinates(points: list, file_name):
    with open(file_name, "w") as file:
        for point in points:
            str_point = str(point[0]) + " " + str(point[1]) + "\n"
            file.write(str_point)
    user=pwd.getpwnam('violette')
    os.chown(file_name, user.pw_uid, user.pw_gid)
        

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

         
def startMain():
    #mainSP = subprocess.Popen(["python3","main.py"], cwd="/home/violette/field")#, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
    mainSP = subprocess.Popen(["python3","main.py"], cwd=os.getcwd().split("/uiWebRobot")[0])
    return mainSP

def startLiveCam():
    #camSP = subprocess.Popen(["python3","serveurCamLive.py"], cwd="/home/violette/field")
    camSP = subprocess.Popen(["python3","serveurCamLive.py"], cwd=os.getcwd().split("/uiWebRobot")[0])
    return camSP

def checkHaveGPS(socketio: SocketIO, statusOfUIObject: dict):
    GPSSerial = serial.Serial(port=config.GPS_PORT, baudrate=config.GPS_BAUDRATE)

    Matrame = "B5 62 06 00 14 00 03 00 00 00 00 00 00 00 00 00 00 00 23 00 03 00 00 00 00 00 43 AE"
    GPSSerial.write(bytearray.fromhex(Matrame))

    while True:
        try:
            data = str(GPSSerial.readline())
        except:
            continue
        if "GNGGA" in data and ",,," not in data:
            data = data.split(",")
            if data[6] != 0:
                print("GPS work")
                statusOfUIObject["GPSWork"] = True
                socketio.emit('checklist', {"status","GPSWork"}, namespace='/server')
                GPSSerial.close()
                break

def updateFields(field_name):

    field_name = quote(field_name,safe="")

    cmd = "ln -sf 'fields/"+field_name+".txt' ../field.txt"

    os.system(cmd)
            
    with open("../field.txt") as file:
        points = file.readlines()
    
    coords = list()
    for coord in points:
        coord = coord.replace("\n","").split(" ")
        coords.append([float(coord[1]),float(coord[0])])
    coords.append(coords[0])

    other_fields = get_other_field()
    current_field_name = subprocess.run(["readlink","../field.txt"], stdout=subprocess.PIPE).stdout.decode('utf-8').replace("fields/", "")[:-5]

    return coords, other_fields, unquote(current_field_name)