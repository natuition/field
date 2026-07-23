import time

from flask_socketio import SocketIO
import posix_ipc
from urllib.parse import quote, unquote
import logging
import sys
import os
import json

from config import config
from uiWebRobot.state_machine import State, utilsFunction
from uiWebRobot.state_machine.states import WaitWorkingState, ErrorState
from uiWebRobot.state_machine.Events import Events
from uiWebRobot.state_machine.FrontEndObjects import FrontEndObjects, ButtonState
from shared_class.robot_synthesis import RobotSynthesis
import utility
import adapters
from logger import LoggerFactory


class CreateFieldStateWithNavX(State.State):
    """This state corresponds when the robot process a geojson file from NavX with Linestring for create the field. """

    def __init__(self,
                 socketio: SocketIO,
                 file_logger: utility.Logger,
                 smoothie: adapters.SmoothieAdapter,
                 vesc_engine: adapters.VescAdapterV4):
        self.__logger = LoggerFactory.create(self.__class__.__name__)
        self.robot_synthesis_value = RobotSynthesis.UI_CREATE_FIELD_STATE
        self.socketio = socketio
        self.__file_logger = file_logger
        self.smoothie = smoothie
        self.vesc_engine = vesc_engine
        
        self.socketio.emit('field', {"status": "pushed"}, namespace='/button', broadcast=True)
        self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.CHARGING,
                                                startButton=ButtonState.DISABLE,
                                                continueButton=ButtonState.DISABLE,
                                                stopButton=ButtonState.ENABLE,
                                                wheelButton=ButtonState.NOT_HERE,
                                                removeFieldButton=ButtonState.DISABLE,
                                                joystick=True,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE)
        self.field = None
        self.__ui_languages, self.__current_ui_language = utilsFunction.get_ui_language()


    def on_event(self, event):
        if event == Events.WHEEL:
            self.smoothie.freewheels()
            return self
        else:
            try:
                if self.smoothie is not None:
                    self.smoothie.disconnect()
                    self.smoothie = None
                if self.vesc_engine is not None:
                    self.vesc_engine.close()
                    self.vesc_engine = None
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except Exception as e:
                self.__file_logger.write_and_flush(e + "\n")
            return ErrorState.ErrorState(self.socketio, self.__file_logger)
        
    def saveField(self, fieldPath: str, fieldName: str):
        cpt = 1
        fieldName = quote(fieldName, safe="", encoding='utf-8')
        if (os.path.exists(fieldPath + fieldName)):
            while os.path.exists(f"{fieldPath + fieldName[:-4]}_{cpt}.txt"):
                cpt += 1
            fieldName = f"{fieldName[:-4]}_{cpt}.txt"
        path = fieldPath + fieldName
        msg = f"Save field in {path}..."
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.info(msg)
        utilsFunction.save_gps_coordinates(self.field, path)
        return (path, unquote(fieldName[:-4], encoding='utf-8'))

    def on_socket_data(self, data):
        if data["type"] == "create_field":
            msg = f"File value : {data['value']}."
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.debug(msg)    
            
            self.__logger.info("Calculating the largest inscribed rectangle...")
            
            result = utilsFunction.largest_inscribed_rectangle(
                data['value'],
                max_iterations=500,
                population_size=20
            )
            self.__logger.info("Largest inscribed rectangle calculated.")
            self.__logger.info(f"\t- Surface : {result.area:.3f} m²")
            self.__logger.info(f"\t- Dimensions : {result.width:.3f} × {result.height:.3f} m")
            self.__logger.info(f"\t- CRS métrique : {result.metric_crs}")
            
            self.field = result.corners
            field_name = "Example field"
            
            field_path, field_name = self.saveField("./fields/", field_name + ".txt")
            
            if utilsFunction.is_valid_field_file(field_path, self.__file_logger):
                fields_list = utilsFunction.load_field_list("./fields")

                if len(fields_list) > 0:
                    coords, other_fields, current_field_name = utilsFunction.updateFields(field_name)
                else:
                    coords, other_fields, current_field_name = list(), list(), ""

                self.socketio.emit('newField', json.dumps(
                    {"field": coords, "other_fields": other_fields, "current_field_name": current_field_name,
                    "fields_list": fields_list}), namespace='/map')
                
                self.socketio.emit('field', {"status": "validate_geojson"}, namespace='/button', broadcast=True)
                return WaitWorkingState.WaitWorkingState(self.socketio, self.__file_logger, True, self.smoothie, self.vesc_engine)
            else:
                if os.path.exists(field_path):
                    os.remove(field_path)
                message = self.__ui_languages["working_zone_too_small"][self.__current_ui_language]
                self.socketio.emit('notification', {"message_name": "not_a_good_zone", "message": message}, namespace='/broadcast', broadcast=True)
            

        return self

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field
