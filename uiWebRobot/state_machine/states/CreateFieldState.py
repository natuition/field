import sys
sys.path.append('../')

from flask_socketio import SocketIO
import navigation
import posix_ipc
import os
import json
from urllib.parse import quote, unquote

from state_machine import State
from state_machine.states import WaitWorkingState
from state_machine.states import ErrorState
from state_machine import Events
from state_machine.FrontEndObjects import FrontEndObjects, ButtonState
from state_machine import utilsFunction
from shared_class.robot_synthesis import RobotSynthesis

from config import config
import time
import utility
import adapters
from navigation import NavigationV3


# This state corresponds when the robot is generating the work area.
class CreateFieldState(State.State):

    def __init__(self,
                 socketio: SocketIO,
                 logger: utility.Logger,
                 smoothie: adapters.SmoothieAdapter,
                 vesc_engine: adapters.VescAdapterV4):
        self.robot_synthesis_value = RobotSynthesis.UI_CREATE_FIELD_STATE
        self.socketio = socketio
        self.logger = logger
        self.smoothie = smoothie
        self.vesc_engine = vesc_engine

        self.socketio.emit('field', {"status": "pushed"}, namespace='/button', broadcast=True)
        try:
            if self.smoothie is None:
                msg = f"[{self.__class__.__name__}] -> initSmoothie"
                self.logger.write_and_flush(msg + "\n")
                print(msg)
                self.smoothie = utilsFunction.initSmoothie(self.logger)

            msg = f"[{self.__class__.__name__}] -> initGPSComputing"
            self.logger.write_and_flush(msg + "\n")
            print(msg)
            self.nav = navigation.GPSComputing()
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            raise e

        self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.CHARGING,
                                                startButton=ButtonState.DISABLE,
                                                continueButton=ButtonState.DISABLE,
                                                stopButton=ButtonState.ENABLE,
                                                wheelButton=ButtonState.NOT_HERE,
                                                removeFieldButton=ButtonState.DISABLE,
                                                joystick=True,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE)

        self.fieldCreator = FieldCreator(self.logger, self.nav, self.vesc_engine, self.smoothie,
                                         self.socketio)

        self.field = None
        self.manoeuvre = False

        try:
            self.notificationQueue = posix_ipc.MessageQueue(config.QUEUE_NAME_UI_NOTIFICATION)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except:
            self.notificationQueue = None

        #self.__send_last_pos_thread_alive = True
        #self._send_last_pos_thread = threading.Thread(target=send_last_pos_thread_tf, args=(lambda : self.send_last_pos_thread_alive, self.socketio, self.logger), daemon=True)

    def on_event(self, event):
        if event == Events.Events.STOP:
            self.socketio.emit('stop', {"status": "pushed"}, namespace='/button', broadcast=True)
            self.statusOfUIObject.fieldButton = ButtonState.NOT_HERE
            self.statusOfUIObject.stopButton = ButtonState.CHARGING

            #self.__send_last_pos_thread_alive = False
            #self._send_last_pos_thread.join()

            try:
                self.fieldCreator.setSecondPoint()
            except TimeoutError:
                if self.notificationQueue is not None:
                    self.notificationQueue.send(json.dumps({"message_name": "No_GPS_for_field"}))
                return WaitWorkingState.WaitWorkingState(self.socketio, self.logger, False, self.smoothie, self.vesc_engine)

            self.field = self.fieldCreator.calculateField()
            if not config.TWO_POINTS_FOR_CREATE_FIELD and not config.FORWARD_BACKWARD_PATH:
                self.manoeuvre = True
                if config.MAKE_MANEUVER_AFTER_FIELD_CREATE:
                    self.fieldCreator.manoeuvre()
                self.manoeuvre = False

            #self.__send_last_pos_thread_alive = True
            #self._send_last_pos_thread = threading.Thread(target=send_last_pos_thread_tf, args=(lambda : self.send_last_pos_thread_alive, self.socketio, self.logger), daemon=True)
            #self._send_last_pos_thread.start()

            self.statusOfUIObject.stopButton = ButtonState.NOT_HERE
            self.statusOfUIObject.fieldButton = ButtonState.VALIDATE
            self.socketio.emit('field', {"status": "finish"}, namespace='/button', broadcast=True)
            return self
        elif event == Events.Events.VALIDATE_FIELD:
            return self
        elif event == Events.Events.VALIDATE_FIELD_NAME:
            self.socketio.emit('field', {"status": "validate"}, namespace='/button', broadcast=True)
            return WaitWorkingState.WaitWorkingState(self.socketio, self.logger, True, self.smoothie, self.vesc_engine)
        elif event == Events.Events.WHEEL:
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
                self.logger.write_and_flush(e + "\n")
            return ErrorState.ErrorState(self.socketio, self.logger)

    def on_socket_data(self, data):
        if data["type"] == "joystick":
            if self.statusOfUIObject.fieldButton != ButtonState.VALIDATE and not self.manoeuvre:
                x = int(data["x"]) / 2
                if x < 0:
                    x *= -(config.A_MIN / 100)
                if x > 0:
                    x *= config.A_MAX / 100
                # print(f"[{self.__class__.__name__}] -> Move '{x}'.")
                self.smoothie.custom_move_to(A_F=config.A_F_UI, A=x)
        elif data["type"] == "create_field":
            msg = f"[{self.__class__.__name__}] -> Slider value : {data['value']}."
            self.logger.write_and_flush(msg + "\n")
            print(msg)
            self.statusOfUIObject.slider = float(data["value"])
            self.fieldCreator.setFieldSize(float(data["value"]) * 1000)

            try:
                self.fieldCreator.setFirstPoint()

                #self._send_last_pos_thread.start()

                self.socketio.emit('field', {"status": "inRun"}, namespace='/button', broadcast=True)
                self.statusOfUIObject.fieldButton = ButtonState.NOT_HERE
            except TimeoutError:
                if self.notificationQueue is not None:
                    self.notificationQueue.send(json.dumps({"message_name": "No_GPS_for_field"}))
                #self.__send_last_pos_thread_alive = False
                #self._send_last_pos_thread.join()
                return WaitWorkingState.WaitWorkingState(self.socketio, self.logger, False, self.smoothie, self.vesc_engine)

        elif data["type"] == "modifyZone":
            msg = f"[{self.__class__.__name__}] -> Slider value : {data['value']}."
            self.logger.write_and_flush(msg + "\n")
            print(msg)
            self.statusOfUIObject.slider = float(data["value"])
            self.fieldCreator.setFieldSize(float(data["value"]) * 1000)
            self.field = self.fieldCreator.calculateField()
        elif data["type"] == "validerZone":
            msg = f"[{self.__class__.__name__}] -> Slider value final : {data['value']}."
            self.logger.write_and_flush(msg + "\n")
            print(msg)
            self.statusOfUIObject.slider = float(data["value"])
            self.fieldCreator.setFieldSize(float(data["value"]) * 1000)
            self.field = self.fieldCreator.calculateField()
            self.socketio.emit('field', {"status": "validate_name"}, namespace='/button', room=data["client_id"])
        elif data["type"] == "validate_field_name":
            self.statusOfUIObject.fieldButton = ButtonState.CHARGING
            #patch bug field
            utilsFunction.save_gps_coordinates(self.field, "../fields/tmp.txt")
            field_name = self.fieldCreator.saveField("../fields/", data["name"] + ".txt")

            fields_list = utilsFunction.load_field_list("../fields")

            if len(fields_list) > 0:
                coords, other_fields, current_field_name = utilsFunction.updateFields(field_name)
            else:
                coords, other_fields, current_field_name = list(), list(), ""

            self.socketio.emit('newField', json.dumps(
                {"field": coords, "other_fields": other_fields, "current_field_name": current_field_name,
                 "fields_list": fields_list}), namespace='/map')

        return self

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field


class FieldCreator:

    def __init__(self,
                 logger: utility.Logger,
                 nav: navigation.GPSComputing,
                 vesc_engine: adapters.VescAdapterV4,
                 smoothie: adapters.SmoothieAdapter,
                 socketio: SocketIO):
        self.A = [0, 0]
        self.B = [0, 0]
        self.C = [0, 0]
        self.D = [0, 0]
        self.length_field = 0
        self.field = []
        self.logger = logger
        self.nav = nav
        self.vesc_emergency: adapters.VescAdapterV4 = vesc_engine
        self.smoothie = smoothie
        self.socketio = socketio

    def setFirstPoint(self):
        msg = f"[{self.__class__.__name__}] -> Getting point A..."
        self.logger.write_and_flush(msg + "\n")
        print(msg)
        with adapters.GPSUbloxAdapterWithoutThread(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP) as gps:
            self.A = utility.average_point(gps, None, self.nav, self.logger)

        self.socketio.emit('newPos', json.dumps([self.A[1], self.A[0]]), namespace='/map')

        if self.vesc_emergency is None:
            self.vesc_emergency: adapters.VescAdapterV4 = utilsFunction.initVesc(self.logger)
        msg = f"[{self.__class__.__name__}] -> Moving forward..."
        self.logger.write_and_flush(msg + "\n")
        print(msg)
        self.vesc_emergency.set_target_rpm(
            config.SI_SPEED_UI * config.MULTIPLIER_SI_SPEED_TO_RPM,
            self.vesc_emergency.PROPULSION_KEY)
        self.vesc_emergency.start_moving(self.vesc_emergency.PROPULSION_KEY)

    def setSecondPoint(self):
        msg = f"[{self.__class__.__name__}] -> Stop moving forward..."
        self.logger.write_and_flush(msg + "\n")
        print(msg)

        self.vesc_emergency.stop_moving(self.vesc_emergency.PROPULSION_KEY)

        msg = f"[{self.__class__.__name__}] -> Getting point B..."
        self.logger.write_and_flush(msg + "\n")
        print(msg)
        with adapters.GPSUbloxAdapterWithoutThread(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP) as gps:
            self.B = utility.average_point(gps, None, self.nav, self.logger)

        self.socketio.emit('newPos', json.dumps([self.B[1], self.B[0]]), namespace='/map')

    def setFieldSize(self, size: int):
        self.length_field = size

    def calculateField(self):
        width_field = self.nav.get_distance(self.A, self.B)

        msg = f"[{self.__class__.__name__}] -> Field_size : {round(self.length_field / 1000, 2)} (length) / {round(width_field / 1000, 2)} (width)."
        self.logger.write_and_flush(msg + "\n")
        print(msg)

        if not config.TWO_POINTS_FOR_CREATE_FIELD:
            self.C = self.nav.get_coordinate(self.B, self.A, 90, self.length_field)
            self.D = self.nav.get_coordinate(self.C, self.B, 90, width_field)
            self.field = [self.B, self.C, self.D, self.A]
        else:
            self.field = [self.B, self.A]

        other_fields = utilsFunction.get_other_field()

        link_path = os.path.realpath("../field.txt")
        current_field_name = (link_path.split("/")[-1]).split(".")[0]

        self.socketio.emit('newField', json.dumps(
            {"field": self.formattingFieldPointsForSend(), "other_fields": other_fields,
             "current_field_name": unquote(current_field_name, encoding='utf-8')}), namespace='/map')

        return self.field

    def formattingFieldPointsForSend(self):
        coords = list()

        for coord in self.field:
            coords.append([coord[1], coord[0]])

        if len(self.field) == 4:
            coords.append(coords[0])

        return coords

    def saveField(self, fieldPath: str, fieldName: str):
        cpt = 1
        fieldName = quote(fieldName, safe="", encoding='utf-8')
        if (os.path.exists(fieldPath + fieldName)):
            while os.path.exists(f"{fieldPath + fieldName[:-4]}_{cpt}.txt"):
                cpt += 1
            fieldName = f"{fieldName[:-4]}_{cpt}.txt"
        path = fieldPath + fieldName
        msg = f"[{self.__class__.__name__}] -> Save field in {path}..."
        self.logger.write_and_flush(msg + "\n")
        print(msg)
        utilsFunction.save_gps_coordinates(self.field, path)
        return unquote(fieldName[:-4], encoding='utf-8')

    def manoeuvre(self):
        # move backward and stop
        if config.UI_VERBOSE_LOGGING:
            msg = f"Field creation: starting vesc movement of " \
                  f"RPM={-config.SI_SPEED_UI * config.MULTIPLIER_SI_SPEED_TO_RPM}"
            print(msg)
            self.logger.write_and_flush(msg + "\n")
        self.vesc_emergency.set_target_rpm(
            -config.SI_SPEED_UI * config.MULTIPLIER_SI_SPEED_TO_RPM,
            self.vesc_emergency.PROPULSION_KEY)
        self.vesc_emergency.set_time_to_move(config.MANEUVER_TIME_BACKWARD, self.vesc_emergency.PROPULSION_KEY)
        self.vesc_emergency.start_moving(self.vesc_emergency.PROPULSION_KEY)
        self.vesc_emergency.wait_for_stop(self.vesc_emergency.PROPULSION_KEY)
        if config.UI_VERBOSE_LOGGING:
            msg = f"Field creation: stopped vesc movement of " \
                  f"RPM={-config.SI_SPEED_UI * config.MULTIPLIER_SI_SPEED_TO_RPM}"
            print(msg)
            self.logger.write_and_flush(msg + "\n")

        # vesc can't stop robot instantly, so we wait for 1 sec before turn wheels right
        time.sleep(1)

        # turn wheels to right
        if config.UI_VERBOSE_LOGGING:
            msg = f"Field creation: starting turning smoothie wheels to A={config.A_MIN}"
            print(msg)
            self.logger.write_and_flush(msg + "\n")
        self.smoothie.custom_move_to(A_F=config.A_F_UI, A=config.A_MIN)
        self.smoothie.wait_for_all_actions_done()
        if config.UI_VERBOSE_LOGGING:
            msg = f"Field creation: stopped turning smoothie wheels to A={config.A_MIN}"
            print(msg)
            self.logger.write_and_flush(msg + "\n")

        # move forward (wheels are turned to right) and stop
        if config.UI_VERBOSE_LOGGING:
            msg = f"Field creation: starting vesc movement of " \
                  f"RPM={config.SI_SPEED_UI * config.MULTIPLIER_SI_SPEED_TO_RPM}"
            print(msg)
            self.logger.write_and_flush(msg + "\n")
        self.vesc_emergency.set_target_rpm(
            config.SI_SPEED_UI * config.MULTIPLIER_SI_SPEED_TO_RPM,
            self.vesc_emergency.PROPULSION_KEY)
        self.vesc_emergency.set_time_to_move(config.MANEUVER_TIME_FORWARD, self.vesc_emergency.PROPULSION_KEY)
        self.vesc_emergency.start_moving(self.vesc_emergency.PROPULSION_KEY)
        self.vesc_emergency.wait_for_stop(self.vesc_emergency.PROPULSION_KEY)
        self.vesc_emergency.set_target_rpm(0, self.vesc_emergency.PROPULSION_KEY)
        if config.UI_VERBOSE_LOGGING:
            msg = f"Field creation: stopped vesc movement of " \
                  f"RPM={config.SI_SPEED_UI * config.MULTIPLIER_SI_SPEED_TO_RPM}"
            print(msg)
            self.logger.write_and_flush(msg + "\n")

        # align wheels to center
        if config.UI_VERBOSE_LOGGING:
            msg = "Field creation: starting turning smoothie wheels to A=0"
            print(msg)
            self.logger.write_and_flush(msg + "\n")
        self.smoothie.custom_move_to(A_F=config.A_F_UI, A=0)
        self.smoothie.wait_for_all_actions_done()
        if config.UI_VERBOSE_LOGGING:
            msg = "Field creation: stopped turning smoothie wheels to A=0"
            print(msg)
            self.logger.write_and_flush(msg + "\n")

        self.vesc_emergency.set_time_to_move(config.VESC_MOVING_TIME, self.vesc_emergency.PROPULSION_KEY)
