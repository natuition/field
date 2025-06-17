import sys
sys.path.append('../')
import time
from flask_socketio import SocketIO
import threading
import os
import json
from urllib.parse import quote

from state_machine import State
from state_machine.states import CreateFieldState
from state_machine.states import StartingState
from state_machine.states import ResumeState
from state_machine.states import ErrorState
from state_machine.states import CalibrateState
from state_machine.states import ActuatorScreeningState
from state_machine import Events
from shared_class.robot_synthesis import RobotSynthesis

from state_machine.FrontEndObjects import FrontEndObjects, ButtonState, AuditButtonState
from state_machine import utilsFunction
from config import config
import adapters
import utility

from EnvironnementConfig import EnvironnementConfig


# This state corresponds when the robot is waiting to work, during this state we can control it with the joystick.

class WaitWorkingState(State.State):

    def __init__(self,
                 socketio: SocketIO,
                 logger: utility.Logger,
                 createField: bool,
                 smoothie: adapters.SmoothieAdapter = None,
                 vesc_engine: adapters.VescAdapterV4 = None):
        self.robot_synthesis_value = RobotSynthesis.UI_WAIT_WORKING_STATE
        self.socketio = socketio
        self.logger = logger
        self.smoothie = smoothie
        self.vesc_engine = vesc_engine

        if config.UI_VERBOSE_LOGGING:
            msg = f"[{self.__class__.__name__}] -> Self initialization"
            self.logger.write_and_flush(msg + "\n")
            print(msg)

        try:
            if self.vesc_engine is None:
                if config.UI_VERBOSE_LOGGING:
                    msg = f"[{self.__class__.__name__}] -> initVesc"
                    self.logger.write_and_flush(msg + "\n")
                    print(msg)
                self.vesc_engine = utilsFunction.initVesc(self.logger)
            elif config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> no need to initVesc"
                self.logger.write_and_flush(msg + "\n")
                print(msg)

            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> Vesc engine : set target rpm"
                self.logger.write_and_flush(msg + "\n")

            self.vesc_engine.set_target_rpm(0, self.vesc_engine.PROPULSION_KEY)

            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> Vesc engine : set current rpm"
                self.logger.write_and_flush(msg + "\n")

            self.vesc_engine.set_current_rpm(0, self.vesc_engine.PROPULSION_KEY)

            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> Vesc engine : set start moving"
                self.logger.write_and_flush(msg + "\n")

            self.vesc_engine.start_moving(
                self.vesc_engine.PROPULSION_KEY,
                smooth_acceleration=True,
                smooth_deceleration=True)

            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> Vesc engine : started"
                self.logger.write_and_flush(msg + "\n")

            if self.smoothie is None:
                if config.UI_VERBOSE_LOGGING:
                    msg = f"[{self.__class__.__name__}] -> initSmoothie"
                    self.logger.write_and_flush(msg + "\n")
                    print(msg)
                try:
                    self.smoothie = utilsFunction.initSmoothie(self.logger)
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except Exception as e:
                    if "[Timeout sm]" in str(e):
                        self.smoothie = utilsFunction.initSmoothie(self.logger)
                    else:
                        raise e
            elif config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> no need to initSmoothie"
                self.logger.write_and_flush(msg + "\n")
                print(msg)

        except KeyboardInterrupt:
            raise KeyboardInterrupt

        self.lastValueX = 0
        self.lastValueY = 0

        if config.UI_VERBOSE_LOGGING:
            msg = f"[{self.__class__.__name__}] -> Setting FrontEndObjects..."
            self.logger.write_and_flush(msg + "\n")

        self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.ENABLE,
                                                startButton=ButtonState.ENABLE,
                                                continueButton=ButtonState.ENABLE,
                                                stopButton=ButtonState.NOT_HERE,
                                                wheelButton=ButtonState.DISABLE,
                                                removeFieldButton=ButtonState.ENABLE,
                                                joystick=True,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE,
                                                audit=AuditButtonState.EXTRACTION_ENABLE)

        if createField:
            self.statusOfUIObject.continueButton = ButtonState.DISABLE

        self.learn_go_straight_angle = 0

        if config.LEARN_GO_STRAIGHT_UI:
            if os.path.isfile(f"../{config.LEARN_GO_STRAIGHT_FILE}"):
                with open(f"../{config.LEARN_GO_STRAIGHT_FILE}", "r") as learn_go_straight_file:
                    self.learn_go_straight_angle = float(
                        learn_go_straight_file.read())
                    self.logger.write_and_flush(
                        f"LEARN_GO_STRAIGHT:{self.learn_go_straight_angle}\n")
                    self.smoothie.custom_move_to(
                        A_F=config.A_F_UI, A=self.learn_go_straight_angle)

        self.socketio.emit(
            'checklist', {"status": "refresh"}, namespace='/server', broadcast=True)

        self.field = None

        self.send_last_pos_thread_alive = True
        self._send_last_pos_thread = threading.Thread(target=utilsFunction.send_last_pos_thread_tf, args=(
            lambda: self.send_last_pos_thread_alive, self.socketio, self.logger), daemon=True)
        self._send_last_pos_thread.start()

        self.__voltage_thread_alive = True
        self.input_voltage = {"input_voltage": "?"}
        self.__voltage_thread = threading.Thread(target=utilsFunction.voltage_thread_tf,
                                                 args=(lambda: self.__voltage_thread_alive,
                                                       self.vesc_engine,
                                                       self.socketio,
                                                       self.input_voltage,
                                                       self.logger,
                                                       self.recreate_vesc),
                                                 daemon=True)
        self.__voltage_thread.start()

        self.__last_joystick_info = time.time()
        self.__check_joystick_info_alive = True
        self.__joystick_info_thread = threading.Thread(target=self.__check_joystick_info_tf,
                                                       daemon=True)
        self.__joystick_info_thread.start()

        self.can_go_setting = True

        EnvironnementConfig.NATUITION_CHECKLIST(False)

        if config.UI_VERBOSE_LOGGING:
            msg = f"[{self.__class__.__name__}] -> Self initialization DONE"
            self.logger.write_and_flush(msg + "\n")
            print(msg)

    def __check_joystick_info_tf(self):
        while self.__check_joystick_info_alive:
            if time.time() - self.__last_joystick_info > config.TIMEOUT_JOYSTICK_USER_ACTION:
                if self.vesc_engine is not None:
                    self.vesc_engine.set_target_rpm(0, self.vesc_engine.PROPULSION_KEY)
            time.sleep(0.5)

    def __stop_thread(self):
        self.can_go_setting = False
        self.send_last_pos_thread_alive = False
        self.__voltage_thread_alive = False
        self.__check_joystick_info_alive = False
        self.__voltage_thread.join()
        self._send_last_pos_thread.join()
        self.__joystick_info_thread.join()

    def on_event(self, event):
        if event == Events.Events.CREATE_FIELD:
            self.__stop_thread()
            self.statusOfUIObject.fieldButton = ButtonState.CHARGING
            self.statusOfUIObject.startButton = ButtonState.DISABLE
            self.statusOfUIObject.continueButton = ButtonState.DISABLE
            self.statusOfUIObject.joystick = ButtonState.DISABLE
            self.statusOfUIObject.audit = AuditButtonState.BUTTON_DISABLE
            return CreateFieldState.CreateFieldState(self.socketio, self.logger, self.smoothie, self.vesc_engine)
        elif event == Events.Events.CALIBRATION:
            self.__stop_thread()
            return CalibrateState(self.socketio, self.logger, self.smoothie, self.vesc_engine)
        elif event == Events.Events.ACTUATOR_SCREENING:
            self.__stop_thread()
            return ActuatorScreeningState(self.socketio, self.logger, self.smoothie, self.vesc_engine)
        elif event in [Events.Events.START_MAIN, Events.Events.START_AUDIT]:
            self.__stop_thread()
            self.statusOfUIObject.startButton = ButtonState.CHARGING
            self.statusOfUIObject.fieldButton = ButtonState.DISABLE
            self.statusOfUIObject.continueButton = ButtonState.DISABLE
            self.statusOfUIObject.joystick = False
            if event == Events.Events.START_MAIN:
                self.statusOfUIObject.audit = AuditButtonState.NOT_IN_USE
            elif event == Events.Events.START_AUDIT:
                self.statusOfUIObject.audit = AuditButtonState.IN_USE
            if self.smoothie is not None:
                self.smoothie.disconnect()
                self.smoothie = None
            if self.vesc_engine is not None:
                self.vesc_engine.close()
                self.vesc_engine = None
            return StartingState.StartingState(self.socketio, self.logger, (event == Events.Events.START_AUDIT))
        elif event in [Events.Events.CONTINUE_MAIN, Events.Events.CONTINUE_AUDIT]:
            self.__stop_thread()
            self.statusOfUIObject.continueButton = ButtonState.CHARGING
            self.statusOfUIObject.startButton = ButtonState.DISABLE
            self.statusOfUIObject.fieldButton = ButtonState.DISABLE
            self.statusOfUIObject.joystick = False
            if event == Events.Events.CONTINUE_MAIN:
                self.statusOfUIObject.audit = AuditButtonState.NOT_IN_USE
            elif event == Events.Events.CONTINUE_AUDIT:
                self.statusOfUIObject.audit = AuditButtonState.IN_USE
            if self.smoothie is not None:
                self.smoothie.disconnect()
                self.smoothie = None
            if self.vesc_engine is not None:
                self.vesc_engine.close()
                self.vesc_engine = None
            return ResumeState.ResumeState(self.socketio, self.logger, (event == Events.Events.CONTINUE_AUDIT))
        elif event == Events.Events.AUDIT_ENABLE:
            self.statusOfUIObject.audit = AuditButtonState.EXTRACTION_DISABLE
            return self
        elif event == Events.Events.AUDIT_DISABLE:
            self.statusOfUIObject.audit = AuditButtonState.EXTRACTION_ENABLE
            return self
        elif event == Events.Events.CLOSE_APP:
            self.__stop_thread()
            return self
        else:
            self.__stop_thread()
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
        if data["type"] == 'joystick':
            self.__last_joystick_info = time.time()
            x = int(data["x"])
            if x < 0:
                x *= -(config.A_MIN / 100)
            if x > 0:
                x *= config.A_MAX / 100
            y = int(data["y"])
            if self.lastValueX != x:
                self.smoothie.custom_move_to(
                    A_F=config.A_F_UI, A=x + self.learn_go_straight_angle)
                self.lastValueX = x
            if self.lastValueY != y:
                if y > 15 or y < -15:
                    y = (y / 100) * (config.SI_SPEED_UI * config.MULTIPLIER_SI_SPEED_TO_RPM * 0.9) + (
                        config.SI_SPEED_UI * config.MULTIPLIER_SI_SPEED_TO_RPM / 10)
                else:
                    y = 0

                print(f"vesc = {self.vesc_engine}")
                if self.vesc_engine is not None:
                    self.vesc_engine.set_target_rpm(y, self.vesc_engine.PROPULSION_KEY)
                self.lastValueY = y
            if(self.statusOfUIObject.wheelButton) :
                self.statusOfUIObject.wheelButton = ButtonState.DISABLE
                self.socketio.emit("wheel", "unrelease", namespace='/button')

        elif data["type"] == 'getInputVoltage':
            utilsFunction.sendInputVoltage(
                self.socketio, self.input_voltage["input_voltage"])

        elif data["type"] == 'getField':
            coords, other_fields, current_field_name = utilsFunction.updateFields(
                data["field_name"])
            fields_list = utilsFunction.load_field_list("../fields")
            self.socketio.emit('newField', json.dumps(
                {"field": coords, "other_fields": other_fields, "current_field_name": current_field_name,
                 "fields_list": fields_list}), namespace='/map')

        elif data["type"] == 'removeField':
            os.remove(
                "../fields/" + quote(data["field_name"], safe="", encoding='utf-8') + ".txt")
            fields_list = utilsFunction.load_field_list("../fields")

            if len(fields_list) > 0:
                coords, other_fields, current_field_name = utilsFunction.updateFields(
                    fields_list[0])
            else:
                coords, other_fields, current_field_name = list(), list(), ""

            self.socketio.emit('newField', json.dumps(
                {"field": coords, "other_fields": other_fields, "current_field_name": current_field_name,
                 "fields_list": fields_list}), namespace='/map')
            
        elif data["type"] == 'wheel':
            if(data["status"]=="release") :
                self.smoothie.freewheels()
                self.statusOfUIObject.wheelButton = ButtonState.ENABLE
                self.socketio.emit("wheel", "release", namespace='/button')
            else :
                self.smoothie.tighten_wheels()
                self.statusOfUIObject.wheelButton = ButtonState.DISABLE
                self.socketio.emit("wheel", "unrelease", namespace='/button')
        return self

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field

    def recreate_vesc(self):
        self.vesc_engine = None
        self.vesc_engine = utilsFunction.initVesc(self.logger)
        return self.vesc_engine