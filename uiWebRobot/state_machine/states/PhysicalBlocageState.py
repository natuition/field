import sys
import threading

sys.path.append('../')

from flask_socketio import SocketIO

from uiWebRobot.state_machine import State
from uiWebRobot.state_machine import Events
from uiWebRobot.state_machine import utilsFunction
from uiWebRobot.state_machine import GearboxProtection
from uiWebRobot.state_machine.states import ErrorState, ResumeState

from shared_class.robot_synthesis import RobotSynthesis

from uiWebRobot.state_machine.FrontEndObjects import ButtonState, FrontEndObjects
from config import config
import utility
import adapters

# This state corresponds when the robot is physically blocking.
class PhysicalBlocageState(State.State) :
    def __init__(
            self, 
            socketio: SocketIO, 
            logger: utility.Logger,
            smoothie: adapters.SmoothieAdapter = None,
            vesc_engine : adapters.VescAdapterV4 = None
            ) :
        self.socketio = socketio
        self.logger = logger
        self.smoothie = smoothie
        self.vesc_engine = vesc_engine

        self.robot_synthesis_value = RobotSynthesis.UI_PHYSICAL_BLOCAGE

        msg = f"[{self.__class__.__name__}] -> Physically blocked"
        self.logger.write_and_flush(msg + "\n")
        print(msg)    

        self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.DISABLE,
                                                startButton=ButtonState.DISABLE,
                                                continueButton=ButtonState.DISABLE,
                                                stopButton=ButtonState.NOT_HERE,
                                                wheelButton=ButtonState.NOT_HERE,
                                                removeFieldButton=ButtonState.DISABLE,
                                                joystick=False,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE)

        self.field = None
        self.socketio.emit('reload', {}, namespace='/broadcast', broadcast=True)
        self.__gps = adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP)
        self.__blocagePos = self.__gps.get_last_position()
        self.__allPath = []
        self.__gb = GearboxProtection()

        self.__backward_thread_alive = True
        self.__backward_thread = threading.Thread(target=self.__backward_thread_tf,
                                                 args=(lambda: self.__backward_thread_alive,
                                                       self.vesc_engine,
                                                       self.socketio),
                                                 daemon=True)
        self.__backward_thread.start()

        self.__check_backward_thread_alive = True
        self.__check_backward_thread = threading.Thread(target=self.__check_backward_thread_tf,
                                                 args=(lambda: self.__check_backward_thread_alive,
                                                       self.vesc_engine,
                                                       self.socketio),
                                                 daemon=True)
        self.__check_backward_thread.start()

    def on_event(self, event):
        if event in Events.Events.PHYSICAL_BLOCAGE:
            self.__stop_thread()
            self.statusOfUIObject.continueButton = ButtonState.CHARGING
            self.statusOfUIObject.startButton = ButtonState.DISABLE
            self.statusOfUIObject.fieldButton = ButtonState.DISABLE
            self.statusOfUIObject.joystick = False
            if self.smoothie is not None:
                self.smoothie.disconnect()
                self.smoothie = None
            if self.vesc_engine is not None:
                self.vesc_engine.close()
                self.vesc_engine = None
            return ResumeState.ResumeState(self.socketio, self.logger, (event == Events.Events.CONTINUE_AUDIT))
        
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

    def __backward_thread_tf(self) :
        speed = -config.SI_SPEED_FWD
        self.vesc_engine.set_target_rpm(speed * config.MULTIPLIER_SI_SPEED_TO_RPM, self.vesc_engine.PROPULSION_KEY)
        while self.__backward_thread_alive :
            self.smoothie.custom_move_to(A_F=config.A_F_MAX, A=0)

    def __check_backward_thread_tf(self) :
        while self.__check_backward_thread_alive :
            current_position = self.__gps.get_last_position()
            self.__allPath.append(current_position)
            self.__gb.store_coord(current_position)
            if self.__gb.is_physically_blocked() :
                utilsFunction.change_state(Events.Events.ERROR)
            if self.__gb.is_remote(self.__blocagePos) :
                utilsFunction.change_state(Events.Events.PHYSICAL_BLOCAGE)

    def __stop_thread(self):
        self.__backward_thread_alive = False
        self.__backward_thread.join()
        self.__check_backward_thread_alive = False
        self.__check_backward_thread.join()


             