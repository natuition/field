import sys
sys.path.append('../')

from state_machine.Events import Events
from state_machine.utilsFunction import *
from state_machine.StateMachine import StateMachine

import importlib.util
from flask_socketio import SocketIO, emit
from engineio.payload import Payload
from werkzeug.exceptions import HTTPException
from flask import Flask, render_template, make_response, send_from_directory, request, redirect

import logging
import json
import os
import traceback
from urllib.parse import unquote
import posix_ipc
from threading import Thread
from datetime import datetime
from uiWebRobot.setting_page import SettingPageManager
import utility
from config import config
from uiWebRobot.state_machine.states import *


__author__ = 'Vincent LAMBERT'


class UIWebRobot:

    def __init__(self):
        self.__app = Flask(__name__)
        self.__setting_flask()
        self.__init_flask_route()  # ROUTE FLASK
        self.__socketio = SocketIO(
            self.__app, async_mode=None, logger=False, engineio_logger=False)
        self.__init_socketio()  # SOCKET IO
        self.__reload_config()
        self.init_params()
        self.demo_pause_client = utility.DemoPauseClient(
            config.DEMO_PAUSES_HOST, config.DEMO_PAUSES_PORT)

    def __init_socketio(self):
        self.__socketio.on_event(
            'data', self.on_socket_broadcast, namespace='/broadcast')
        self.__socketio.on_event('disconnect', self.on_disconnect)
        self.__socketio.on_event(
            'data', self.on_socket_data, namespace='/server')

    def __init_flask_route(self):
        self.__app.add_url_rule("/", view_func=self.index)
        self.__app.add_url_rule("/setting", view_func=self.setting)
        self.__app.add_url_rule("/map", view_func=self.maps)
        self.__app.add_url_rule("/offline.html", view_func=self.offline)
        self.__app.add_url_rule("/styles.css", view_func=self.style)
        self.__app.add_url_rule("/sw.js", view_func=self.worker)
        self.__app.add_url_rule("/js/socket.io.min.js",
                                view_func=self.socket_io_min)
        self.__app.add_url_rule(
            "/static/<random_time>/<file_path>/<file_name>", view_func=self.getJsFile)
        self.__app.add_url_rule("/reboot", view_func=self.reboot)
        self.__app.add_url_rule("/restart_ui", view_func=self.restart_ui)
        self.__app.add_url_rule("/calibrate", view_func=self.calibrate, methods=['GET', 'POST'])
        self.__app.add_url_rule("/actuator_screening", view_func=self.actuator_screening)

    def __setting_flask(self):
        self.__app.register_error_handler(Exception, self.handle_exception)
        self.__app.config['DEBUG'] = False
        self.__app.logger.disabled = True
        self.__log = logging.getLogger('werkzeug')
        self.__log.disabled = True
        Payload.max_decode_packets = 500

    def __reload_config(self):
        print("Reload config in application.py...")
        spec = importlib.util.spec_from_file_location(
            "config.name", "../config/config.py")
        self.__config = importlib.util.module_from_spec(spec)
        sys.modules["config.name"] = self.__config
        spec.loader.exec_module(self.__config)

    def init_params(self):
        self.__filename_for_send_from_directory = not "path" in send_from_directory.__code__.co_varnames
        with open("ui_language.json", "r", encoding='utf-8') as read_file:
            self.__ui_languages = json.load(read_file)
        thread_notification = Thread(target=self.catch_send_notification)
        thread_notification.setDaemon(True)
        thread_notification.start()
        self.__stateMachine = StateMachine(self.__socketio)

    def get_state_machine(self) -> StateMachine:
        return self.__stateMachine

    @staticmethod
    def load_coordinates(file_path):
        positions_list = []
        try:
            with open(file_path) as file:
                for line in file:
                    if line != "":
                        coords = list(map(float, line.split(" ")))
                        positions_list.append([coords[0], coords[1]])
        except OSError as e:
            return None
        return positions_list

    @staticmethod
    def load_ai_list(dir_path):
        ia_list = []
        for file in os.listdir(dir_path):
            if file.endswith(".conf"):
                ia_list.append(file.split(".conf")[0])
        return ia_list

    @staticmethod
    def formattingFieldPointsForSend(corners):
        coords = list()
        for coord in corners:
            coords.append([coord[1], coord[0]])
        coords.append(coords[0])
        return coords

    def catch_send_notification(self):
        try:
            posix_ipc.unlink_message_queue(
                self.__config.QUEUE_NAME_UI_NOTIFICATION)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except:
            pass

        notificationQueue = posix_ipc.MessageQueue(
            self.__config.QUEUE_NAME_UI_NOTIFICATION, posix_ipc.O_CREX)
        ui_language = self.__config.UI_LANGUAGE

        while True:
            try:
                notification = notificationQueue.receive(timeout=1)
                message_name = json.loads(notification[0])["message_name"]
                message = self.__ui_languages[message_name][ui_language]
                self.__socketio.emit('notification', {
                                     "message_name": message_name, "message": message}, namespace='/broadcast', broadcast=True)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                continue

    # SOCKET IO
    def on_socket_data(self, data):
        msg_socket_data_before_event = ["field_name", "allChecked"]
        msg_socket_to_event = {
            "stop": Events.STOP, 
            "run_target_detection": Events.CALIBRATION_DETECT,
            "run_target_move": Events.CALIBRATION_MOVE,
            "wheel": Events.WHEEL,
            "start": Events.START_MAIN,
            "continue": Events.CONTINUE_MAIN,
            "field": Events.CREATE_FIELD,
            "field_name": Events.VALIDATE_FIELD_NAME,
            "allChecked": Events.LIST_VALIDATION,
            "calibration_validate": Events.CALIBRATION_VALIDATE,
            "calibration_cancel": Events.CALIBRATION_CANCEL,
            "screening_start": Events.ACTUATOR_SCREENING_START,
            "screening_pause": Events.ACTUATOR_SCREENING_PAUSE,
            "screening_quit": Events.ACTUATOR_SCREENING_STOP
        }
        msg_socket_data_after_event = ["run_move_to_target", "step_axis_xy", "getInputVoltage", "modifyZone", "getField", "getStats", "getLastPath", "field"]
        if "type" in data:
            if data["type"] in msg_socket_data_before_event:
                self.get_state_machine().on_socket_data(data)
            if data["type"] in msg_socket_to_event.keys():
                self.get_state_machine().on_event(msg_socket_to_event[data["type"]])
            if data["type"] in msg_socket_data_after_event:
                self.get_state_machine().on_socket_data(data)

            if data["type"] == "joystick" and isinstance(self.get_state_machine().currentState, (WaitWorkingState, CreateFieldState)):
                self.get_state_machine().on_socket_data(data)
            elif data["type"] == "demo_resume_cmd":
                self.demo_pause_client.send_resume_cmd()
            elif data["type"] == "validerZone":
                data["client_id"] = request.sid
                self.get_state_machine().on_socket_data(data)
                self.get_state_machine().on_event(Events.VALIDATE_FIELD)
            elif data["type"] == "removeField":
                if isinstance(self.get_state_machine().currentState, WaitWorkingState):
                    self.get_state_machine().on_socket_data(data)

    def on_socket_broadcast(self, data):
        emit(data["type"], data, broadcast=True)

    def on_disconnect(self):
        if isinstance(self.get_state_machine().currentState, (WaitWorkingState,CreateFieldState)):
            self.get_state_machine().on_socket_data(
                {"type": "joystick", "x": 0, "y": 0})

    # ROUTE FLASK

    def index(self):
        sn = self.__config.ROBOT_SN
        # sn = "SNXXX"
        statusOfUIObject = self.get_state_machine().getStatusOfControls()

        IA_list = UIWebRobot.load_ai_list("../yolo")
        Field_list = load_field_list("../fields")

        if not Field_list:
            Field_list = None
            current_field = None
        else:
            Field_list.sort(key=str.casefold)
            link_path = os.path.realpath("../field.txt")
            current_field = (link_path.split("/")[-1]).split(".")[0]
            current_field = unquote(current_field, encoding='utf-8')

        if isinstance(self.get_state_machine().currentState, CalibrateState):
            return redirect('/calibrate')

        if isinstance(self.get_state_machine().currentState, ActuatorScreeningState):
            return redirect('/actuator_screening')

        if isinstance(self.get_state_machine().currentState, ErrorState):
            if self.get_state_machine().currentState.getReason():
                return render_template("Error.html", sn=sn, error_message=self.__ui_languages["Error_500"][self.__get_ui_language()], reason=self.get_state_machine().currentState.getReason()), 500
            else:
                return render_template("Error.html", sn=sn, error_message=self.__ui_languages["Error_500"][self.__get_ui_language()]), 500

        return render_template('UIRobot.html', demo_mode=self.__config.ALLOW_DEMO_PAUSES, sn=sn, statusOfUIObject=statusOfUIObject, ui_languages=self.__ui_languages, ui_language=self.__get_ui_language(), Field_list=Field_list, current_field=current_field, IA_list=IA_list, now=datetime.now().strftime("%H_%M_%S_%f"), slider_min=self.__config.SLIDER_CREATE_FIELD_MIN, slider_max=self.__config.SLIDER_CREATE_FIELD_MAX, slider_step=self.__config.SLIDER_CREATE_FIELD_STEP)

    def setting(self):
        sn = self.__config.ROBOT_SN

        if not isinstance(self.get_state_machine().currentState, WaitWorkingState):
            return redirect('/')

        if not self.get_state_machine().currentState.can_go_setting:
            return redirect('/')

        setting_page_manager = SettingPageManager(
            self.__socketio, self.__ui_languages, self.__config, self.__reload_config)
        try:
            return render_template('UISetting.html', sn=sn, ui_languages=self.__ui_languages, ui_language=self.__get_ui_language(), now=datetime.now().strftime("%H_%M_%S_%f"), setting_page_generate=setting_page_manager.generate_html())
        except Exception as e:
            print(f"Error : {e}")
            traceback.print_exc()
            return redirect('/')

    def maps(self):
        if not isinstance(self.get_state_machine().currentState, (WorkingState, WaitWorkingState, CreateFieldState, ResumeState, StartingState)):
            return redirect('/')
        myCoords = [0, 0]
        field = self.get_state_machine().getField()
        if field is None:
            field = UIWebRobot.load_coordinates("../field.txt")
        if field is None:
            return render_template('map.html', myCoords=myCoords, now=datetime.now().strftime("%H_%M_%S__%f"))
        else:
            coords_other = get_other_field()
            coords_field = UIWebRobot.formattingFieldPointsForSend(field)
            if coords_other:
                return render_template('map.html', coords_field=coords_field, myCoords=myCoords, coords_other=coords_other, now=datetime.now().strftime("%H_%M_%S__%f"))
            else:
                return render_template('map.html', coords_field=coords_field, myCoords=myCoords, now=datetime.now().strftime("%H_%M_%S__%f"))

    def calibrate(self):
        if not isinstance(self.get_state_machine().currentState, (WaitWorkingState, CalibrateState)):
            return redirect('/')
        
        if isinstance(self.get_state_machine().currentState, (WaitWorkingState)):
            self.get_state_machine().on_event(Events.CALIBRATION)

        currentState: CalibrateState = self.get_state_machine().currentState

        if request.method == 'POST':
            if not currentState.checkPassword(request.form['password']):
                return render_template(currentState.getStatusOfControls()["currentHTML"], ui_languages=self.__ui_languages, ui_language=self.__get_ui_language(), password_wrong=True)
            else:
                currentState.getStatusOfControls()["currentHTML"] = "CalibrateDetect.html"
        return render_template(currentState.getStatusOfControls()["currentHTML"], ui_languages=self.__ui_languages, ui_language=self.__get_ui_language())

    def actuator_screening(self):
        if not isinstance(self.get_state_machine().currentState, (WaitWorkingState, ActuatorScreeningState)):
            return redirect('/')
        
        if isinstance(self.get_state_machine().currentState, (WaitWorkingState)):
            self.get_state_machine().on_event(Events.ACTUATOR_SCREENING)

        currentState: ActuatorScreeningState = self.get_state_machine().currentState

        return render_template(currentState.getStatusOfControls()["currentHTML"], ui_languages=self.__ui_languages, ui_language=self.__get_ui_language())
    
    def __get_ui_language(self):
        ui_language = self.__config.UI_LANGUAGE
        if ui_language not in self.__ui_languages["Supported Language"]:
            ui_language = "en"
        return ui_language
        

    def offline(self):
        sn = self.__config.ROBOT_SN
        ui_language = self.__config.UI_LANGUAGE
        if ui_language not in self.__ui_languages["Supported Language"]:
            ui_language = "en"
        return render_template('offline.html', sn=sn, ui_languages=self.__ui_languages, ui_language=ui_language)

    def style(self):
        if self.__filename_for_send_from_directory:
            response = make_response(send_from_directory(
                self.__app.static_folder, filename='css/style.css'))
        else:
            response = make_response(send_from_directory(
                self.__app.static_folder, path='css/style.css'))
        response.headers['Content-Type'] = 'text/css'
        return response

    def worker(self):
        if self.__filename_for_send_from_directory:
            response = make_response(send_from_directory(
                self.__app.static_folder, filename='js/offline_worker.js'))
        else:
            response = make_response(send_from_directory(
                self.__app.static_folder, path='js/offline_worker.js'))
        response.headers['Content-Type'] = 'application/javascript'
        return response

    def socket_io_min(self):
        if self.__filename_for_send_from_directory:
            response = make_response(send_from_directory(
                self.__app.static_folder, filename='js/socket.io.min.js'))
        else:
            response = make_response(send_from_directory(
                self.__app.static_folder, path='js/socket.io.min.js'))
        response.headers['Content-Type'] = 'application/javascript'
        return response

    def getJsFile(self, file_path, file_name, random_time):
        if ".js" in file_name:
            if self.__filename_for_send_from_directory:
                response = make_response(send_from_directory(
                    self.__app.static_folder, filename=f'{file_path}/{file_name}', mimetype='application/javascript'))
            else:
                response = make_response(send_from_directory(
                    self.__app.static_folder, path=f'{file_path}/{file_name}', mimetype='application/javascript'))
        else:
            if self.__filename_for_send_from_directory:
                response = make_response(send_from_directory(
                    self.__app.static_folder, filename=f'{file_path}/{file_name}', mimetype='application/css'))
            else:
                response = make_response(send_from_directory(
                    self.__app.static_folder, path=f'{file_path}/{file_name}', mimetype='application/css'))
        return response

    def reboot(self):
        os.system('sudo reboot')
        return None

    def restart_ui(self):
        os.system('sudo systemctl restart UI')
        return None

    def handle_exception(self, e):
        # pass through HTTP errors

        if isinstance(e, HTTPException):
            return e

        # now you're handling non-HTTP exceptions only
        self.get_state_machine().on_event(Events.ERROR)
        sn = self.__config.ROBOT_SN
        ui_language = self.__config.UI_LANGUAGE
        if ui_language not in self.__ui_languages["Supported Language"]:
            ui_language = "en"
        exc_type, value, traceback = sys.exc_info()
        print(f"Error handled : {exc_type} : {value}.\n{traceback}.")
        return render_template("Error.html", sn=sn, error_message=self.__ui_languages["Error_500"][ui_language], reason=f"{str(exc_type)} : {value}"), 500

    def run(self, host=None, port=None, debug=None, load_dotenv=True, **options):
        self.__app.run(host, port, debug, load_dotenv, **options)


def main():
    uiWebRobot = UIWebRobot()
    try:
        uiWebRobot.run(host="0.0.0.0", port="80",
                       debug=True, use_reloader=False)
    finally:
        if isinstance(uiWebRobot.get_state_machine().currentState, WaitWorkingState):
            uiWebRobot.get_state_machine().on_event(Events.CLOSE_APP)
        print("Closing app...")


if __name__ == "__main__":
    main()
