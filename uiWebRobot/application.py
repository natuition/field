import atexit
import importlib.util
import signal
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
import sys

from safe_import_of_config import config
from uiWebRobot.state_machine.Events import Events
from uiWebRobot.state_machine.utilsFunction import *
from uiWebRobot.state_machine.StateMachine import StateMachine
from uiWebRobot.setting_page import SettingPageManager
from uiWebRobot.state_machine.states import *
from notification import RobotStateClient
from shared_class.robot_synthesis import RobotSynthesis
import utility
from logger import LoggerFactory

__author__ = 'Vincent LAMBERT'


class IgnoreSocketIODisconnected:
    def __init__(self, app):
        self.app = app

    def __call__(self, environ, start_response):
        try:
            return self.app(environ, start_response)
        except KeyError as e:
            if str(e) == "'Session is disconnected'":
                start_response("400 Bad Request", [("Content-Type", "text/plain")])
                return [b"Socket.IO session already disconnected"]
            raise

class UIWebRobot:

    def __init__(self):
        self.__logger = LoggerFactory.create(self.__class__.__name__)
        self.__app = Flask(__name__)
        self.__setting_flask()
        self.__init_flask_route()  # ROUTE FLASK
        self.__socketio = SocketIO(
            self.__app,
            async_mode="threading",
            logger=False,
            engineio_logger=False
        )
        self.__app.wsgi_app = IgnoreSocketIODisconnected(self.__app.wsgi_app)
        self.__init_socketio()  # SOCKET IO
        self.__reload_config()
        self.__robot_state_client = RobotStateClient()
        self.init_params()
        self.demo_pause_client = utility.DemoPauseClient(
            config.DEMO_PAUSES_HOST, config.DEMO_PAUSES_PORT)
        
        self.__logger.info(f"UIWebRobot started ✅")
        
    @property
    def app(self):
        return self.__app

    @property
    def socketio(self):
        return self.__socketio

    def exit(self):
        self.__logger.info("Send RobotSynthesis...")

        try:
            self.__robot_state_client.set_robot_state(RobotSynthesis.OP)
        except Exception:
            self.__logger.error(traceback.format_exc())

        self.__logger.info("Stopping catch_send_notification thread...")

        try:
            self.__notification_thread_alive = False

            if self.__notification_thread is not None and self.__notification_thread.is_alive():
                self.__notification_thread.join(timeout=1.0)

                if self.__notification_thread.is_alive():
                    self.__logger.warning("catch_send_notification thread did not stop cleanly.")
                else:
                    self.__logger.info("catch_send_notification thread stopped.")
        except Exception:
            self.__logger.error(traceback.format_exc())

        self.__logger.info("Exit done ✅")

    def on_connect(self):
        self.__logger.debug("A client is connected.")

    def __init_socketio(self):
        self.__socketio.on_event(
            'data', self.on_socket_broadcast, namespace='/broadcast')
        self.__socketio.on_event('disconnect', self.on_disconnect)
        self.__socketio.on_event('connect', self.on_connect)
        self.__socketio.on_event(
            'data', self.on_socket_data, namespace='/server')

    def __init_flask_route(self):
        self.__app.add_url_rule("/", view_func=self.index)
        self.__app.add_url_rule("/received_on_socket_data/<event_name>", view_func=self.received_on_socket_data)
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
        self.__app.add_url_rule("/run_life_line", view_func=self.run_life_line)
        self.__app.add_url_rule("/analyse_data_vesc", view_func=self.analyse_data_vesc)

    def __setting_flask(self):
        self.__app.register_error_handler(Exception, self.handle_exception)
        self.__app.config['DEBUG'] = False
        self.__app.logger.disabled = True
        self.__log = logging.getLogger('werkzeug')
        self.__log.disabled = True
        Payload.max_decode_packets = 500

    def __reload_config(self):
        self.__logger.info("Reload config in application.py...")
        config_path = os.path.abspath("./config/config.py")
        self.__logger.debug(f"Config path: {config_path}")
        spec = importlib.util.spec_from_file_location("config.name", config_path)
        self.__logger.debug("Reload config... 2 spec OK")
        self.__config = importlib.util.module_from_spec(spec)
        self.__logger.debug("Reload config... 3 module OK")
        sys.modules["config.name"] = self.__config
        self.__logger.debug("Reload config... 4 before exec_module")
        spec.loader.exec_module(self.__config)
        self.__logger.debug("Reload config... 5 after exec_module OK")

    def init_params(self):
        self.__logger.info("Init params...")
        self.__filename_for_send_from_directory = not "path" in send_from_directory.__code__.co_varnames
        with open("./uiWebRobot/ui_language.json", "r", encoding='utf-8') as read_file:
            self.__ui_languages = json.load(read_file)
        
        self.__logger.info("Starting thread for catch_send_notification...")
        self.__notification_thread_alive = True
        self.__notification_thread = Thread(target=self.catch_send_notification)
        self.__notification_thread.daemon = True
        self.__notification_thread.start()
        
        self.__logger.debug("Starting state machine...")
        self.__stateMachine = StateMachine(self.__socketio, self.__robot_state_client)

    def get_state_machine(self) -> StateMachine:
        return self.__stateMachine

    def load_coordinates(self, file_path):
        positions_list = []
        try:
            with open(file_path) as file:
                for line in file:
                    if line != "":
                        coords = list(map(float, line.split(" ")))
                        positions_list.append([coords[0], coords[1]])
        except OSError as e:
            return None
        if len(positions_list) == 0:
            self.__logger.error(f"Erreur : Le fichier {file_path} est vide.")
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
        self.__logger.info("Starting catch_send_notification...")
        try:
            self.__logger.debug("Unlinking message queue...")
            posix_ipc.unlink_message_queue(
                self.__config.QUEUE_NAME_UI_NOTIFICATION)
            self.__logger.debug("Unlinked message queue.")
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            pass

        self.__logger.debug("Creating message queue...")
        notificationQueue = posix_ipc.MessageQueue(
            self.__config.QUEUE_NAME_UI_NOTIFICATION, posix_ipc.O_CREX)
        self.__logger.debug("Created message queue.")
        ui_language = self.__config.UI_LANGUAGE

        self.__logger.debug("Waiting for notification...")
        while self.__notification_thread_alive:
            try:
                notification = notificationQueue.receive(timeout=1)
                self.__logger.debug(f"Received notification: {notification}")
                message_name = json.loads(notification[0])["message_name"]
                self.__logger.debug(f"Message name: {message_name}")
                message = self.__ui_languages[message_name][ui_language]
                self.__logger.debug(f"Message: {message}")
                self.__socketio.emit('notification', {
                                     "message_name": message_name, "message": message}, namespace='/broadcast', broadcast=True)
                self.__logger.debug("Notification sent.")
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except posix_ipc.BusyError:
                continue
            except Exception as e:
                self.__logger.error(f"Error while receiving notification: {e}")
                self.__logger.error(traceback.format_exc())

    # SOCKET IO
    def on_socket_data(self, data):
        msg_socket_data_before_event = [
            Events.VALIDATE_FIELD_NAME, 
            Events.LIST_VALIDATION, 
            Events.WHEEL
        ]
        msg_socket_to_event = [
            Events.STOP, 
            Events.ERROR,
            Events.CALIBRATION_DETECT,
            Events.CALIBRATION_MOVE,
            Events.START_MAIN,
            Events.CONTINUE_MAIN,
            Events.CREATE_FIELD,
            Events.VALIDATE_FIELD_NAME,
            Events.LIST_VALIDATION,
            Events.CALIBRATION_VALIDATE,
            Events.CALIBRATION_CANCEL,
            Events.ACTUATOR_SCREENING_START,
            Events.ACTUATOR_SCREENING_PAUSE,
            Events.ACTUATOR_SCREENING_STOP,
            Events.PHYSICAL_BLOCAGE
        ]
        msg_socket_data_after_event = ["run_move_to_target", "step_axis_xy", "getInputVoltage", "modifyZone", "getField", "getStats", "getLastPath", "create_field", "wait_working_state_refresh", "penetrometry_new_params"]
        
        if "type" in data:
            if data["type"] in [str(i) for i in msg_socket_data_before_event]:
                self.get_state_machine().on_socket_data(data)

            if data["type"] in [str(i) for i in msg_socket_to_event]:
                self.get_state_machine().on_event(Events.from_str(data["type"]))

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
        if isinstance(self.get_state_machine().currentState, (WaitWorkingState,CreateFieldStateWithNavX,CreateFieldState)):
            self.get_state_machine().on_socket_data(
                {"type": "joystick", "x": 0, "y": 0})

    # ROUTE FLASK

    def index(self):
        sn = self.__config.ROBOT_SN
        # sn = "SNXXX"
        statusOfUIObject = self.get_state_machine().getStatusOfControls()

        IA_list = UIWebRobot.load_ai_list("./yolo")
        Field_list = load_field_list("./fields")

        if not Field_list:
            Field_list = None
            current_field = None
        else:
            Field_list.sort(key=str.casefold)
            link_path = os.path.realpath("./field.txt")
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

        return render_template('UIRobot.html', create_field_with_navx=self.__config.CREATE_FIELD_WITH_NAVX, demo_mode=self.__config.ALLOW_DEMO_PAUSES, sn=sn, statusOfUIObject=statusOfUIObject, ui_languages=self.__ui_languages, ui_language=self.__get_ui_language(), Field_list=Field_list, current_field=current_field, IA_list=IA_list, now=datetime.now().strftime("%H_%M_%S_%f"), slider_min=self.__config.SLIDER_CREATE_FIELD_MIN, slider_max=self.__config.SLIDER_CREATE_FIELD_MAX, slider_step=self.__config.SLIDER_CREATE_FIELD_STEP)

    def received_on_socket_data(self, event_name):
        self.on_socket_data({"type": event_name})
        response = self.__app.response_class(
                response=json.dumps({True}),
                status=200,
                mimetype='application/json'
        )
        return response

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
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            self.__logger.error(f"Error : {e}")
            traceback.print_exc()
            return redirect('/')

    def maps(self):
        if not isinstance(self.get_state_machine().currentState, (WorkingState, WaitWorkingState, CreateFieldState, CreateFieldStateWithNavX, ResumeState, StartingState, PhysicalBlocageState)):
            return redirect('/')
        myCoords = [0, 0]
        if isinstance(self.get_state_machine().currentState, (PhysicalBlocageState)):
            field = None
        else:
            field = self.get_state_machine().getField()

        if (field is None) or (len(field) == 0):
            field = self.load_coordinates("./field.txt")
        if (field is None) or (len(field) == 0):
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

        """
        if request.method == 'POST':
            if not currentState.checkPassword(request.form['password']):
                return render_template(currentState.getStatusOfControls()["currentHTML"], ui_languages=self.__ui_languages, ui_language=self.__get_ui_language(), password_wrong=True)
            else:
                currentState.getStatusOfControls()["currentHTML"] = "CalibrateDetect.html"
        """
        return render_template(currentState.getStatusOfControls()["currentHTML"], ui_languages=self.__ui_languages, ui_language=self.__get_ui_language())

    def actuator_screening(self):
        if not isinstance(self.get_state_machine().currentState, (WaitWorkingState, ActuatorScreeningState)):
            return redirect('/')
        
        if isinstance(self.get_state_machine().currentState, (WaitWorkingState)):
            self.get_state_machine().on_event(Events.ACTUATOR_SCREENING)

        currentState: ActuatorScreeningState = self.get_state_machine().currentState

        return render_template(currentState.getStatusOfControls()["currentHTML"], ui_languages=self.__ui_languages, ui_language=self.__get_ui_language(), hasStarted=currentState.getStatusOfControls()["hasStarted"], count=currentState.getStatusOfControls()["count"], now=datetime.now().strftime("%H_%M_%S_%f"))
    

    def analyse_data_vesc(self):
        if not isinstance(self.get_state_machine().currentState, (WorkingState)):
            return redirect('/')
        return render_template("AnalyseDataVesc.html", ui_languages=self.__ui_languages, ui_language=self.__get_ui_language(), now=datetime.now().strftime("%H_%M_%S_%f"))


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
        self.__robot_state_client.set_robot_state(RobotSynthesis.UI_RESTART_APP)
        os.system('sudo reboot')
        return None

    def restart_ui(self):
        os.system('sudo systemctl restart UI')
        return None

    def run_life_line(self):
        utility.life_line_reset()
        return "Restarting equipment on the current life line."

    def handle_exception(self, e):
        # pass through HTTP errors

        if isinstance(e, HTTPException):
            self.__logger.error(f"Error handled : {e}.")
            return e

        # now you're handling non-HTTP exceptions only
        self.get_state_machine().on_event(Events.ERROR)
        sn = self.__config.ROBOT_SN
        ui_language = self.__config.UI_LANGUAGE
        if ui_language not in self.__ui_languages["Supported Language"]:
            ui_language = "en"
        exc_type, exc_value, exc_traceback = sys.exc_info()
        self.__logger.error(f"Error handled : {exc_type} : {exc_value}.")
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        return render_template("Error.html", sn=sn, error_message=self.__ui_languages["Error_500"][ui_language], reason=f"{str(exc_type)} : {exc_value}"), 500

    def run(self, host=None, port=None, debug=None, load_dotenv=True, **options):
        self.__app.run(host, port, debug, load_dotenv, **options)


def main():
    LoggerFactory.set_level(config.LOG_LEVEL_UI)
    runtime_logger = LoggerFactory.create("Runtime")
    try:
        uiWebRobot = UIWebRobot()
        host="0.0.0.0"
        port="80"
        runtime_logger.info(f"Starting UIWebRobot on host: {host} port: {port}.")
        uiWebRobot.run(host=host, port=port,
                       debug=True, use_reloader=False)
    except Exception:
        runtime_logger.error(traceback.format_exc(), stack_info=True)
    finally:
        if isinstance(uiWebRobot.get_state_machine().currentState, WaitWorkingState):
            runtime_logger.info("Closing app...")
            uiWebRobot.get_state_machine().on_event(Events.CLOSE_APP)
        uiWebRobot.exit()
        
# def shutdown_ui(*args):
#     global _shutdown_done

#     if _shutdown_done:
#         return

#     _shutdown_done = True

#     try:
#         if isinstance(uiWebRobot.get_state_machine().currentState, WaitWorkingState):
#             uiWebRobot.get_state_machine().on_event(Events.CLOSE_APP)
#         uiWebRobot.exit()

#     except Exception:
#         try:
#             runtime_logger.error("Error during shutdown_ui: " + traceback.format_exc())
#         except Exception:
#             pass

if __name__ == "__main__":
    main()

# # For Gunicorn
# LoggerFactory.set_level(config.LOG_LEVEL_UI)
# runtime_logger = LoggerFactory.create("Runtime")
# runtime_logger.info(f"Starting UIWebRobot...")
# uiWebRobot = UIWebRobot()
# app = uiWebRobot.app
# socketio = uiWebRobot.socketio
# _shutdown_done = False

# signal.signal(signal.SIGTERM, shutdown_ui)
# signal.signal(signal.SIGINT, shutdown_ui)
# atexit.register(shutdown_ui)
