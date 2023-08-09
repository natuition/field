import sys
sys.path.append('../')

import logging
import importlib.util
from flask_socketio import SocketIO
from engineio.payload import Payload
from flask import Flask, render_template, make_response, send_from_directory, redirect
from flask_cors import CORS

import fileinput
import pwd
import grp
import os
import shutil
import subprocess
import re
import json

import adapters
import utility
from cameraCalibration import CameraCalibration

__author__ = 'Vincent LAMBERT'


class Deployement:

    def __init__(self) -> None:
        self.__app = Flask(__name__)
        CORS(self.__app)
        self.__setting_flask()
        self.__init_flask_route()  # ROUTE FLASK
        self.__socketio = SocketIO(
            self.__app, async_mode=None, logger=False, engineio_logger=False)
        self.__init_socketio()  # SOCKET IO
        self.__reload_config()
        self.__smoothie: adapters.SmoothieAdapter = adapters.SmoothieAdapter(
            utility.get_smoothie_vesc_addresses()["smoothie"], calibration_at_init=False)
        self.__cameraCalibration: CameraCalibration = CameraCalibration()
        self.__offset_x, self.__offset_y = 0, 0
        self.__log = {
            'Date': None,
            'Vesc foc': 'KO',
            'Vesc Z PWM': 'KO',
            'X Y DIR setup': 'KO',
            'Camera focus': 'KO',
            'Camera crop': 'KO',
            'Camera offset': 'KO',
            'Client configuration apply': 'KO',
        }

    def __setting_flask(self):
        self.__app.config['DEBUG'] = False
        self.__app.logger.disabled = True
        self.__logger = logging.getLogger('werkzeug')
        self.__logger.disabled = True
        Payload.max_decode_packets = 500

    def __init_flask_route(self):
        self.__app.add_url_rule("/", view_func=self.__index)
        self.__app.add_url_rule(
            "/register", methods=['POST'], view_func=self.__register)
        self.__app.add_url_rule("/vesc_foc", view_func=self.__vesc_foc)
        self.__app.add_url_rule("/vesc_z", view_func=self.__vesc_z)
        self.__app.add_url_rule("/x_y_dir", view_func=self.__x_y_dir)
        self.__app.add_url_rule("/camera_focus", view_func=self.__camera_focus)
        self.__app.add_url_rule("/camera_crop_picture",
                                view_func=self.__camera_crop_picture)
        self.__app.add_url_rule("/camera_target_detection",
                                view_func=self.__camera_target_detection)
        self.__app.add_url_rule("/camera_target_move",
                                view_func=self.__camera_target_move)
        self.__app.add_url_rule(
            "/client_config", view_func=self.__client_config)
        self.__app.add_url_rule("/end", view_func=self.__end)
        self.__app.add_url_rule("/show_pdf/<filename>",
                                view_func=self.__show_pdf)

    def __init_socketio(self):
        self.__socketio.on_event(
            'client_config', self.__on_client_config, namespace='/server')
        self.__socketio.on_event(
            'validate_log', self.__on_validate_log, namespace='/server')
        self.__socketio.on_event(
            'run_round_detection', self.__on_run_round_detection, namespace='/server')
        self.__socketio.on_event(
            'run_target_detection', self.__on_run_target_detection, namespace='/server')
        self.__socketio.on_event(
            'run_move_to_target', self.__on_run_move_to_target, namespace='/server')
        self.__socketio.on_event(
            'move_step', self.__on_move_step, namespace='/server')
        self.__socketio.on_event(
            'x_y_dir', self.__on_x_y_dir, namespace='/server')

    def run(self, host=None, port=None, debug=None, load_dotenv=True, **options):
        self.__app.run(host, port, debug, load_dotenv, **options)

    """ ---------------------------- Flask ROUTE ---------------------------- """

    def __show_pdf(self, filename):
        if not "path" in send_from_directory.__code__.co_varnames:
            response = make_response(send_from_directory(
                self.__app.static_folder, filename=f'pdf/{filename}.pdf'))
        else:
            response = make_response(send_from_directory(
                self.__app.static_folder, path=f'pdf/{filename}.pdf'))
        response.headers['Content-Type'] = 'application/pdf'
        response.headers['Content-Disposition'] = f'inline; filename={filename}.pdf'
        return response

    def __index(self):
        d = utility.get_current_time().split(" ")[0].split("-")
        h = utility.get_current_time().split(" ")[1].split("-")
        self.__date = f"{d[0]}/{d[1]}/{d[2]} à {h[0]}h {h[1]}min"
        self.__log["Date"] = self.__date
        return render_template('index.html', SN=self.__config.ROBOT_SN)

    def __register(self):
        return redirect("/vesc_foc")

    def __vesc_foc(self):
        return render_template('vesc_foc.html', SN=self.__config.ROBOT_SN, log=self.__log)

    def __vesc_z(self):
        return render_template('vesc_z.html', SN=self.__config.ROBOT_SN, log=self.__log)

    def __x_y_dir(self):
        return render_template('x_y_dir.html', A_MAX=self.__config.A_MAX, Y_MAX=self.__config.Y_MAX, X_MAX=self.__config.X_MAX, IN_RESET=json.dumps(False), SN=self.__config.ROBOT_SN, log=self.__log)

    def __camera_focus(self):
        self.__cameraCalibration.focus_adjustment_step()
        return render_template('camera_focus.html', SN=self.__config.ROBOT_SN, log=self.__log)

    def __camera_crop_picture(self):
        try:
            self.__cameraCalibration.focus_adjustment_step_validate()
        except:
            pass
        return render_template('camera_crop_picture.html', SN=self.__config.ROBOT_SN, log=self.__log)

    def __camera_target_detection(self):
        res = self.__smoothie.ext_calibrate_cork()
        if res != self.__smoothie.RESPONSE_OK:
            print("Initial cork calibration was failed, smoothie response:\n", res)
        return render_template('camera_target_detection.html', SN=self.__config.ROBOT_SN, log=self.__log)

    def __camera_target_move(self):
        if self.__cameraCalibration.target_x is None:
            return redirect("/camera_target_detection")
        return render_template('camera_target_move.html', SN=self.__config.ROBOT_SN, log=self.__log)

    def __client_config(self):
        utility.change_config_value("../config/config.py",
            "CORK_TO_CAMERA_DISTANCE_X", self.__config.CORK_TO_CAMERA_DISTANCE_X + self.__offset_x)
        utility.change_config_value("../config/config.py",
            "CORK_TO_CAMERA_DISTANCE_Y", self.__config.CORK_TO_CAMERA_DISTANCE_Y + self.__offset_y)
        return render_template('client_config.html', SN=self.__config.ROBOT_SN)

    def __end(self):
        return render_template('end.html', log=self.__log, SN=self.__config.ROBOT_SN)

    """ ---------------------------- Socketio ROUTE ---------------------------- """

    def __on_client_config(self, data):
        if data["apply"]:
            for service in ["ntripClient.service", "UI.service", "configBackup.service"]:
                shutil.copyfile(
                    f"./services/{service}", f"/etc/systemd/system/{service}")
                os.system(f"sudo systemctl enable {service}")
            self.__log["Client configuration apply"] = "OK"
            utility.create_directories(
                f"configFinal{self.__config.ROBOT_SN}", "/media/smoothie")
            # python_conf
            os.system(f"sudo rm -f ./configFinal{self.__config.ROBOT_SN}/*")
            d = utility.get_current_time().split(" ")[0].split("-")
            config_final_path = f"./configFinal{self.__config.ROBOT_SN}/config_{d[0]}_{d[1]}_{d[2]}.py"
            shutil.copyfile(f"../config/config.py", config_final_path)
            shutil.chown(config_final_path, "violette", "violette")
            # smoothie_conf
            disk = subprocess.check_output(
                'lsblk -Jo KNAME,SIZE,TYPE | grep \'"type": "part"\' | grep \'"size": "14.9G\' | grep -Po \'"kname": ".*?"\' | cut -d \' \' -f 2 | tr -dc \'[:alnum:]\'', shell=True).decode("utf-8")
            if disk:
                os.system(f"sudo mount /dev/{disk} /media/smoothie")
                d = utility.get_current_time().split(" ")[0].split("-")
                config_final_path = f"./configFinal{self.__config.ROBOT_SN}/config_{d[0]}_{d[1]}_{d[2]}.txt"
                shutil.copyfile(f"/media/smoothie/config", config_final_path)
                os.system(f"sudo umount -l /dev/{disk}")
                shutil.chown(config_final_path, "violette", "violette")
            self.__socketio.emit(
                'apply_config', {'apply_done': True}, namespace='/server', broadcast=True)

    def __on_validate_log(self, data):
        print(self.__log)
        print(data)
        self.__log[data["key"]] = data["value"]
        print(self.__log)

    def __on_run_round_detection(self, data):
        if data["run_detection"]:
            self.__cameraCalibration.step_crop_picture()
            with open('./scene_center.jpg', 'rb') as f:
                image_data = f.read()
            self.__socketio.emit(
                'image', {'image_data': image_data}, namespace='/server', broadcast=True)

    def __on_run_target_detection(self, data):
        if data["run_detection"]:
            res = self.__cameraCalibration.offset_calibration_step_detect(
                self.__smoothie)
            with open('./target_detection.jpg', 'rb') as f:
                image_data = f.read()
            self.__socketio.emit('image', {
                                 'image_data': image_data, "res": res}, namespace='/server', broadcast=True)

    def __on_run_move_to_target(self, data):
        if data["run_move_to_target"]:
            self.__cameraCalibration.offset_calibration_step_move(
                self.__smoothie)
            self.__socketio.emit(
                'move', {'move': True}, namespace='/server', broadcast=True)

    def __on_move_step(self, data):
        if "x" in data:
            self.__offset_x += data["x"]
        if "y" in data:
            self.__offset_y += data["y"]

    def __on_x_y_dir(self, data):
        if "x" in data and self.__smoothie is not None:
            self.__smoothie.custom_move_for(
                X_F=self.__config.X_F_MAX, X=data["x"])
        if "y" in data and self.__smoothie is not None:
            self.__smoothie.custom_move_for(
                Y_F=self.__config.Y_F_MAX, Y=data["y"])
        if "a" in data and self.__smoothie is not None:
            self.__smoothie.custom_move_for(
                A_F=self.__config.A_F_MAX, A=data["a"])
        if "inv" in data and self.__smoothie is not None:
            translate_axis_grec = {"x": "alpha_dir_pin",
                                   "y": "beta_dir_pin", "a": "delta_dir_pin"}
            for axis, invert in data["inv"].items():
                if invert:
                    cmd = f"config-get sd {translate_axis_grec[axis.lower()]}"
                    # print(cmd)
                    self.__smoothie.get_connector().write(cmd)
                    response = self.__smoothie.get_connector().read_some()
                    matches = re.findall(
                        f"{translate_axis_grec[axis.lower()]} is set to (.*?)\n", response)
                    if matches:
                        value = matches[0][:-1]
                        if "!" in value:
                            value = value[:-1]
                        else:
                            value = value + "!"

                        """ ----- Put config in sd ----- """
                        cmd = f"config-set sd {translate_axis_grec[axis.lower()]} {value}"
                        self.__smoothie.get_connector().write(cmd)
                        response = self.__smoothie.get_connector().read_some()
                        """ ----- Put config in firm ----- """
                        self.__smoothie.get_connector().write(cmd.replace("sd", "firm"))
                        response = self.__smoothie.get_connector().read_some()

            # TODO: Reload config smoothie without reset
            # self.__smoothie.get_connector().write("reset")
            # response = self.__smoothie.get_connector().read_some()

    """ ---------------------------- Utils methodes ---------------------------- """

    def __reload_config(self):
        print("Reload config in new_deploy.py...")
        spec = importlib.util.spec_from_file_location(
            "config.name", "../config/config.py")
        self.__config = importlib.util.module_from_spec(spec)
        sys.modules["config.name"] = self.__config
        spec.loader.exec_module(self.__config)


def main():
    deployement = Deployement()
    try:
        deployement.run(host="0.0.0.0", port="80",
                        debug=True, use_reloader=False)
    finally:
        print("Closing deployement...")


if __name__ == "__main__":
    main()
