import adapters
from config import config
import utility
import datetime
import cv2 as cv
import traceback

X_MOVEMENT = 50
Y_MOVEMENT = 50


class Self_Testing:
    """
    A module that tests the main modules of the robot
    """

    def __init__(self):
        self.all_tests = 14
        self.pass_tests = 0
        smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
        if "vesc" in smoothie_vesc_addr:
            vesc_address = smoothie_vesc_addr["vesc"]
        else:
            msg = "Couldn't get vesc's USB address!"
            print(msg)
            exit(1)
        if config.SMOOTHIE_BACKEND == 1:
            smoothie_address = config.SMOOTHIE_HOST
        else:
            if "smoothie" in smoothie_vesc_addr:
                smoothie_address = smoothie_vesc_addr["smoothie"]
            else:
                msg = "Couldn't get smoothie's USB address!"
                print(msg)
                exit(1)
        self.smoothie = adapters.SmoothieAdapter(smoothie_address)
        self.vesc = adapters.VescAdapter(config.VESC_RPM_SLOW, 10, config.VESC_ALIVE_FREQ,
                                         config.VESC_CHECK_FREQ, vesc_address, config.VESC_BAUDRATE)
        self.gps = adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP)
        self.camera = adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                                       config.CROP_H_TO, config.CV_ROTATE_CODE,
                                                       config.ISP_DIGITAL_GAIN_RANGE_FROM,
                                                       config.ISP_DIGITAL_GAIN_RANGE_TO,
                                                       config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                                       config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                                       config.AE_LOCK, config.CAMERA_W, config.CAMERA_H,
                                                       config.CAMERA_W,
                                                       config.CAMERA_H, config.CAMERA_FRAMERATE,
                                                       config.CAMERA_FLIP_METHOD)

        self.name = datetime.datetime.now().strftime("%d-%m-%Y %H:%M")
        self.logger = utility.Logger(self.name + '.txt')

    def test_cork_Z_up(self):
        """
        Function to check the movement of the corkscrew up the Z axis
        :return: response of the Smoothie
        """
        response = self.smoothie.ext_cork_up()
        return response

    def test_cork_Z_down(self):
        """
        Function to check the movement of the corkscrew down the Z axis
        :return: response of the Smoothie
        """
        response = self.smoothie.custom_move_for(Z_F=config.Z_F_EXTRACTION_DOWN, Z=config.EXTRACTION_Z)
        return response

    def test_cork_X_right(self):
        """
        Function to check the movement of the corkscrew 10 to the right the X axis
        :return: response of the Smoothie
        """
        response = self.smoothie.custom_move_for(X_F=config.X_F_MAX, X=X_MOVEMENT)
        return response

    def test_cork_X_left(self):
        """
        Function to check the movement of the corkscrew 10 to the left the X axis
        :return: response of the Smoothie
        """
        response = self.smoothie.custom_move_for(X_F=config.X_F_MAX, X=-X_MOVEMENT)
        return response

    def test_cork_Y_up(self):
        """
        Function to check the movement of the corkscrew 10 to the up the Y axis
        :return: response of the Smoothie
        """
        response = self.smoothie.custom_move_for(Y_F=config.Y_F_MAX, Y=Y_MOVEMENT)
        return response

    def test_cork_Y_down(self):
        """
        Function to check the movement of the corkscrew 10 to the down the Y axis
        :return: response of the Smoothie
        """
        response = self.smoothie.custom_move_for(Y_F=config.Y_F_MAX, Y=-Y_MOVEMENT)
        return response

    def test_steering_wheels_right(self):
        """
        Function to check the moving the steering wheels to the right
        :return: response of the Smoothie
        """
        response = self.smoothie.custom_move_for(A_F=config.A_F_MAX, A=config.A_MIN)
        return response

    def test_steering_wheels_left(self):
        """
        Function to check the moving the steering wheels to the left
        :return: response of the Smoothie
        """
        response = self.smoothie.custom_move_for(A_F=config.A_F_MAX, A=config.A_MAX)
        return response

    def test_steering_wheels_center(self):
        """
        Function to check the moving the steering wheels to the center
        :return: response of the Smoothie
        """
        response = self.smoothie.custom_move_to(A_F=config.A_F_MAX, A=0)
        return response

    def test_motion_wheels_forward(self):
        """
        Function to check the moving the steering wheels to the center
        :return: response of the Smoothie
        """
        self.vesc.start_moving()
        self.vesc.wait_for_stop()

    def test_motion_wheels_back(self):
        """
        Function to check the moving the robot forward
        :return: response of the Smoothie
        """
        self.vesc.set_rpm(-config.VESC_RPM_SLOW)
        self.vesc.start_moving()
        self.vesc.wait_for_stop()

    def test_camera(self):
        """
        Function to check the camera
        :return: image
        """
        image = self.camera.get_image()
        return image

    def test_navigation(self):
        """
        Function to check the moving the robot back
        :return: response of the Smoothie
        """
        position = self.gps.get_last_position()
        return position

    def run(self):
        """
        Main function of the self-test module
        :return: returns a report on the verification process to the console and saves the results to a log file
        """
        msg = 'Starting self-test...\n'
        self.logger.write(msg)
        print(msg)

        response = None
        key = None
        error = None
        msg = 'Centering the corkscrew...'
        self.logger.write(msg + '\n')
        print(msg)
        try:
            response = self.smoothie.ext_align_cork_center(config.XY_F_MAX)
            self.smoothie.wait_for_all_actions_done()
            print(response)
            self.logger.write(response + '\n')
            msg = 'Has the corkscrew been centered? (y/n)'
            self.logger.write(msg + '\n')
            key = input(msg + '\n')
            self.logger.write(key + '\n')
        except:
            error = traceback.format_exc()
            print(error)
        else:
            error = 'No software errors found'
            print(error)
        finally:
            self.logger.write(error + '\n')
            if response == self.smoothie.RESPONSE_OK and key == 'y':
                msg = 'No technical errors found'
                self.logger.write(msg + '\n')
                print(msg)
                self.pass_tests += 1

        response = None
        key = None
        error = None
        msg = 'Start the corkscrew movement down the Z axis ' + str(config.Z_F_EXTRACTION_DOWN)
        self.logger.write(msg + '\n')
        print(msg)
        try:
            response = self.test_cork_Z_down()
            self.smoothie.wait_for_all_actions_done()
            print(response)
            self.logger.write(response + '\n')
            msg = 'Was there corkscrew movement a downward Z-axis corkscrew?? (y/n)'
            self.logger.write(msg + '\n')
            key = input(msg)
            self.logger.write(key + '\n')
        except:
            error = traceback.format_exc()
            print(error)
        else:
            error = 'No software errors found'
            print(error)
        finally:
            self.logger.write(error + '\n')
            if response == self.smoothie.RESPONSE_OK and key == 'y':
                msg = 'No technical errors found'
                self.logger.write(msg + '\n')
                print(msg)
                self.pass_tests += 1

        response = None
        key = None
        error = None
        msg = 'Start the corkscrew movement up the Z axis...'
        self.logger.write(msg + '\n')
        print(msg)
        try:
            response = self.test_cork_Z_up()
            self.smoothie.wait_for_all_actions_done()
            print(response)
            self.logger.write(response + '\n')
            msg = 'Was there corkscrew movement an upward Z-axis corkscrew? (y/n)'
            self.logger.write(msg + '\n')
            key = input(msg)
            self.logger.write(key + '\n')
        except:
            error = traceback.format_exc()
            print(error)
        else:
            error = 'No software errors found'
            print(error)
        finally:
            self.logger.write(error + '\n')
            if response == self.smoothie.RESPONSE_OK and key == 'y':
                msg = 'No technical errors found'
                self.logger.write(msg + '\n')
                print(msg)
                self.pass_tests += 1

        if response == self.smoothie.RESPONSE_OK and key == 'y':

            response = None
            key = None
            error = None
            msg = 'Start the corkscrew movement to the right along the X axis ' + str(X_MOVEMENT)
            self.logger.write(msg + '\n')
            print(msg)
            try:
                response = self.test_cork_X_right()
                self.smoothie.wait_for_all_actions_done()
                print(response)
                self.logger.write(response + '\n')
                msg = 'Was the corkscrew movement to the right on the X axis done? (y/n)'
                self.logger.write(msg + '\n')
                key = input(msg)
                self.logger.write(key + '\n')
            except:
                error = traceback.format_exc()
                print(error)
            else:
                error = 'No software errors found'
                print(error)
            finally:
                self.logger.write(error + '\n')
                if response == self.smoothie.RESPONSE_OK and key == 'y':
                    msg = 'No technical errors found'
                    self.logger.write(msg + '\n')
                    print(msg)
                    self.pass_tests += 1

            response = None
            key = None
            error = None
            msg = 'Start the corkscrew movement to to the left along the X axis ' + str(-X_MOVEMENT)
            self.logger.write(msg + '\n')
            print(msg)
            try:
                response = self.test_cork_X_left()
                self.smoothie.wait_for_all_actions_done()
                print(response)
                self.logger.write(response + '\n')
                msg = 'Was the corkscrew movement to the left on the X axis done? (y/n)'
                self.logger.write(msg + '\n')
                key = input(msg)
                self.logger.write(key + '\n')
            except:
                error = traceback.format_exc()
                print(error)
            else:
                error = 'No software errors found'
                print(error)
            finally:
                self.logger.write(error + '\n')
                if response == self.smoothie.RESPONSE_OK and key == 'y':
                    msg = 'No technical errors found'
                    self.logger.write(msg + '\n')
                    print(msg)
                    self.pass_tests += 1

            response = None
            key = None
            error = None
            msg = 'Start the corkscrew movement up along the Y axis ' + str(Y_MOVEMENT)
            self.logger.write(msg + '\n')
            print(msg)
            try:
                response = self.test_cork_Y_up()
                self.smoothie.wait_for_all_actions_done()
                print(response)
                self.logger.write(response + '\n')
                msg = 'Was the corkscrew movement up on the Y axis done? (y/n)'
                self.logger.write(msg + '\n')
                key = input(msg)
                self.logger.write(key + '\n')
            except:
                error = traceback.format_exc()
                print(error)
            else:
                error = 'No software errors found'
                print(error)
            finally:
                self.logger.write(error + '\n')
                if response == self.smoothie.RESPONSE_OK and key == 'y':
                    msg = 'No technical errors found'
                    self.logger.write(msg + '\n')
                    print(msg)
                    self.pass_tests += 1

            response = None
            key = None
            error = None
            msg = 'Start the corkscrew movement down along the Y axis ' + str(-Y_MOVEMENT)
            self.logger.write(msg + '\n')
            print(msg)
            try:
                response = self.test_cork_Y_down()
                self.smoothie.wait_for_all_actions_done()
                print(response)
                self.logger.write(response + '\n')
                msg = 'Was the corkscrew movement down on the Y axis done? (y/n)'
                self.logger.write(msg + '\n')
                key = input(msg)
                self.logger.write(key + '\n')
            except:
                error = traceback.format_exc()
                print(error)
            else:
                error = 'No software errors found'
                print(error)
            finally:
                self.logger.write(error + '\n')
                if response == self.smoothie.RESPONSE_OK and key == 'y':
                    msg = 'No technical errors found'
                    self.logger.write(msg + '\n')
                    print(msg)
                    self.pass_tests += 1

            response = None
            key = None
            error = None
            msg = 'Start moving the robot forward (-2800)...'
            self.logger.write(msg + '\n')
            print(msg)
            try:
                self.test_motion_wheels_forward()
                msg = 'Did the robot move forward? (y/n)'
                self.logger.write(msg + '\n')
                key = input(msg)
                self.logger.write(key + '\n')
            except:
                error = traceback.format_exc()
                print(error)
            else:
                error = 'No software errors found'
                print(error)
            finally:
                self.logger.write(error + '\n')
                if response == self.smoothie.RESPONSE_OK and key == 'y':
                    msg = 'No technical errors found'
                    self.logger.write(msg + '\n')
                    print(msg)
                    self.pass_tests += 1

            response = None
            key = None
            error = None
            msg = 'Start moving the robot back (2800)...'
            self.logger.write(msg + '\n')
            print(msg)
            try:
                self.test_motion_wheels_back()
                msg = 'Did the robot move backward? (y/n)'
                self.logger.write(msg + '\n')
                key = input(msg)
                self.logger.write(key + '\n')
            except:
                error = traceback.format_exc()
                print(error)
            else:
                error = 'No software errors found'
                print(error)
            finally:
                self.logger.write(error + '\n')
                if response == self.smoothie.RESPONSE_OK and key == 'y':
                    msg = 'No technical errors found'
                    self.logger.write(msg + '\n')
                    print(msg)
                    self.pass_tests += 1

        response = None
        key = None
        error = None
        msg = 'Start moving the steering wheels to the right...'
        self.logger.write(msg + '\n')
        print(msg)
        try:
            response = self.test_steering_wheels_right()
            self.smoothie.wait_for_all_actions_done()
            print(response)
            self.logger.write(response + '\n')
            msg = 'Did the steering wheels move to the right? (y/n)'
            self.logger.write(msg + '\n')
            key = input(msg)
            self.logger.write(key + '\n')
        except:
            error = traceback.format_exc()
            print(error)
        else:
            error = 'No software errors found'
            print(error)
        finally:
            self.logger.write(error + '\n')
            if response == self.smoothie.RESPONSE_OK and key == 'y':
                msg = 'No technical errors found'
                self.logger.write(msg + '\n')
                print(msg)
                self.pass_tests += 1

        response = None
        key = None
        error = None
        msg = 'Start moving the steering wheels to the left...'
        self.logger.write(msg + '\n')
        print(msg)
        try:
            response = self.test_steering_wheels_left()
            self.smoothie.wait_for_all_actions_done()
            print(response)
            self.logger.write(response + '\n')
            msg = 'Did the steering wheels move to the left? (y/n)'
            self.logger.write(msg + '\n')
            key = input(msg)
            self.logger.write(key + '\n')
        except:
            error = traceback.format_exc()
            print(error)
        else:
            error = 'No software errors found'
            print(error)
        finally:
            self.logger.write(error + '\n')
            if response == self.smoothie.RESPONSE_OK and key == 'y':
                msg = 'No technical errors found'
                self.logger.write(msg + '\n')
                print(msg)
                self.pass_tests += 1

        response = None
        key = None
        error = None
        msg = 'Start moving the steering wheels to the center...'
        self.logger.write(msg + '\n')
        print(msg)
        try:
            response = self.test_steering_wheels_center()
            self.smoothie.wait_for_all_actions_done()
            print(response)
            self.logger.write(response + '\n')
            msg = 'Did the steering wheels move to the center? (y/n)'
            self.logger.write(msg + '\n')
            key = input(msg)
            self.logger.write(key + '\n')
        except:
            error = traceback.format_exc()
            print(error)
        else:
            error = 'No software errors found'
            print(error)
        finally:
            self.logger.write(error + '\n')
            if response == self.smoothie.RESPONSE_OK and key == 'y':
                msg = 'No technical errors found'
                self.logger.write(msg + '\n')
                print(msg)
                self.pass_tests += 1

        response = None
        key = None
        error = None
        msg = 'Start checking camera...'
        self.logger.write(msg + '\n')
        print(msg)
        try:
            image = self.test_camera()
            response = self.smoothie.RESPONSE_OK
            print(response)
            self.logger.write(response + '\n')
            cv.imwrite(self.name + ".jpg", image)
            msg = 'Has the image been saved? (y/n)'
            self.logger.write(msg + '\n')
            key = input(msg)
            self.logger.write(key + '\n')
            error = 'No software errors found'
            print(error)
        except:
            error = traceback.format_exc()
            print(error)
        finally:
            self.logger.write(error + '\n')
            if response == self.smoothie.RESPONSE_OK and key == 'y':
                msg = 'No technical errors found'
                self.logger.write(msg + '\n')
                print(msg)
                self.pass_tests += 1

        response = None
        key = None
        error = None
        msg = 'Start checking navigation...'
        self.logger.write(msg + '\n')
        print(msg)
        try:
            position = self.test_navigation()
            print(position)
            self.logger.write(str(position) + '\n')
            msg = 'Has at least one GPS point been received? (y/n)'
            self.logger.write(msg + '\n')
            key = input(msg)
            self.logger.write(key + '\n')
        except:
            error = traceback.format_exc()
            print(error)
        else:
            error = 'No software errors found'
            print(error)
        finally:
            self.logger.write(error + '\n')
            if key == 'y':
                msg = 'No technical errors found'
                self.logger.write(msg + '\n')
                print(msg)
                self.pass_tests += 1

            msg = str(self.pass_tests) + '/' + str(self.all_tests) + ' test(s) are passed'
            self.logger.write(msg + '\n')
            print(msg)
            msg = 'Done.'
            self.logger.write(msg + '\n')
            print(msg)

        self.logger.close()
        self.smoothie.disconnect()
        self.gps.disconnect()
        self.camera.release()
        self.vesc.disconnect()


if __name__ == "__main__":
    self_test = Self_Testing()
    self_test.run()
