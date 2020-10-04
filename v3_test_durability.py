"""
Script for hardware testing, does random movements and extractions.
"""

import adapters
from config import config
import utility
import datetime
import traceback
import random as rnd


COUNT_OF_ITERATION = 4000
COUNT_OF_EXTRACTION = 4
MOVE_FORWARD_FROM = 1
MOVE_FORWARD_TO = 3

SHIFT_MIN = 5
SHIFT_MAX = 40


def main():
    with adapters.SmoothieAdapter(config.SMOOTHIE_HOST) as smoothie:
        with adapters.VescAdapter(config.VESC_RPM_SLOW, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                                    config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE) as vesc:
            name = datetime.datetime.now().strftime("%d-%m-%Y %H:%M")
            with utility.Logger(name + '.txt') as logger:
                msg = 'Starting durability test...\n'
                logger.write(msg)
                print(msg)

                msg = 'RPM: {0}'.format(config.VESC_RPM_SLOW)
                logger.write(msg + "\n")
                print(msg)

                for i in range(COUNT_OF_ITERATION):
                    msg = 'Iteration {0}...'.format(i)
                    logger.write(msg + "\n")
                    print(msg)
                    try:
                        A_rand = rnd.randint(config.A_MIN, config.A_MAX)  # 1
                        msg = 'Start moving the steering wheels to {0}...'.format(A_rand)
                        logger.write(msg + "\n")
                        print(msg)

                        response = smoothie.custom_move_to(config.A_F_MAX, A=A_rand)
                        # if response != smoothie.RESPONSE_OK:
                        logger.write(response + '\n')
                        print(response)
                        smoothie.wait_for_all_actions_done()

                        moving_time_rand = rnd.uniform(MOVE_FORWARD_FROM, MOVE_FORWARD_TO)  # 2
                        msg = 'Start moving the robot forward, VESC_MOVING_TIME {0}...'.format(moving_time_rand)
                        logger.write(msg + '\n')
                        print(msg)

                        vesc.set_moving_time(moving_time_rand)
                        vesc.start_moving()
                        vesc.wait_for_stop()

                        msg = 'Stop moving the robot forward, VESC_MOVING_TIME {0}...'.format(moving_time_rand)
                        logger.write(msg + '\n')
                        print(msg)

                        for j in range(COUNT_OF_EXTRACTION):
                            msg = 'Go to the extraction position Y min...'
                            logger.write(msg + '\n')
                            print(msg)

                            response = smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2 / config.XY_COEFFICIENT_TO_MM,
                                                               Y=config.Y_MIN)
                            # if response != smoothie.RESPONSE_OK:
                            logger.write(response + '\n')
                            print(response)
                            smoothie.wait_for_all_actions_done()

                            point_rand = rnd.choice(config.IMAGE_CONTROL_POINTS_MAP)  # 3
                            msg = 'Start the corkscrew movement to the control point {0}...'.format(point_rand)
                            logger.write(msg + '\n')
                            print(msg)

                            response = smoothie.custom_move_for(config.XY_F_MAX, X=point_rand[2] / config.XY_COEFFICIENT_TO_MM,
                                                                Y=point_rand[3] / config.XY_COEFFICIENT_TO_MM)
                            # if response != smoothie.RESPONSE_OK:
                            logger.write(response + '\n')
                            print(response)
                            smoothie.wait_for_all_actions_done()

                            cur_coord = smoothie.get_adapter_current_coordinates()
                            rand_sm_x = rnd.randint(SHIFT_MIN, SHIFT_MAX)  # 4
                            rand_sm_y = rnd.randint(SHIFT_MIN, SHIFT_MAX)
                            error_msg_x = smoothie.validate_value(cur_coord['X'] * config.XY_COEFFICIENT_TO_MM,
                                                                  rand_sm_x * config.XY_COEFFICIENT_TO_MM,
                                                                  "X", config.X_MIN, config.X_MAX, "X_MIN", "X_MAX")
                            error_msg_y = smoothie.validate_value(cur_coord['Y'] * config.XY_COEFFICIENT_TO_MM,
                                                                  rand_sm_y * config.XY_COEFFICIENT_TO_MM,
                                                                  "Y", config.Y_MIN, config.Y_MAX, "Y_MIN", "Y_MAX")
                            if error_msg_x:
                                rand_sm_x = -rand_sm_x
                            if error_msg_y:
                                rand_sm_y = -rand_sm_y

                            msg = 'Start pointing the corkscrew by {0} mm along the X axis, and {1} mm along the Y axis...'.format(
                                rand_sm_x, rand_sm_y)
                            logger.write(msg + '\n')
                            print(msg)

                            response = smoothie.custom_move_for(config.XY_F_MAX, X=rand_sm_x, Y=rand_sm_y)
                            logger.write(response + '\n')
                            print(response)
                            smoothie.wait_for_all_actions_done()

                            msg = 'Starting extraction movements...'  # 5
                            logger.write(msg + '\n')
                            print(msg)

                            response = smoothie.custom_move_for(config.Z_F_EXTRACTION_DOWN, Z=config.EXTRACTION_Z)
                            print(response)
                            logger.write(response + "\n")
                            smoothie.wait_for_all_actions_done()

                            response = smoothie.ext_cork_up()
                            print(response)
                            logger.write(response + "\n")
                            smoothie.wait_for_all_actions_done()
                    except KeyboardInterrupt:
                        msg = "Keyboard interrupt. Exiting."
                        print(msg)
                        logger.write(msg + "\n")
                        exit(0)
                    except:
                        error = traceback.format_exc()
                        logger.write(error + "\n")
                        print(error)
                    else:
                        error = 'No software errors found'
                        logger.write(error + '\n')
                        print(error)

                msg = 'Done.'
                logger.write(msg + '\n')
                print(msg)


if __name__ == '__main__':
    main()
