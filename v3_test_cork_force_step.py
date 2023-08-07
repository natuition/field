from config import config
import adapters
import utility
from utility import get_current_time

def main():
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

    with adapters.SmoothieAdapter(smoothie_address) as smoothie:
        with adapters.VescAdapterV4(vesc_address,
                                    config.VESC_BAUDRATE,
                                    config.VESC_ALIVE_FREQ,
                                    config.VESC_CHECK_FREQ,
                                    config.VESC_STOPPER_CHECK_FREQ) as vesc_engine:
            
            x, y = config.X_MIN, config.Y_MIN
            x_dir = 1
            nb_extract = 467
            step_size_x_y = 30

            logger = utility.Logger("./log_extraction_test_" + get_current_time() + ".txt", append_file=True)

            user_cmd = input("Press enter to start test, type anything to cast-off the test, type ctrl+x during the scipt to exit.")
            if user_cmd == "":
                try:
                    while True:
                        smoothie.custom_move_to(Y_F=config.Y_F_MAX, X_F=config.X_F_MAX, X=x, Y=y)
                        smoothie.wait_for_all_actions_done()

                        while True:
                            res = smoothie.custom_move_for(Z_F=config.Z_F_EXTRACTION_DOWN, Z=config.EXTRACTION_Z)
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                raise Exception(res)
                            res = smoothie.ext_cork_up()
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                raise Exception(res)
                            nb_extract+=1
                            print(nb_extract)
                            logger.write_and_flush(f"{nb_extract}\n")

                            x += step_size_x_y*x_dir

                            if x > config.X_MAX or x < config.X_MIN:
                                x_dir*=-1
                                y+=step_size_x_y
                                continue

                            if y > config.Y_MAX:
                                x, y = config.X_MIN, config.Y_MIN
                                break

                            smoothie.custom_move_to(Y_F=config.Y_F_MAX, X_F=config.X_F_MAX, X=x, Y=y)
                            smoothie.wait_for_all_actions_done()
                        
                        vesc_engine.set_time_to_move(config.STEP_FORWARD_TIME, vesc_engine.PROPULSION_KEY)
                        vesc_engine.set_target_rpm(
                            config.SI_SPEED_STEP_FORWARD * config.MULTIPLIER_SI_SPEED_TO_RPM,
                            vesc_engine.PROPULSION_KEY)
                        vesc_engine.start_moving(vesc_engine.PROPULSION_KEY)
                        vesc_engine.wait_for_stop(vesc_engine.PROPULSION_KEY)
                except KeyboardInterrupt:
                    print("Script stop by ctrl+c")
                finally:
                    res = smoothie.ext_cork_up()
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        raise Exception(res)


if __name__ == '__main__':
    main()
