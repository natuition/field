from config import config
from adapters import VescAdapterV4
from utility import Logger

def main():        
    logger = Logger("/tmp/vesc_v4_forward.log")
    with VescAdapterV4("/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00", 
                        config.VESC_BAUDRATE, 
                        config.VESC_ALIVE_FREQ,
                        config.VESC_CHECK_FREQ, 
                        config.VESC_STOPPER_CHECK_FREQ, 
                        logger) as vesc_engine:
        try:
            rpm = config.SI_SPEED_FAST * config.MULTIPLIER_SI_SPEED_TO_RPM  #int(input("Set RPM: "))
            input_str = input("Set moving time (seconds; will start moving immediately, or inf): ")
            moving_time = config.VESC_MOVING_TIME
            if input_str.lower() != "inf":
                moving_time = float(input_str)
            vesc_engine.set_target_rpm(rpm, vesc_engine.PROPULSION_KEY)
            vesc_engine.set_time_to_move(moving_time, vesc_engine.PROPULSION_KEY)
            vesc_engine.start_moving(vesc_engine.PROPULSION_KEY, True)
            vesc_engine.wait_for_stop(vesc_engine.PROPULSION_KEY)
        except KeyboardInterrupt:
            print("Emergency stop (KB interrupt)")
            vesc_engine.set_time_to_move(config.VESC_MOVING_TIME, vesc_engine.PROPULSION_KEY)
            vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY, True)
        finally:
            vesc_engine.set_time_to_move(config.VESC_MOVING_TIME, vesc_engine.PROPULSION_KEY)
            vesc_engine.stop_moving(vesc_engine.PROPULSION_KEY, True)
            print("Done.")


if __name__ == '__main__':
    main()
