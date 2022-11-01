from config import config
import adapters


def main():
    with adapters.VescAdapterV3(config.VESC_PORT, config.VESC_BAUDRATE, config.VESC_ALIVE_FREQ,
                              config.VESC_CHECK_FREQ, config.VESC_STOPPER_CHECK_FREQ) as vesc_engine:
        try:
            rpm = int(input("Set RPM: "))
            moving_time = float(input("Set moving time (seconds; will start moving immediately): "))

            vesc_engine.set_rpm(rpm, adapters.VescAdapterV3.PROPULSION_KEY)
            vesc_engine.set_time_to_move(moving_time, adapters.VescAdapterV3.PROPULSION_KEY)
            vesc_engine.start_moving(adapters.VescAdapterV3.PROPULSION_KEY)
            vesc_engine.wait_for_stop(adapters.VescAdapterV3.PROPULSION_KEY)
        except KeyboardInterrupt:
            print("Emergency stop (KB interrupt)")
            vesc_engine.stop_moving()
        finally:
            vesc_engine.set_time_to_move(config.VESC_MOVING_TIME, adapters.VescAdapterV3.PROPULSION_KEY)
            vesc_engine.set_rpm(config.VESC_RPM_SLOW, adapters.VescAdapterV3.PROPULSION_KEY)
            print("Done.")


if __name__ == '__main__':
    main()
