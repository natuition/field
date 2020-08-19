from config import config
import adapters


def main():
    with adapters.VescAdapter(config.VESC_RPM_SLOW, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                              config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE) as vesc_engine:
        try:
            rpm = int(input("Set RPM: "))
            moving_time = float(input("Set moving time (seconds; will start moving immediately): "))

            vesc_engine.set_rpm(rpm)
            vesc_engine.set_moving_time(moving_time)
            vesc_engine.start_moving()
            vesc_engine.wait_for_stop()
        except KeyboardInterrupt:
            print("Emergency stop (KB interrupt)")
            vesc_engine.stop_moving()
        finally:
            vesc_engine.set_moving_time(config.VESC_MOVING_TIME)
            vesc_engine.set_rpm(config.VESC_RPM_SLOW)
            print("Done.")


if __name__ == '__main__':
    main()
