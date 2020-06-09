from config import config
import adapters


def main():
    with adapters.VescAdapter(config.VESC_RPM, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                              config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE) as vesc_engine:
        rpm = int(input("Set RPM: "))
        moving_time = float(input("RPM = " + str(rpm) + "; set moving time (seconds; will start moving immediately): "))

        vesc_engine.set_rpm(rpm)
        vesc_engine.set_moving_time(moving_time)
        vesc_engine.start_moving()
        vesc_engine.wait_for_stop()
        vesc_engine.set_moving_time(config.VESC_MOVING_TIME)
        vesc_engine.set_rpm(config.VESC_RPM)


if __name__ == '__main__':
    main()
