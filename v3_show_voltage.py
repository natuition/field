import adapters
from config import config


def main():
    report_field_names = ['temp_fet_filtered', 'temp_motor_filtered', 'avg_motor_current',
                          'avg_input_current', 'rpm', 'input_voltage']
    print("Loading vesc...")
    with adapters.VescAdapter(config.VESC_RPM_SLOW, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                              config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE) as vesc:
        voltage = vesc.get_sensors_data(report_field_names)["input_voltage"]
        print(voltage)


if __name__ == '__main__':
    main()
