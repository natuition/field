from config import config
import connectors
import time
import utility

def send_smoothie_order_without_wait(sm_con, g_code):
    sm_con.write(g_code)
    res = sm_con.read_some()

def main():
    ports = utility.get_smoothie_vesc_addresses()
    sm_con = connectors.SmoothieV11SerialConnector(ports["smoothie"],config.SMOOTHIE_BAUDRATE)
    #sm_con = connectors.SmoothieV11TelnetConnector("192.168.9.101")
    try:
        send_smoothie_order_without_wait(sm_con,"G91")

        sm_con.write("G0 Z30 F1950")
        print(sm_con.get_serial().readline())
        sm_con.write("G0 Z-01 F1500")
        print(sm_con.get_serial().readline())
        sm_con.write("G28 Z-100")
        print(sm_con.get_serial().readline())

    finally:
        sm_con.disconnect()


if __name__ == '__main__':
    main()


