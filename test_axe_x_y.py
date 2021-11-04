from config import config
import connectors
import time
import utility

def wait_for_actions_done(smoothie_connector):
    smoothie_connector.write("M400")
    return smoothie_connector.read_some()

def send_smoothie_order_without_wait(sm_con, g_code):
    sm_con.write(g_code)
    res = sm_con.read_some()

def send_smoothie_order_with_wait(sm_con, g_code):
    send_smoothie_order_without_wait(sm_con, g_code)
    wait_for_actions_done(sm_con)

def send_smoothie_order_with_wait_with_time(sm_con, g_code):
    t1 = time.time()
    send_smoothie_order_with_wait(sm_con, g_code)
    print(f"Time for execute cmd {g_code} : {time.time()-t1}.")

def main():
    ports = utility.get_smoothie_vesc_addresses()
    #sm_con = connectors.SmoothieV11SerialConnector(ports["smoothie"],config.SMOOTHIE_BAUDRATE)
    sm_con = connectors.SmoothieV11TelnetConnector("192.168.9.101")
    try:
        send_smoothie_order_without_wait(sm_con,"G91")

        while True:
            user_cmd = input("Press enter to run test, type anything to exit: ")
            if user_cmd != "":
                break
            for i in range(100) :    
                #go MIN YX
                send_smoothie_order_with_wait(sm_con,"G28 YX")

                #go Y265
                send_smoothie_order_with_wait_with_time(sm_con,"G0 Y188 F25000")
                send_smoothie_order_with_wait_with_time(sm_con,"G0 X456 F25000")
                send_smoothie_order_with_wait_with_time(sm_con,"G0 Y-188 F25000")
                send_smoothie_order_with_wait_with_time(sm_con,"G0 X-456 F25000")
                #go MIN YX
                send_smoothie_order_with_wait(sm_con,"G28 YX")
    finally:
        sm_con.disconnect()


if __name__ == '__main__':
    main()


