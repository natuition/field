from config import config
import connectors
import traceback
import utility
import time

def wait_for_actions_done(smoothie_connector):
    smoothie_connector.write("M400")
    return smoothie_connector.read_some()

def execute_gcode_and_wait(g_code, sm_con):
    print(g_code, " - ", end="")
    sm_con.write(g_code)
    res = sm_con.read_some()
    print(res)
    wait_for_actions_done(sm_con)
    
def main():
    smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
    if config.SMOOTHIE_BACKEND == 1:
        smoothie_address = config.SMOOTHIE_HOST
    else:
        if "smoothie" in smoothie_vesc_addr:
            smoothie_address = smoothie_vesc_addr["smoothie"]
        else:
            msg = "Couldn't get smoothie's USB address!"
            print(msg)
            exit(1)
    sm_con = connectors.SmoothieV11SerialConnector(smoothie_address, config.SMOOTHIE_BAUDRATE)

    try:
        sm_con.write("G91")
        res = sm_con.read_some()
        print("G91 - ", res)

        sm_con.write("M280 S2")
        res = sm_con.read_some()
        print("M280 S2 - ", res)

        # go home
        g_code = "G28 XY F30000"
        execute_gcode_and_wait(g_code, sm_con)

        # go middle
        g_code = "G0 X"+ str(config.X_MAX/2) +" F30000"
        execute_gcode_and_wait(g_code, sm_con)

        i = 50

        while i>0:
            try:
                print(i)

                user_cmd = input("Press enter to make seeder cycle")
                if user_cmd != "":
                    break

                # go up
                g_code = "G0 Y"+ str(config.Y_MAX/2) +" F30000"
                execute_gcode_and_wait(g_code, sm_con)

                # open seeder
                g_code = "M280 S5.5"
                execute_gcode_and_wait(g_code, sm_con)

                time.sleep(0.5)

                # close seeder
                g_code = "M280 S2"
                execute_gcode_and_wait(g_code, sm_con)

                time.sleep(0.5)

                # shake
                g_code = "G0 X-1Y-1 F30000"
                execute_gcode_and_wait(g_code, sm_con)
                g_code = "G0 X2Y2 F30000"
                execute_gcode_and_wait(g_code, sm_con)
                g_code = "G0 X-1Y-1 F30000"
                execute_gcode_and_wait(g_code, sm_con)

                # go down
                g_code = "G0 Y-"+ str(config.Y_MAX/2) +" F30000"
                execute_gcode_and_wait(g_code, sm_con)

                i-=1;

            except KeyboardInterrupt:
                break;
    except:
        print(traceback.format_exc())
    finally:
        sm_con.disconnect()


if __name__ == '__main__':
    main()
