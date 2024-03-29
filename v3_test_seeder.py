from config import config
import connectors
import traceback
import utility
import time

def wait_for_actions_done(smoothie_connector):
    smoothie_connector.write("M400")
    return smoothie_connector.read_some()

def execute_gcode_and_wait(g_code, sm_con):
    #print(g_code, " - ", end="")
    sm_con.write(g_code)
    res = sm_con.read_some()
    #print(res)
    wait_for_actions_done(sm_con)
    
def main():
    smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
    min_f_x_y = config.Y_F_MAX if (config.Y_F_MAX < config.X_F_MAX) else config.X_F_MAX
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

        sm_con.write(f"M280 S{config.SEEDER_CLOSE_COMMAND}")
        res = sm_con.read_some()
        print(f"M280 S{config.SEEDER_CLOSE_COMMAND} - ", res)

        # go home
        g_code = f"G28 XY F{min_f_x_y}"
        execute_gcode_and_wait(g_code, sm_con)

        # go middle
        g_code = "G0 X"+ str(config.X_MAX/2) + " Y"+ str(config.Y_MAX/2) +f" F{min_f_x_y}"
        execute_gcode_and_wait(g_code, sm_con)

        i = 1000

        while i>0:
            try:
                print(i)

                # go up
                g_code = "G0 Y"+ str(config.Y_MAX/2) +f" F{config.Y_F_MAX}"
                execute_gcode_and_wait(g_code, sm_con)

                # open seeder
                g_code = f"M280 S{config.SEEDER_OPEN_COMMAND}"
                execute_gcode_and_wait(g_code, sm_con)

                time.sleep(0.5)

                # close seeder
                g_code = f"M280 S{config.SEEDER_CLOSE_COMMAND}"
                execute_gcode_and_wait(g_code, sm_con)

                time.sleep(0.5)

                # shake
                g_code = f"G0 X-1Y-1 F{min_f_x_y}"
                execute_gcode_and_wait(g_code, sm_con)
                g_code = f"G0 X2Y2 F{min_f_x_y}"
                execute_gcode_and_wait(g_code, sm_con)
                g_code = f"G0 X-1Y-1 F{min_f_x_y}"
                execute_gcode_and_wait(g_code, sm_con)

                # go down
                g_code = "G0 Y-"+ str(config.Y_MAX/2) +f" F{config.Y_F_MAX}"
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
