from config import config
import connectors
import traceback


def wait_for_actions_done(smoothie_connector):
    smoothie_connector.write("M400")
    return smoothie_connector.read_some()


def main():
    sm_con = connectors.SmoothieV11TelnetConnector(config.SMOOTHIE_HOST)

    try:
        sm_con.write("G91")
        res = sm_con.read_some()
        print("G91 - ", res)

        while True:
            user_cmd = input("Press enter to extraction, type anything to exit: ")
            if user_cmd != "":
                break

            # cork down
            g_code = "G0 Z" + str(config.EXTRACTION_Z) + " F" + str(config.EXTRACTION_Z_F)
            print(g_code, " - ", end="")
            sm_con.write(g_code)
            res = sm_con.read_some()
            print(res)
            wait_for_actions_done(sm_con)

            # stub to set force for cork up
            g_code = "G0 Z0.1 F1300"
            print(g_code, " - ", end="")
            sm_con.write(g_code)
            res = sm_con.read_some()
            print(res)
            wait_for_actions_done(sm_con)

            # cork up
            g_code = "G28 Z" + str(config.CALIBRATION_DISTANCE if config.Z_AXIS_CALIBRATION_TO_MAX else -config.CALIBRATION_DISTANCE)
            print(g_code, " - ", end="")
            sm_con.write(g_code)
            res = sm_con.read_some()
            print(res)
            wait_for_actions_done(sm_con)
    except:
        print(traceback.format_exc())
    finally:
        sm_con.disconnect()
