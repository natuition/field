from config import config
import adapters
import time


def main():
    print("Loading smoothie adapter")
    smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)

    while True:
        print("Calibrating cork")
        smoothie.ext_calibrate_cork()
        smoothie.wait_for_all_actions_done()

        print("Aligning cork to center")
        res = smoothie.ext_align_cork_center(config.XY_F_MAX)
        smoothie.wait_for_all_actions_done()
        print(res)

        print("Extracting, cork down")
        # extraction, cork down
        res = smoothie.custom_move_for(F=1700, Z=-35)
        smoothie.wait_for_all_actions_done()
        print(res)

        # extraction, cork up
        print("Extracting, cork up")
        res = smoothie.ext_cork_up()
        smoothie.wait_for_all_actions_done()
        print(res)

        print("30 sec delay...")
        time.sleep(30)


if __name__ == '__main__':
    main()
