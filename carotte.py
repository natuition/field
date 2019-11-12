from config import config
import adapters


def main():
    print("Loading smoothie adapter")
    smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)

    print("Aligning cork to center")
    res = smoothie.ext_align_cork_center(config.XY_F_MAX)
    if res != smoothie.RESPONSE_OK:
        print("Couldn't align cork to center, smoothie error occurred:", res)
        exit(1)

    print("Extracting")
    # extraction, cork down
    res = smoothie.custom_move_for(config.Z_F_MAX, Z=-30)
    smoothie.wait_for_all_actions_done()
    if res != smoothie.RESPONSE_OK:
        print("Couldn't move the extractor down, smoothie error occurred:", res)
        exit(1)

    # extraction, cork up
    res = smoothie.ext_cork_up()
    smoothie.wait_for_all_actions_done()
    if res != smoothie.RESPONSE_OK:
        print("Couldn't move the extractor up, smoothie error occurred:", res)
        exit(1)

    print("Done!")


if __name__ == '__main__':
    main()
