from config import config
import adapters


def main():
    print("Loading smoothie adapter")
    smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)

    print("Moving forward")
    res = smoothie.nav_move_forward(48.9, 1000)
    if res != smoothie.RESPONSE_OK:
        print("Couldn't move forward, smoothie error occurred:", res)
        exit(1)

    print("Done!")


if __name__ == '__main__':
    main()
