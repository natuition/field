from config import config
import adapters


def main():
    print("Loading smoothie adapter")
    smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)

    print("Turning wheels right")
    res = smoothie.custom_move_for(1000, A=38)
    if res != smoothie.RESPONSE_OK:
        print("Couldn't turn wheels right, smoothie error occurred:", res)
        exit(1)

    print("Done!")


if __name__ == '__main__':
    main()
