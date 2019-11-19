from config import config
import connectors
import adapters


def main():
    print("Loading smoothie connector")
    smoothie = connectors.SmoothieConnector(config.SMOOTHIE_HOST)

    print("Turning wheels right")
    smoothie.write("G0 A-38 F1000")
    res = smoothie.read_some()

    if res != adapters.SmoothieAdapter.RESPONSE_OK:
        print("Couldn't turn wheels right, smoothie error occurred:", res)
        exit(1)

    print("Execution done")


if __name__ == '__main__':
    main()
