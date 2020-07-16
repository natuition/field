from config import config
import connectors
import adapters


def main():
    print("Loading smoothie connector")
    smoothie = connectors.SmoothieV11TelnetConnector(config.SMOOTHIE_HOST)

    print("Turning wheels left")
    smoothie.write("G0 A38 F1000")
    res = smoothie.read_some()

    if res != adapters.SmoothieAdapter.RESPONSE_OK:
        print("Couldn't turn wheels left, smoothie error occurred:", res)
        exit(1)

    print("Execution done")


if __name__ == '__main__':
    main()
