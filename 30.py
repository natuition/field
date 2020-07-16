from config import config
import adapters
import connectors


def main():
    print("Loading smoothie connector")
    smoothie = connectors.SmoothieV11TelnetConnector(config.SMOOTHIE_HOST)

    print("Moving forward")
    smoothie.write("G0 B16.3 F1000")
    res = smoothie.read_some()

    if res != adapters.SmoothieAdapter.RESPONSE_OK:
        print("Couldn't move forward, smoothie error occurred:", res)
        exit(1)

    print("Execution done")


if __name__ == '__main__':
    main()
