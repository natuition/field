from config import config
import adapters
import connectors


def main():
    print("Loading smoothie connector")
    smoothie = connectors.SmoothieConnector(config.SMOOTHIE_HOST)

    print("Moving forward")
    res = smoothie.write("G0 B48.9 F1000")

    if res != adapters.SmoothieAdapter.RESPONSE_OK:
        print("Couldn't move forward, smoothie error occurred:", res)
        exit(1)

    print("Execution done")


if __name__ == '__main__':
    main()
