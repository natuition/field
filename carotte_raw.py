from config import config
import connectors


def main():
    print("Loading smoothie connector")
    smoothie = connectors.SmoothieConnector(config.SMOOTHIE_HOST)

    print("Calibrating Z")
    smoothie.write("G28 Z1000")
    print(smoothie.read_some())

    print("Extracting")
    smoothie.write("G0 Z-30 F1000")
    print(smoothie.read_some())

    print("Picking cork up")
    smoothie.write("G28 Z1000")
    print(smoothie.read_some())

    print("Done!")


if __name__ == '__main__':
    main()
