import adapters
from config import config


def ask_meters_multiplier():
    while True:
        try:
            meters_multiplier = float(input("Set distance meter multiplier (1 = 1m, 2 = 2m, ...): "))
            if meters_multiplier <= 0.0001:
                print("Multiplier should be > 0")
                continue
            return meters_multiplier
        except KeyboardInterrupt:
            exit()
        except Exception as e:
            print(e)


def ask_speed_mode(meters_multiplier):
    # ask for movement speed mode
    while True:
        speed_mode = input("Type l to set low speed, type h to set high speed: ")
        if speed_mode == "l":
            distance = -314 * meters_multiplier
            force = 1900
            return distance, force
        elif speed_mode == "h":
            distance = -17.4 * meters_multiplier
            force = 1100
            return distance, force
        else:
            print("Wrong speed mode, can be l or h")


def main():
    smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
    meters_multiplier = ask_meters_multiplier()
    distance, force = ask_speed_mode(meters_multiplier)
    moving = False

    # main loop
    while True:
        command = input("Hit enter to " + ("STOP" if moving else "MOVE") + ", type anything to exit: ")
        if command != "":
            if moving:
                print("Stopping movement...")
                smoothie.halt()
                smoothie.reset()
            print("Done.")
            break

        if not moving:
            response = smoothie.custom_move_for(force, B=distance)
            if response == smoothie.RESPONSE_OK:
                print("Moving forward for", meters_multiplier, "meters")
            else:
                print("Couldn't move forward, smoothie error occurred:", response)
                exit(1)
        else:
            smoothie.halt()
            smoothie.reset()
            print("Movement interrupted, standing")

        moving = not moving


if __name__ == '__main__':
    main()
