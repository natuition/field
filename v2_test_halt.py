import adapters
from config import config

METERS_TO_MOVE = 3


def main():
    smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
    moving = False

    # ask for movement speed mode
    while True:
        speed_mode = input("Type l to set low speed, type h to set high speed: ")
        if speed_mode == "l":
            distance = -314 * METERS_TO_MOVE
            force = 1900
            break
        elif speed_mode == "h":
            distance = -17.4 * METERS_TO_MOVE
            force = 1100
            break
        else:
            print("Wrong speed mode, can be l or h")

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
                print("Moving forward for", METERS_TO_MOVE, "meters")
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
