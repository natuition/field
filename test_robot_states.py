import notification


def main():
    with notification.RobotStateClient() as robot_synthesis_client:
        while True:
            status = int(input("New status (0-3): "))
            if status == 0:
                robot_synthesis_client.set_robot_state(notification.RobotStates.ENABLED)
            elif status == 1:
                robot_synthesis_client.set_robot_state(notification.RobotStates.WORKING)
            elif status == 2:
                robot_synthesis_client.set_robot_state(notification.RobotStates.OUT_OF_SERVICE)
            elif status == 3:
                robot_synthesis_client.set_robot_state(notification.RobotStates.ANTI_THEFT)


if "__main__" == __name__:
    main()
