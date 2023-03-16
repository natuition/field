from notification import RobotSynthesisClient, RobotSynthesis

def main():
    with RobotSynthesisClient() as robot_synthesis_client:
        while True:
            status = int(input("New status (0-3): "))
            if status == 0:
                robot_synthesis_client.set_last_robot_synthesis(RobotSynthesis.OP)
            elif status == 1:
                robot_synthesis_client.set_last_robot_synthesis(RobotSynthesis.WORKING)
            elif status == 2:
                robot_synthesis_client.set_last_robot_synthesis(RobotSynthesis.HS)
            elif status == 3:
                robot_synthesis_client.set_last_robot_synthesis(RobotSynthesis.ANTI_THEFT)


if "__main__" == __name__:
    main()