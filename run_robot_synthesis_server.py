from notification import RobotStateServer
from config import config
import time

def main():
    with RobotStateServer() as robot_synthesis_server:
        robot_synthesis_server.wait()


if "__main__" == __name__:
    main()