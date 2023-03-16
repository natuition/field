from notification import RobotSynthesisServer
from config import config
import time

def main():
    with RobotSynthesisServer() as robot_synthesis_server:
        robot_synthesis_server.wait()


if "__main__" == __name__:
    main()