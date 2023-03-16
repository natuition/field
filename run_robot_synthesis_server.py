from notification import RobotSynthesisServer
from config import config
import time

def main():
    with RobotSynthesisServer() as robot_synthesis_server:
        while True:
            time.sleep(1)


if "__main__" == __name__:
    main()