import utility
from subprocess import Popen,DEVNULL
from optparse import OptionParser
from config import config

if __name__ == '__main__':
    usage="ser2net.py start|stop"
    parser=OptionParser(version=str(1.0), usage=usage)
    (_, args) = parser.parse_args()
    if len(args) == 1:
        if args[0].lower() == "start":
            if config.SER2NET_CONNECT_GPS_PORT:
                ubx_addr = config.GPS_PORT
                baudrate = config.GPS_BAUDRATE
            else:
                ubx_addr = utility.get_ublox_address()
                baudrate = config.NTRIP_OUTPUT_BAUDRATE
            cmd = ["sudo","ser2net","-C",f"6000:raw:600:{ubx_addr}:{baudrate} NONE 1STOPBIT 8DATABITS XONXOFF LOCAL -RTSCTS"]
            p = Popen(cmd)
            print(f"Sofware {cmd[1]} are start.")
        elif args[0].lower() == "stop":
            user_cmd = input("Kill sofware at port 6000/tcp. Press enter to validate, type anything to refuse.")
            if user_cmd == "":
                cmd = ["sudo","fuser","-k","6000/tcp"]
                p = Popen(cmd,stdout=DEVNULL, stderr=DEVNULL)
        else:
            print(usage)
    else:
        print(usage)
