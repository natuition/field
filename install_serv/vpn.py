import sys
sys.path.append('../')
from config import config
import os
import shutil
from glob import glob

os.system(f"unzip -o ./vpn/{config.ROBOT_SN}.zip -d ./unzip")
shutil.copytree("./unzip/nix/etc/openvpn/natuition.com","/etc/openvpn/natuition.com")
certificat_conf_path = glob('./unzip/nix/etc/openvpn/natuition.com/*.conf')[0].replace("./unzip/nix","")
os.system(f"sudo ln -s {certificat_conf_path} /etc/openvpn/{config.ROBOT_SN.lower()}.conf")
os.system("sudo systemctl enable openvpn.service")
os.system("sudo systemctl restart openvpn.service")
shutil.rmtree('./unzip')
shutil.rmtree('./vpn')
