import os
import glob
import datetime
import shutil
import pytz
import pwd
import grp

# load config, if failed - copy and load config backups until success or no more backups
def is_config_empty(config_full_path: str):
    with open(config_full_path, "r") as config_file:
        for line in config_file:
            if line not in ["", "\n"]:
                return False
    return True

def make_import(config_directory_path: str = "./config", config_backup_path : str = "./configBackup"):
    try:
        if not os.path.isfile(f"{config_directory_path}/config.py"):
            raise Exception("config file is not exist")

        if is_config_empty(f"{config_directory_path}/config.py"):
            raise Exception("config file is empty")

        from config import config
        print("Config.py file work good !")
    except KeyboardInterrupt:
        raise KeyboardInterrupt
    except Exception as exc:
        print(f"Failed to load current config.py! ({str(exc)})")

        # load config backups
        config_backups = [path for path in glob.glob(
            f"{config_backup_path}/*.py") if "config" in path]
        for i in range(len(config_backups)):
            ds = config_backups[i].split("_")[1:]  # date structure
            ds.extend(ds.pop(-1).split(":"))
            ds[-1] = ds[-1][:ds[-1].find(".")]
            config_backups[i] = [
                config_backups[i],
                datetime.datetime(
                    day=int(ds[0]),
                    month=int(ds[1]),
                    year=int(ds[2]),
                    hour=int(ds[3]),
                    minute=int(ds[4]),
                    second=int(ds[5])
                ).timestamp()
            ]
        # make last backups to be placed and used first
        config_backups.sort(key=lambda item: item[1], reverse=True)

        # try to find and set as current last valid config
        for config_backup in config_backups:
            try:
                try:
                    os.rename(
                        f"{config_directory_path}/config.py",
                        f"{config_directory_path}/ERROR_{datetime.datetime.now(pytz.timezone('Europe/Berlin')).strftime('%d-%m-%Y %H-%M-%S %f')}"
                        f"_config.py")
                except:
                    pass
                shutil.copy(config_backup[0], f"{config_directory_path}/config.py")
                uid = pwd.getpwnam("violette").pw_uid
                gid = grp.getgrnam("violette").gr_gid
                os.chown(f"{config_directory_path}/config.py", uid, gid)

                if is_config_empty(f"{config_directory_path}/config.py"):
                    raise Exception("config file is empty")

                from config import config
                print("Successfully loaded config:", config_backup[0])
                break
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except Exception as e:
                print(e)
                pass
        else:
            print(f"Couldn't find proper '{config_directory_path}/config.py' file and '{config_backup_path}' directories!")
            exit()