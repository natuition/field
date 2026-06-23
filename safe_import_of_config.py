import os
import glob
import datetime
import shutil
import pytz
import pwd
import grp
import importlib

def is_config_empty(config_full_path: str):
    with open(config_full_path, "r") as config_file:
        for line in config_file:
            if line not in ["", "\n"]:
                return False
    return True

def make_import(config_directory_path: str = "config",
                config_backup_path: str = "configBackup"):
    
    if not os.path.isfile(f"{config_directory_path}/config.py"):
        config_directory_path = os.path.join("..", config_directory_path)
        config_backup_path = os.path.join("..", config_backup_path)

    try:
        if not os.path.isfile(f"{config_directory_path}/config.py"):
            raise Exception("config file doesn't exist")

        if is_config_empty(f"{config_directory_path}/config.py"):
            raise Exception("config file is empty")

        # import dynamique
        config_module = importlib.import_module("config.config")

        print("Config.py file works good !")

        return config_module

    except KeyboardInterrupt:
        raise

    except Exception as exc:
        print(f"Failed to load current config.py ! ({str(exc)})")

        config_backups = [
            path for path in glob.glob(f"{config_backup_path}/*.py")
            if "config" in path
        ]

        for i in range(len(config_backups)):
            ds = config_backups[i].split("_")[1:]
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

        config_backups.sort(key=lambda item: item[1], reverse=True)

        for config_backup in config_backups:
            try:
                try:
                    os.rename(
                        f"{config_directory_path}/config.py",
                        f"{config_directory_path}/ERROR_"
                        f"{datetime.datetime.now(pytz.timezone('Europe/Berlin')).strftime('%d-%m-%Y_%H-%M-%S_%f')}"
                        f"_config.py"
                    )
                except:
                    pass

                shutil.copy(
                    config_backup[0],
                    f"{config_directory_path}/config.py"
                )

                uid = pwd.getpwnam("violette").pw_uid
                gid = grp.getgrnam("violette").gr_gid

                os.chown(
                    f"{config_directory_path}/config.py",
                    uid,
                    gid
                )

                if is_config_empty(f"{config_directory_path}/config.py"):
                    raise Exception("config file is empty")

                # IMPORTANT :
                importlib.invalidate_caches()

                config_module = importlib.import_module("config.config")

                # force reload
                config_module = importlib.reload(config_module)

                print("Successfully loaded config:", config_backup[0])

                return config_module

            except KeyboardInterrupt:
                raise

            except Exception as e:
                print(e)

        print(
            f"Couldn't find proper "
            f"'{config_directory_path}/config.py' "
            f"file and '{config_backup_path}' directories!"
        )
        exit()


config = make_import("./config", "./configBackup")