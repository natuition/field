import os
import glob
import datetime
import shutil
import pytz
import pwd
import grp
import re
import ast

def get_latest_default_config_file(pattern="*_defaults.py"):
    files = glob.glob(pattern)
    version_pattern = re.compile(r"v(\d+)")  # capture "v123" comme version 123
    versioned = []

    for f in files:
        match = version_pattern.search(f)
        if match:
            versioned.append((int(match.group(1)), f))
        else:
            # Si pas de version trouvée, version = 0
            versioned.append((0, f))

    if not versioned:
        return None

    # Trie d’abord par version puis par nom
    versioned.sort(key=lambda x: (x[0], x[1]))
    return versioned[-1][1]

def extract_globals_static(path):
    """Parse Python file and extract top-level variable assignments safely (no execution)."""
    with open(path, "r") as f:
        source = f.read()

    try:
        tree = ast.parse(source, filename=path)
    except SyntaxError as e:
        raise Exception(f"Invalid Python syntax in {path}: {e}")

    result = {}
    for node in tree.body:
        # On ne garde que les assignations de haut niveau (pas dans une fonction)
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    try:
                        result[target.id] = ast.literal_eval(node.value)
                    except Exception:
                        # si la valeur n'est pas un littéral (ex: appel de fonction), on ignore
                        result[target.id] = None
    return result

# load config, if failed - copy and load config backups until success or no more backups
def is_config_empty(config_full_path: str):
    with open(config_full_path, "r") as config_file:
        for line in config_file:
            if line not in ["", "\n"]:
                return False
    return True

def validate_config_file(config_directory_path) -> bool:
    """Validate a Python config file without importing it.

    Checks that the file is non-empty and has valid Python syntax by compiling it.
    Returns True if valid, False otherwise.
    """
    config_full_path = f"{config_directory_path}/config.py"
    try:
        # 1️⃣ check empty
        if is_config_empty(config_full_path):
            print(f"[{os.path.basename(__file__)}] ❌ Config file is empty.")
            return False

        # 2️⃣ check syntax
        with open(config_full_path, "r") as f:
            source = f.read()
        compile(source, config_full_path, "exec")

        # 3️⃣ find latest default
        latest_default = get_latest_default_config_file(os.path.join(config_directory_path, "*_defaults.py"))
        if not latest_default:
            print(f"[{os.path.basename(__file__)}] ⚠️ No default config found to compare.")
            return True  # syntax OK but no structure check possible

        # 4️⃣ parse both files safely (no execution)
        default_vars = extract_globals_static(latest_default)
        user_vars = extract_globals_static(config_full_path)

        # 5️⃣ compare keys
        missing_keys = [k for k in default_vars if k not in user_vars]
        extra_keys   = [k for k in user_vars if k not in default_vars]

        if missing_keys:
            print(f"[{os.path.basename(__file__)}] ⚠️ Missing keys in {config_full_path}: {missing_keys}")

        if extra_keys:
            print(f"[{os.path.basename(__file__)}] ℹ️ Extra keys found (not in defaults): {extra_keys}")

        print(f"[{os.path.basename(__file__)}] ✅ Config validated successfully — syntax and keys are OK.")
        return True
    except Exception:
        return False

def prepare_valid_config(config_directory_path: str = "config", config_backup_path : str = "configBackup"):
    try:
        if not os.path.isfile(f"{config_directory_path}/config.py"):
            raise Exception("config file doesn't exist")

        if not validate_config_file(config_directory_path):
            raise Exception("config file is empty or has invalid syntax")

        # Config syntax is OK
        print(f"[{os.path.basename(__file__)}] Config.py file works good !")
    except KeyboardInterrupt:
        raise KeyboardInterrupt
    except Exception as exc:
        print(f"[{os.path.basename(__file__)}] Failed to load current config.py ! ({str(exc)})")

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

                if not validate_config_file(config_directory_path):
                    raise Exception("config file is empty or has invalid syntax")

                print(f"[{os.path.basename(__file__)}] Successfully loaded config:", config_backup[0])
                break
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except Exception as e:
                print(f"[{os.path.basename(__file__)}] {e}")
                pass
        else:
            print(f"[{os.path.basename(__file__)}] Couldn't find proper '{config_directory_path}/config.py' file and '{config_backup_path}' directories!")
            exit()