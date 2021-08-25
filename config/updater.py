import glob
import re
import os

version_defaults_file = sorted([(int(m.group(1)),m.string) for m in (re.compile(".*v([0-9]*).*").match(line) for line in glob.glob("*_defaults.py"))], key=lambda x: (x[0], x[1]))

module_default_config = __import__(version_defaults_file[-1][1][:-3])
default_config_vars = {k: v for k, v in module_default_config.__dict__.items() if not k.startswith('_')}
default_config = type("default_config", (object, ), default_config_vars)

module_config = __import__("config")
config_vars = {k: v for k, v in module_config.__dict__.items() if not k.startswith('_')}
config = type("config", (object, ), config_vars)

changed_vars = dict()
for vars, value in default_config_vars.items():
    if vars in config_vars:
        if value == config_vars[vars]:
            continue
        if vars == "CONFIG_VERSION":
            changed_vars[vars] = value
        else:
            changed_vars[vars] = config_vars[vars]
        continue

name = "old_config.py"
if os.path.exists(name):
    cpt = 1
    while os.path.exists(f"old_config_{cpt}.py"):
        cpt+=1
    name = f"old_config_{cpt}.py"

os.rename("config.py", name)
print(f"The current config file has been renamed {name}.")

print("Here are all the different values between the last config default and the current config,\nthe values of the current config file have been transferred to the new one : ")
for k,v in changed_vars.items():
    if k != "CONFIG_VERSION":
        print(f"\t- {k} = {v}")

regex_replace = r'^(?!["#\n])(.*?)=(.*?)([#].*)?$'
with open(version_defaults_file[-1][1], 'r') as read_file, open('config.py', 'w+') as write_file:
    for line in read_file.readlines():
        res = re.search(regex_replace, line)
        if res:
            var_name = res.group(1).replace(" ","")
            if var_name in changed_vars:
                if isinstance(changed_vars[var_name],str):
                    new_var = f'"{str(changed_vars[var_name])}"'
                else:
                    new_var = repr(changed_vars[var_name])
                result = re.sub(regex_replace, var_name+" = "+new_var+" \\3", line, 0, re.MULTILINE)
                if result:
                    write_file.write(result)
                    continue
        write_file.write(line)
