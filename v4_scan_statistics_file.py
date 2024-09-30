import os
import re
from datetime import datetime

path_logs = "./logs"

regex = r"Plantain_great: (\d*)|Daisy: (\d*)|Dandellion: (\d*)|Plantain_narrowleaf: (\d*)|Porcelle: (\d*)"

dict_ref = {
    1:"Plantain_great",
    2:"Daisy",
    3:"Dandellion",
    4:"Plantain_narrowleaf",
    5:"Porcelle"  
}

dict_plant = {
    1:0,
    2:0,
    3:0,
    4:0,
    5:0     
}

path_sorted = [x[0] for x in os.walk(path_logs) if x[0] not in [path_logs]]
path_sorted.sort(key=lambda date: datetime.strptime(date.split("logs/")[1], '%d-%m-%Y %H-%M-%S %f'))
directories = list()
index = 0
for path in path_sorted:
    print(f"{index} : {path}")
    index+=1

res = input("Give a log interval you are interested in : (eg : 1:3 takes logs 1 to 3 inclusive) : ")
regex = r"^(\d*):(\d*)$" 
res_regex = re.search(regex, res)

if res_regex is not None:
    minimum = int(res_regex.groups()[0])
    maximum = int(res_regex.groups()[1])
    if minimum >= 0 and maximum < len(path_sorted)-1:
        for path in path_sorted[minimum:maximum+1]:
            complet_path = path+"/statistics.txt"
            if os.path.isfile(complet_path):
                file = open(complet_path, "r")
                for line in file.readlines():
                    matches = re.finditer(regex, line, re.MULTILINE)
                    for matchNum, match in enumerate(matches, start=1):
                        for groupNum in range(0, len(match.groups())):
                            groupNum = groupNum + 1
                            if match.group(groupNum) is not None:
                                dict_plant[groupNum] += int(match.group(groupNum))
        for key in dict_ref.keys():
            print(f"{dict_ref[key]} : {dict_plant[key]}")
        exit(0)

print("Not good interval !")
exit(1)