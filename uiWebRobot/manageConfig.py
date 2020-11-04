import fileinput
import shutil

class ManageConfig : 

    def __init__(self, pathOfConfig: str):
        self.pathOfConfig = pathOfConfig
        self.openConfigOut()

    def openConfigOut(self):
        self.configFileOut = open(self.pathOfConfig, "r")
    
    def closeConfigOut(self):
        self.configFileOut.close()

    def setValue(self, path: str, value):
        self.closeConfigOut()
        with fileinput.FileInput(self.pathOfConfig, inplace=True, backup='.bak') as file:
            for line in file:
                if path in line:
                    print(path + " = " + str(value), end='\n')
                else:
                    print(line, end='')
        shutil.chown(self.pathOfConfig, "violette")
        self.openConfigOut()

    def getValue(self, value: str):
        self.configFileOut.seek(0)
        for line in self.configFileOut:
            if value in line:
                result = line.split("=")[1].split("#")[0].replace(" ","").replace("\n","")
                if result == "True":
                    return True
                elif result == "False":
                    return False
                elif "float" in result:
                    return float(result.split("\"")[1])
                elif "\"" in result[0]:
                    return result.replace("\"","")
                elif "." not in result:
                    return int(result)
                else:
                    return float(result)

    def getValues(self, values: list):
        results = dict()
        self.configFileOut.seek(0)
        for line in self.configFileOut:
            for value in values:
                if value in line:
                    result = line.split("=")[1].split("#")[0].replace(" ","").replace("\n","")
                    if result == "True":
                        results[value] = True
                    elif result == "False":
                        results[value] = False
                    elif "float" in result:
                        results[value] = float(result.split("\"")[1])
                    elif "\"" in result[0]:
                        results[value] = result.replace("\"","")
                    elif "." not in result:
                        results[value] = int(result)
                    else:
                        results[value] = float(result)
        return results