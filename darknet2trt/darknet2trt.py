import os
import sys
import shutil

from tool.natuition_darknet2onnx import main as main_darknet2onnx
from tool.natuition_onnx2trt import main as main_onnx2trt

def choose_file(fileList: list, fileType: str):
    for cnt, fileName in enumerate(fileList, 1):
        sys.stdout.write(f"[{cnt}] {fileName}\n\r")
    choice = 0
    while choice == 0:
        try:
            choice = int(input(f"Select {fileType} file[1-{cnt}]: "))
        except ValueError:
            choice = 0
        if choice > len(fileList) or choice < 1:
            print(f"The choice must be between 1 and {len(fileList)} !")
            choice = 0
    return fileList[choice-1]

def main():
    items = os.listdir("./")
    fileTypeName = dict()
    allFileType = ["cfg","weights","names","jpg"]
    for fileType in allFileType:
        tmpList = [name for name in items if name.endswith(f".{fileType}")]
        if len(tmpList) == 0:
            print(f"Don't find file with {fileType} extension !")
            print(f"The program needs files with the following extensions in the directory '{os.getcwd()}' : ")
            for fileType in allFileType:
                print(f"\t- '.{fileType}'")
            exit(1)
        else:
            fileTypeName[fileType] = choose_file(tmpList, "fileType")

    onnx_path = main_darknet2onnx(  f"./{fileTypeName['cfg']}", \
                                    f"./{fileTypeName['names']}", \
                                    f"./{fileTypeName['weights']}", \
                                    f"./{fileTypeName['jpg']}", \
                                    f"{fileTypeName['cfg'].replace('.cfg','')}")
    
    main_onnx2trt(onnx_path)

    print("TRT creation finish !")

    shutil.move(f"./{onnx_path.replace('.onnx','.trt')}", f"../yolo/{onnx_path.replace('.onnx','.trt')}")
    shutil.copy2(f"./{fileTypeName['names']}", f"../yolo/{fileTypeName['names']}")

    print("TRT file is move in yolo and names is copy on yolo.")

if __name__ == "__main__":
    main()
