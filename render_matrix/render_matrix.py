from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import threading
import time

app = Flask(__name__)
socketio = SocketIO(app)

@app.route('/')
def index():
    pixel = 30
    content,width = getContent("../last_detection_map.txt","../last_extraction_map.txt")
    pixelS = pixel
    widthS = pixel*width
    return render_template("matrice.html",pixelS=pixelS,widthS=widthS,content=content)

def getContent(path: str,path2: str):
    content = ""
    file = open(path, "r")
    file_extraction = open(path2, "r")
    last_data_extraction = list()
    width = 0
    cpt = 0
    for line in file_extraction:
        cpt+=1
        if cpt > 3:
            lines = line.replace("\n","").split(" ")
            last_data_extraction.append(lines)
    i,j = 0,0 
    for line in file:
        newLine=""
        width = len(line.split(" "))
        line = line.replace("\n","").split(" ")
        for char in line:
            if char not in [" ","\n"]:
                if last_data_extraction[i][j] != "0":
                    newLine += "<div class=\"rectangle extraction-"+last_data_extraction[i][j]+" rectangle-"+str(int(char))+"\"><div class=\"extraction color-"+last_data_extraction[i][j]+"\"></div></div>"
                else:
                    newLine += "<div class=\"rectangle rectangle-"+str(int(char))+"\"></div>"
                j+=1
        content += "\t<div class=\"lines\">\n\t" + newLine + "</div>\n" 
        i+=1
        j=0
    return content,width

def thread_function(socket):
    while True:
        content,width = getContent("../last_detection_map.txt","../last_extraction_map.txt")
        socket.send(content)
        time.sleep(1)

if __name__ == '__main__':
    thread = threading.Thread(target=thread_function, args=(socketio,))
    thread.start()
    socketio.run(app,port="8888",host="0.0.0.0")