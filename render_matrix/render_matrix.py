from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import threading
import time

app = Flask(__name__)
socketio = SocketIO(app)

@app.route('/')
def index():
    pixel = 30
    content,width,footer = getContent("../last_detection_map.txt","../last_extraction_map.txt")
    pixelS = pixel
    widthS = pixel*width
    return render_template("matrice.html",pixelS=pixelS,widthS=widthS,content=content,footer=footer)

def getContent(path: str,path2: str):
    content = ""
    file = open(path, "r")
    file_extraction = open(path2, "r")
    last_data_extraction = list()
    extract_number = 0
    detect_number = 0
    width = 0
    cpt = 0
    for line in file_extraction:
        cpt+=1
        if cpt > 3:
            lines = line.replace("\n","").split(" ")
            last_data_extraction.append(lines)
    i,j = 0,0 
    for line in list(file):
        newLine=""
        width = len(line.split(" "))
        line = line.replace("\n","").split(" ")
        for char in line:
            if char not in [" ","\n"]:
                index = str(int(char)) if str(int(char)) > "0" else ""
                if last_data_extraction[i][j] != "0":
                    extract_number += 1
                    newLine += "<div class=\"rectangle extraction-"+last_data_extraction[i][j]+" rectangle-"+str(int(char))+"\"><div class=\"extraction color-"+last_data_extraction[i][j]+"\"></div>"+index+"</div>"
                else:
                    newLine += "<div class=\"rectangle rectangle-"+str(int(char))+"\">"+index+"</div>"
                if str(int(char)) == "1":
                    detect_number += 1
                j+=1
        content += "\t<div class=\"lines\">\n\t" + newLine + "</div>\n" 
        
        i+=1
        j=0
    footer = f"<p>Extract number : <span id=\"extract_number\">{extract_number}</span></p>"+ "\n"
    footer += f"<p>Detect number : <span id=\"detect_number\">{detect_number}</span></p>"+ "\n"
    return content,width,footer

def thread_function(socket):
    while True:
        content,width,footer = getContent("../last_detection_map.txt","../last_extraction_map.txt")
        socket.emit('content', content)
        socket.emit('footer', footer)
        time.sleep(1)

if __name__ == '__main__':
    thread = threading.Thread(target=thread_function, args=(socketio,))
    thread.start()
    socketio.run(app,port="8888",host="0.0.0.0")