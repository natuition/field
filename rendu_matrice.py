from flask import Flask

app = Flask(__name__)

def getContent(path: str,path2: str):
    content = ""
    file = open(path, "r")
    file_extraction = open(path2, "r")
    last_data_extraction = list()
    width = 0
    for line in file_extraction:
        lines = line.replace("\n","").split(" ")
        last_data_extraction.append(lines)
    i,j = 0,0 
    for line in file:
        newLine=""
        width = len(line.split(" "))
        line = line.replace("\n","").split(" ")
        for char in line:
            if char not in [" ","\n"]:
                newLine += "<div class=\"rectangle extraction-"+last_data_extraction[i][j]+" rectangle-"+str(int(char))+"\"></div>"
                j+=1
        content += "\t<div class=\"lines\">\n\t" + newLine + "</div>\n" 
        i+=1
        j=0
    return content,width

@app.route('/')
def index():
    pixel = 30
    content,width = getContent("last_detection_map.txt","last_extraction_map.txt")
    lines = """
    <html lang="fr">
    <head>
        <meta http-equiv="refresh" content="1">
        <style>
            p{
                margin:0;
            }
            .rectangle{
                width:"""+str(pixel)+"""px;
                height:"""+str(pixel)+"""px;
                background-color:lightgreen;
            }
            .rectangle-1{
                background-color:white!important;
            }
            .rectangle-0{
                background-color:green!important;
            }
            .extraction-0{
                background-image: linear-gradient(45deg, #f00 25%, transparent 25%, transparent 75%, #f00 75%), linear-gradient(45deg, #f00 25%, transparent 25%, transparent 75%, #f00 75%);
                background-size: 5px 5px;
                background-position: 0 0, 2.5px 2.5px;
            }
            .lines{
                display: flex;
            }
            .content{
                border: 1px solid black;
                width: """+str(width*pixel)+"""px;
            }
        </style>
    </head>
    <body>
    <div class=\"content\">
    """+content+"</div></body>"
    return lines

if __name__ == "__main__":
    app.run(host="0.0.0.0",port="8888",debug=True, use_reloader=True)