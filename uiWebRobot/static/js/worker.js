importScripts("socket.io.min.js")
var socketNav

function sendAlive(){
    console.log("alive")
    socketNav.emit('alive', {alive : true});
}

function startAlive(e){
    if(e.data[1] == "go"){
        t=setInterval(sendAlive,250);
    }else if(e.data[1] == "pause"){
        clearInterval(t);
    }
}

self.onmessage = function(e) {
    console.log('Msg: ' + e.data); //Affiche param1
    if(e.data[0] == "init"){
        socketNav = io.connect('http://' + e.data[1] + ':' + e.data[2] + '/navigation');
    }else if(e.data[0] == "param"){
        if(e.data[1] == "stop"){
            close();
            return;
        } 
        setTimeout(startAlive(e), 100);
    }
}