var socketio = io.connect('http://' + document.domain + ':' + location.port + '/server');
socketio.on("reconnect_attempt", (attempt) => {
    if(attempt > 2) location.reload();
});
var isCheck = false

document.getElementById('frameCam').src = 'http://' + document.domain + ':8080/video';

function checkAllBoxAreChecked(){
    var select_ai = document.getElementById("AI_selector");
    if (document.getElementById('closecover').checked &&
        document.getElementById('XY').checked &&
        document.getElementById('opencover').checked &&
        document.getElementById('Z').checked &&
        document.getElementById('camera').checked && 
        document.getElementById('wheelsStraight').checked && 
        isCheck == false){
    		console.log("User all check !")
            $('#checkbutton').attr('disabled', ''); 
            $('#checkbutton').addClass('unselectable'); 
            $('#checkbutton').addClass('active'); 
            $('#AI_selector').attr('disabled', ''); 
            socketio.emit('data', {type: "allChecked", strategy: select_ai.value});
            isCheck = true
    }

}

function activateNext(){
    if (document.getElementById('closecover').checked &&
        document.getElementById('XY').checked &&
        document.getElementById('opencover').checked &&
        document.getElementById('Z').checked &&
        document.getElementById('camera').checked && 
        document.getElementById('wheelsStraight').checked){
            $('#checkbutton').removeAttr('disabled');   
            $('#checkbutton').removeClass('disabled');
    }else{
        $('#checkbutton').addClass('disabled'); 
        $('#checkbutton').attr('disabled', ''); 
    }
}

socketio.on('checklist', function(dataServ) {
    if(dataServ["status"] == "refresh"){
        document.location.reload();
    }
});

window.addEventListener("load", function(event) {
    socketio.emit('data', {type: "getInputVoltage"});
});