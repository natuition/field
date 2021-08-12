var socketio = io.connect('http://' + document.domain + ':' + location.port + '/server');
socketio.on("reconnect_attempt", (attempt) => {
    if(attempt > 2) location.reload();
});
var isCheck = false

document.getElementById('frameCam').src = 'http://' + document.domain + ':8080/video';

function checkAllBoxAreChecked(){
    if (document.getElementById('closecover').checked &&
        document.getElementById('XY').checked &&
        document.getElementById('opencover').checked &&
        document.getElementById('Z').checked &&
        document.getElementById('camera').checked && 
        document.getElementById('wheelsStraight').checked && 
        isCheck == false){
    		console.log("User all check !")
            socketio.emit('data', {type: "allChecked"});
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