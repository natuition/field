var socketio = io.connect('http://' + document.domain + ':' + location.port + '/server');
socketio.on("reconnect_attempt", (attempt) => {
    if (attempt > 2) location.reload();
});
var isCheck = false

document.getElementById('frameCam').src = 'http://' + document.domain + ':8080/video';

function canNext() {
    return document.getElementById('closecover').checked &&
        document.getElementById('XY').checked &&
        document.getElementById('opencover').checked &&
        document.getElementById('Z').checked &&
        document.getElementById('camera').checked &&
        document.getElementById('wheelsStraight').checked &&
        document.getElementById("voltage_indicator").getAttribute("bumper_disable") == "true";
}

function checkAllBoxAreChecked() {
    var select_ai = document.getElementById("AI_selector");
    if (canNext() && isCheck == false) {
        isCheck = true
        console.log("User all check !")
        $('#checkbutton').attr('disabled', '');
        $('#checkbutton').addClass('unselectable');
        $('#checkbutton').addClass('active');
        $('#AI_selector').attr('disabled', '');
        socketio.emit('data', { type: "allChecked", strategy: select_ai.value });
        setTimeout(() => {
            document.location.reload();
        }, 3000);
    }

}

function activateNext() {
    if (canNext()) {
        $('#checkbutton').removeAttr('disabled');
        $('#checkbutton').removeClass('disabled');
    } else {
        $('#checkbutton').addClass('disabled');
        $('#checkbutton').attr('disabled', '');
    }
}

socketio.on('checklist', function (dataServ) {
    if (dataServ["status"] == "refresh") {
        document.location.reload();
    }
});

window.addEventListener("load", function (event) {
    socketio.emit('data', { type: "getInputVoltage" });
});