var socketio = io.connect('http://' + document.domain + ':' + location.port + '/server');
socketio.on("reconnect_attempt", (attempt) => {
    if (attempt > 2) location.reload();
});
var isCheck = false

var show_cam_interval = setInterval(show_cam, 1000);
var count_next_interval;

var count = 19;
var last_count_txt = null;

function count_next() {
    var count_txt = document.createTextNode(count.toString() + " ");
    var el = document.getElementById("checkbutton").getElementsByClassName('loading')[0];
    if (last_count_txt != null) el.removeChild(last_count_txt);
    el.prepend(count_txt);
    last_count_txt = count_txt;
    count = count - 1;
    if (count == -1) clearInterval(count_next_interval);
}

function show_cam() {
    try {
        var img = new Image();
        img.src = 'http://' + document.domain + ':8080/video';
        img.onload = function () {
            document.getElementById('frameCam').src = 'http://' + document.domain + ':8080/video';
            $('#no_cam').remove();
            clearInterval(show_cam_interval);
            activateNext();
        }
    } catch (error) {
        console.error(error);
    }
}

function canNext() {
    return document.getElementById('closecover').checked &&
        document.getElementById('XY').checked &&
        document.getElementById('opencover').checked &&
        document.getElementById('Z').checked &&
        document.getElementById('camera').checked &&
        document.getElementById('wheelsStraight').checked &&
        document.getElementById("voltage_indicator").getAttribute("bumper_disable") == "true" &&
        document.getElementById('no_cam') == null;
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
        count_next_interval = setInterval(count_next, 1000);
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