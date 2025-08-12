var socketio = io.connect('http://' + document.domain + ':' + location.port + '/server');
socketio.on("reconnect_attempt", (attempt) => {
    if (attempt > 2) location.reload();
});
var isCheck = false

var show_cam_interval = setInterval(show_cam, 1000);
var count_next_interval;
var all_checked_interval;

var count = 0;
var last_para_txt = null;
var loading_next = null;

function count_next() {
    loading_next.innerHTML = count.toString() + "% " + "<i class='fas fa-sync-alt fa-spin'></i>";
    // count = count + 5;
   /* if (count > 100) {
        clearInterval(count_next_interval);
        alert((ui_languages["Restart UI"])[ui_language]);
        $.ajax({
            type: "GET",
            url: 'http://' + document.domain + '/restart_ui',
            asynch: true
        });
        document.location.reload();
    }*/
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
        document.getElementById('no_cam') == null;
        //document.getElementById("voltage_indicator").innerHTML.match(/^\d+(\.\d+)?\sV$/);
}

function listValidationEvery500ms() {
    var select_ai = document.getElementById("AI_selector");
    socketio.emit('data', { type: "list_validation", strategy: select_ai.value });
}

socketio.on('data', function (dataServ) {
    if (dataServ["ACK"] == "list_validation") {
        clearInterval(all_checked_interval);
    }
});

function checkAllBoxAreChecked() {
    if (canNext() && isCheck == false) {
        isCheck = true;
        $('#checkbutton').attr('disabled', '');
        $('#checkbutton').addClass('unselectable');
        $('#checkbutton').addClass('active');
        $('#AI_selector').attr('disabled', '');
        loading_next = document.getElementById("checkbutton").getElementsByClassName('loading')[0];
        
        //count_next_interval = setInterval(count_next, 500);
        all_checked_interval = setInterval(listValidationEvery500ms, 1000);
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

socketio.on('list_validation', function (dataServ) {
    if (dataServ["status"] == "refresh") {
        //clearInterval(count_next_interval);
        document.location.reload();
    }
});

socketio.on('wait_working_state', function (dataServ) {
    if (dataServ["status"] == "refresh") {
        //clearInterval(count_next_interval);
        document.location.reload();
        socketio.emit('data', { type: "wait_working_state_refresh"});
    }
});

window.addEventListener("load", function (event) {
    //socketio.emit('data', { type: "getInputVoltage" });
});