function quitScreening() {
    $(':button').prop('disabled', true);
    $('.cancel').html('<span class="spinner-border spinner-border-sm" role="status" aria-hidden="true"></span>');
    socketio.emit('data', { type: "screening_quit" })
}

function startScreening() {
    $('#start_screening').attr({ disabled: true });
    $('#start_screening').parent().addClass('d-none');
    $('#loading_screening').parent().removeClass('d-none');
    socketio.emit('data', { type: "screening_start" });
}

function pauseScreening() {
    $('#pause_screening').attr({ disabled: true });
    $('#pause_screening').parent().addClass('d-none');
    $('#loading_screening').parent().removeClass('d-none');
    socketio.emit('data', { type: "screening_pause" });
}

socketio.on('screening_status', function (data) {
    $('#loading_screening').parent().addClass('d-none');
    if (data == "started") {
        $('#pause_screening').parent().removeClass('d-none');
        $('#pause_screening').attr({ disabled: false });
    } else if (data == "paused") {
        $('#start_screening').parent().removeClass('d-none');
        $('#start_screening').attr({ disabled: false });
    } else if (data["count"]) {
        $('#counted_value').html(data["count"]);
    }
});

socketio.on('href_to', function (data) {
    if (data["delay"] != undefined) {
        var reloader = setTimeout(() => {
            window.location.href = 'http://' + document.domain + data["href"];
            reloader = undefined;
        }, data["delay"]);
    } else {
        window.location.href = 'http://' + document.domain + data["href"];
    }
});

window.addEventListener("load", function () {
    document.getElementsByTagName("d-none")[0].parentElement.classList.add("d-none");
    document.getElementsByTagName("d-none")[0].remove();
});