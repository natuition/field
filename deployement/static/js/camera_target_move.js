function enableNext() {
    if ($("#targetMoveDone").is(':checked')) {
        $('#next').prop('class', "btn btn-success");
        $('#next').prop('disabled', false);
    } else {
        $('#next').prop('disabled', true);
        $('#next').prop('class', "btn btn-danger");
    }
}
$(document).ready(function () {
    $('#next').prop('disabled', true);
});

function moveX(x) {
    socketio.emit('x_y_dir', { "x": x })
    socketio.emit('move_step', { "x": x })
}

function moveY(y) {
    socketio.emit('x_y_dir', { "y": y })
    socketio.emit('move_step', { "y": y })
}

function runMoveToTarget() {
    $('.btn').attr({ disabled: true });
    $('#run_move').html('<span class="spinner-border spinner-border-sm" role="status" aria-hidden="true"></span> Running...');
    socketio.emit('run_move_to_target', { "run_move_to_target": true })
}

socketio.on('move', function (data) {
    if (data["move"]) {
        $('.btn').removeAttr("disabled");
        $('#run_move').attr({ disabled: true });
        $('#run_move').html('Done');
    }
});

function sleep(time) {
    return new Promise((resolve) => setTimeout(resolve, time));
}