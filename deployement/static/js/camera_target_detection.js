var url_cam = 'http://' + document.domain + ':8080/video';

function enableNext() {
    if ($("#targetDetectionDone").is(':checked')) {
        $('#next').prop('class', "btn btn-secondary");
        $('#next').prop('disabled', false);
    } else {
        $('#next').prop('disabled', true);
        $('#next').prop('class', "btn btn-danger");
    }
}
$(document).ready(function () {
    $('#next').prop('disabled', true);
});

function runDetect() {
    $('#run_detection').attr({ disabled: true });
    $('#run_detection').text('Running...');
    $('#spinner').attr({ class: "d-flex justify-content-center align-items-center mt-3" })
    $('#fig_image').attr({ class: "d-none" });
    $('#retry_detection_div').attr({ class: "d-none justify-content-center align-items-center" });
    socketio.emit('run_target_detection', { run_detection: true })
}

socketio.on('image', function (data) {
    var arrayBufferView = new Uint8Array(data['image_data']);
    var blob = new Blob([arrayBufferView], { type: "image/jpeg" });
    var img_url = URL.createObjectURL(blob);
    document.getElementById("fig_image").src = img_url;
    $('#spinner').attr({ class: "d-none justify-content-center align-items-center mt-3" });
    $('#run_detection_div').attr({ class: "d-none justify-content-center align-items-center" });
    $('#fig_image').removeAttr("class");
    $('#retry_detection_div').attr({ class: "d-flex justify-content-center align-items-center" });
    $('#fig_image_txt').text(data['res']);
});