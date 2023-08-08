var url_cam = 'http://' + document.domain + ':8080/video';

function enableNext() {
    if ($("#roundDetectionDone").is(':checked')) {
        $('#next').prop('class', "btn btn-secondary");
        $('#next').prop('disabled', false);
    } else {
        $('#next').prop('disabled', true);
        $('#next').prop('class', "btn btn-danger");
    }
}
$(document).ready(function () {
    $('#next').prop('disabled', true);
    socketio.emit('run_round_detection', { run_detection: true })
});

socketio.on('image', function (data) {
    var arrayBufferView = new Uint8Array(data['image_data']);
    var blob = new Blob([arrayBufferView], { type: "image/jpeg" });
    var img_url = URL.createObjectURL(blob);
    document.getElementById("fig_image").src = img_url;
    $('#spinner').remove();
    $('#fig_image').removeAttr("class");
});