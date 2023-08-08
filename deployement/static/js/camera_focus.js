var url_cam = 'http://' + document.domain + ':8080/video';

function enableNext() {
    if ($("#focusDone").is(':checked')) {
        $('#next').prop('class', "btn btn-secondary");
        $('#next').prop('disabled', false);
    } else {
        $('#next').prop('disabled', true);
        $('#next').prop('class', "btn btn-danger");
    }
}
$(document).ready(function () {
    $('#next').prop('disabled', true);
    verif_iframe_start();
});

var iframe_verif;

function verif_iframe_stop() {
    clearInterval(iframe_verif);
}

function verif_iframe_start() {
    iframe_verif = setInterval(() => {
        try {
            $.ajax({
                type: "HEAD",
                async: true,
                url: url_cam
            })
                .done(function () {
                    document.getElementById("camera_video").src = url_cam;
                    verif_iframe_stop();
                    $('#spinner').remove();
                    $('#camera_video').removeAttr("class");
                })
        } catch (e) { }
    }, 1000);
}