function invertSwitch() {
    enableNext();
    enableApplyInvert();
}
function enableNext() {
    if ($("#xDone").is(':checked') && $('#yDone').is(':checked') && $('#dirDone').is(':checked')
        && !$('#invX').is(':checked') && !$('#invY').is(':checked') && !$('#invDir').is(':checked')) {
        $('#next').attr({
            class: "btn btn-secondary",
            disabled: false
        });
    } else {
        $('#next').attr({
            class: "btn btn-danger",
            disabled: true
        });
    }
}
function enableApplyInvert() {
    if ($('#invX').is(':checked') || $('#invY').is(':checked') || $('#invDir').is(':checked')) {
        $('#invert').attr({
            class: "btn btn-secondary d-block",
            disabled: false
        });
    } else {
        $('#invert').attr({
            class: "btn btn-secondary d-none",
            disabled: true
        });
    }
}
function disableAfterApplyInvert() {
    $('.btn').attr({ disabled: true });
    $('.switch').attr({ disabled: true });
    $('.check').attr({ disabled: true });
    $('.next').attr({ disabled: true });
    $('#invert').attr({
        class: "btn btn-secondary d-block",
        disabled: true
    });
    $('#content_apply_invert').text('Wait 30 seconds please');
    $('#spinner').attr({
        class: "spinner-border spinner-border-sm"
    });
}

$(document).ready(function () {
    enableNext();
});

socketio.on('reload', function (dataServ) {
    location.reload(true);
});