function enableNext() {
    if ($("#propDone").is(':checked') && $('#zDone').is(':checked')) {
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