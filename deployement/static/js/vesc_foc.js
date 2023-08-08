function enableNext() {
    if ($("#propDone").is(':checked') && $('#zDone').is(':checked')) {
        $('#next').prop('class', "btn btn-secondary");
        $('#next').prop('disabled', false);
    } else {
        $('#next').prop('disabled', true);
        $('#next').prop('class', "btn btn-danger");
    }
}
$(document).ready(function () {
    enableNext()
});