function enableNext() {
    if ($('#zDone').is(':checked')) {
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