function run_apply_config() {
    $('.btn').attr({ disabled: true });
    $('#apply_config').html('<span class="spinner-border spinner-border-sm" role="status" aria-hidden="true"></span> Running...');
    socketio.emit('client_config', { 'apply': true });
}
socketio.on('apply_config', function (data) {
    if (data["apply_done"]) {
        $('.btn').removeAttr("disabled");
        $('#apply_config').attr({ disabled: true });
        $('#apply_config').html('Done');
    }
});