function restart_ui() {
    var dialog = confirm("Do you want to restart the robot application ?");
    if (dialog) {
        $.ajax({
            type: "GET",
            url: 'http://' + document.domain + '/restart_ui',
            asynch: true
        });
    }
}