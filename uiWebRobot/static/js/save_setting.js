const socketSaveSetting = io.connect('http://' + document.domain + ':' + location.port + '/save_setting');
socketSaveSetting.on("reconnect_attempt", (attempt) => {
    if(attempt > 2) location.reload();
});

function save_setting() {
    new_setting = {};
    for (let element of document.querySelectorAll('.new_value')) {
        switch (element.tagName.toLowerCase()) {
            case "input":
                new_setting[element.id]=element.checked;
                break;
            case "span":
                if(element.classList.contains("slider")){
                    new_setting[element.id]=parseFloat(element.textContent);
                }
                break;
            case "select":
                new_setting[element.id]=element.value;
                break;
            default:
                console.log("Node '" + element.tagName.toLowerCase() + "' are not implement !!!");
        }
    }
    socketSaveSetting.emit('data', new_setting);

    $('#Button-save').addClass('active');
    $('#Button-save').attr('disabled', '');

    $('#Button-cancel').addClass('disabled');
    $('#Button-cancel').attr('disabled', '');
    $('#Button-reboot_app').addClass('disabled');
    $('#Button-reboot_app').attr('disabled', '');
    $('#Button-reboot_robot').addClass('disabled');
    $('#Button-reboot_robot').attr('disabled', '');
}

socketSaveSetting.on('save_finish', function(dataServ) {
    $(document.getElementsByClassName('active')[0]).addClass('finished');
    setTimeout(() => { 
        $('#Button-save').removeClass('finished');
        $('#Button-save').removeClass('active');
        $('#Button-save').removeAttr('disabled');

        $('#Button-cancel').removeClass('disabled');
        $('#Button-cancel').removeAttr('disabled');
        $('#Button-reboot_app').removeClass('disabled');
        $('#Button-reboot_app').removeAttr('disabled');
        $('#Button-reboot_robot').removeClass('disabled');
        $('#Button-reboot_robot').removeAttr('disabled');
    }, 2000);
});

function go_to_page(path){
    //console.log(path)
    location.href=path;
}