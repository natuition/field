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
    console.log(new_setting);
    socketSaveSetting.emit('data', new_setting);
}