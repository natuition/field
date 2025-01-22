const socketBroadcast_ = io.connect('http://' + document.domain + ':' + location.port + '/broadcast');

let lastAlert;

function sendAlert(message_name,message,reload=true){
    show_alert(message_name, message);
    setTimeout(hide_alert, 5000, reload);
}

function show_alert(message_name, message, type_alert="alert-danger"){
    console.log("Popup with '"+message_name+"' message.")
    var popup_modal = document.getElementById('popup_modal');
    var popup_modal_text = document.getElementById('popup_modal_text');
    var popup_modal_alert = document.getElementById('popup_modal_alert');
    popup_modal_alert.classList = "alert " + type_alert;
    popup_modal_text.innerHTML = message;
    popup_modal.style.display = 'block';
}

function hide_alert(reload=true){
    var popup_modal = document.getElementById('popup_modal');
    var popup_modal_text = document.getElementById('popup_modal_text');
    popup_modal_text.innerHTML = "";
    popup_modal.style.display = 'none';
    if(reload) document.location.reload();
}

socketBroadcast_.on('notification', function(data) {
    clearTimeout(lastAlert);
    lastAlert = setTimeout(sendAlert, 500, data["message_name"], data["message"]);
});

socketBroadcast_.on('popup_modal', function(data) {
    show_alert(data["message_name"], data["message"], data["type_alert"]);
});

socketBroadcast_.on('reload', function(dataServ) {
    document.location.reload();
    console.log("Reload !")
});
