const socketBroadcast_ = io.connect('http://' + document.domain + ':' + location.port + '/broadcast');

let lastAlert;

function sendAlert(message_name,message){
    console.log("Popup with '"+message_name+"' message.")
    alert(message);
    document.location.reload();
}

socketBroadcast_.on('notification', function(data) {
    clearTimeout(lastAlert);
    lastAlert = setTimeout(sendAlert, 500, data["message_name"], data["message"]);
});