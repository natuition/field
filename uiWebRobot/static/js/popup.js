const socketBroadcast_ = io.connect('http://' + document.domain + ':' + location.port + '/broadcast');

let lastAlert;

function sendAlert(message_name,message,reload=true){
    console.log("Popup with '"+message_name+"' message.")
    alert(message);
    if(reload) document.location.reload();
}

socketBroadcast_.on('notification', function(data) {
    clearTimeout(lastAlert);
    lastAlert = setTimeout(sendAlert, 500, data["message_name"], data["message"]);
});

socketBroadcast_.on('reload', function(dataServ) {
    document.location.reload();
});