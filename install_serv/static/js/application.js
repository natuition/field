var socket = io.connect('http://' + document.domain + ':' + location.port + '/server');

socket.on('smoothie_return', function(data) {
    document.getElementById("answer").innerHTML = data;
})

function send_action(data){
    socket.emit("data",data);
}