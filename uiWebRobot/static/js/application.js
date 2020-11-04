
$(document).ready(function(){
    var socket = io.connect('http://' + document.domain + ':' + location.port + '/voltage');

    socket.on('newVoltage', function(dataServer) {
        dataGraphVoltage = window.myLine.data.datasets[0].data
        dataGraphVoltage.push(dataServer.voltage)
        dataGraphMinutes = window.myLine.data.labels
        dataGraphMinutes.push(dataServer.minutes)
        window.myLine.update();
    });

    receivedHistory = false

    socket.on('newHistoryVoltage', function(dataServ) {
        if(!receivedHistory){
            dataGraphVoltage = window.myLine.data.datasets[0].data
            dataGraphMinutes = window.myLine.data.labels
            dataServ.forEach(dataServer => {
                dataGraphVoltage.push(dataServer.voltage)
                dataGraphMinutes.push(dataServer.minutes)
                }
            )
            window.myLine.update();
            receivedHistory = true
        }
    });

});