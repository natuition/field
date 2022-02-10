const socketVoltage = io.connect('http://' + document.domain + ':' + location.port + '/voltage');

socketVoltage.on('update', function(data) {
    console.log(data)
});