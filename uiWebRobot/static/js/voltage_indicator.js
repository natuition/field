const socketVoltage = io.connect('http://' + document.domain + ':' + location.port + '/voltage');

socketVoltage.on('update', function (data) {
    var voltage_indicator = document.getElementById("voltage_indicator")
    if (data == "?" || data > 5) {
        if (data == "?") voltage_indicator.innerHTML = "<i class='fas fa - spinner fa - spin' style='color: #fff;'></i>";
        voltage_indicator.innerHTML = data + " V";
        voltage_indicator.setAttribute("bumper_disable", true);
        $("#voltage_indicator").parent().css("background-color", "#58b166");
    }
    else {
        voltage_indicator.innerHTML = "Bumper !";
        voltage_indicator.setAttribute("bumper_disable", false);
        $("#voltage_indicator").parent().css("background-color", "#FF3232");
    }
    if (typeof activateNext !== "undefined") {
        activateNext();
    }
});