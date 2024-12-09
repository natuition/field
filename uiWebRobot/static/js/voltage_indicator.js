const socketVoltage = io.connect('http://' + document.domain + ':' + location.port + '/voltage');
socketVoltage.on('update', function (data) {
    var voltage_indicator = document.getElementById("voltage_indicator")
    if (voltage_indicator != null) {
        if (data == "?" || data > 5) {
            if (data == "?") {
                voltage_indicator.innerHTML = "<i class='fas fa-spinner fa-spin' style='color: #fff;'></i>";
                voltage_indicator.setAttribute("bumper_disable", false);
            } else {
                voltage_indicator.innerHTML = data + " V";
                voltage_indicator.setAttribute("bumper_disable", true);
            }
            $("#voltage_indicator").parent().css("background-color", "#58b166");
            $("#voltage_indicator").parent().css("width", "70px");
        }
        else {
            voltage_indicator.innerHTML = "Bumper !";
            voltage_indicator.setAttribute("bumper_disable", false);
            $("#voltage_indicator").parent().css("background-color", "#FF3232");
            $("#voltage_indicator").parent().css("width", "95px");
        }
        if (typeof activateNext !== "undefined") {
            activateNext();
        }
    }
});