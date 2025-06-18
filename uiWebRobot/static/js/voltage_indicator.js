const socketVoltage = io.connect('http://' + document.domain + ':' + location.port + '/voltage');

socketVoltage.on('update', function (data) {
    const voltage_indicator = document.getElementById("voltage_indicator");

    if (voltage_indicator != null) {
        if (data === "?") {
            voltage_indicator.innerHTML = "<i class='fas fa-spinner fa-spin' style='color: #fff;'></i>";
            voltage_indicator.setAttribute("bumper_disable", false);
            $("#voltage_indicator").parent().css("background-color", "#58b166");
            $("#voltage_indicator").parent().css("width", "70px");
            disableButtons();
        } else if (data === "Bumper") {
            voltage_indicator.innerHTML = "Bumper";
            voltage_indicator.setAttribute("bumper_disable", false);
            $("#voltage_indicator").parent().css("background-color", "#FF3232");
            $("#voltage_indicator").parent().css("width", "95px");
            disableButtons();
        } else if (data === "Reseting") {
            voltage_indicator.innerHTML = "Reseting";
            voltage_indicator.setAttribute("bumper_disable", false);
            $("#voltage_indicator").parent().css("background-color", "#FF9532");
            $("#voltage_indicator").parent().css("width", "95px");
            disableButtons();


        } else if (typeof data === "number") {
            voltage_indicator.innerHTML = data + " V";
            voltage_indicator.setAttribute("bumper_disable", true);
            $("#voltage_indicator").parent().css("background-color", "#58b166");
            $("#voltage_indicator").parent().css("width", "70px");
            enableButtons();
        }
    }
});


// Fonction pour desactiver les boutons 
function disableButtons() {
    const continue_button = document.getElementById("Continue");
    if (continue_button) {
        continue_button.classList.add("disabled");
        continue_button.setAttribute("disabled", "disabled");
    }
    const start_button = document.getElementById("Start");
    if (start_button) {
        start_button.classList.add("disabled");
        start_button.setAttribute("disabled", "disabled");
    }
    const newfield_button = document.getElementById("Newfield");
    if (newfield_button) {
        newfield_button.classList.add("disabled");
        newfield_button.setAttribute("disabled", "disabled");
    }
    const joystick_button = document.getElementById("canvas_joystick");
    if (joystick_button) {
        joystick_button.classList.add("disable");
    }
    const checklist_button = document.getElementById("checkbutton");
    if (checklist_button) {
        checklist_button.classList.add("disabled");
        checklist_button.setAttribute("disabled", "disabled");
    }
}

// Fonction pour activer les boutons
function enableButtons() {
    const continue_button = document.getElementById("Continue");
    if (continue_button) {
        continue_button.classList.remove("disabled");
        continue_button.removeAttribute("disabled");
    }
    const start_button = document.getElementById("Start");
    if (start_button) {
        start_button.classList.remove("disabled");
        start_button.removeAttribute("disabled");
    }
    const newfield_button = document.getElementById("Newfield");
    if (newfield_button) {
        newfield_button.classList.remove("disabled");
        newfield_button.removeAttribute("disabled");
    }
    const joystick_button = document.getElementById("canvas_joystick");
    if (joystick_button) {
        joystick_button.classList.remove("disable");
    }
    const checklist_button = document.getElementById("checkbutton");
    if (checklist_button) {
        checklist_button.classList.remove("disabled");
        checklist_button.removeAttribute("disabled");
    }
}