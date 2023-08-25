var socketio = io.connect('http://' + document.domain + ':' + location.port + '/server');
socketio.on("reconnect_attempt", (attempt) => {
    if (attempt > 2) location.reload();
});

var joystick_created = false;
var use_arrow = 0;
var x_y_arrow = [0, 0];

function createJoystick() {
    if (!joystick_created && getComputedStyle(document.getElementById('main')).display != "none") {
        let value = document.getElementById('joystick').clientHeight;
        var joy = new JoyStick('joystick', { title: "canvas_joystick", autoReturnToCenter: true, width: value, height: value });

        var isInCenter = false;

        setInterval(function () {
            if (use_arrow > 0) {
                var xCurrentValue = x_y_arrow[0];
                var yCurrentValue = x_y_arrow[1];
            } else {
                var xCurrentValue = joy.GetX();
                var yCurrentValue = joy.GetY();
            }
            let style_main = getComputedStyle(document.getElementById('main')).display
            if (!document.getElementById('canvas_joystick').classList.contains("disable") && style_main != 'none') {
                if (xCurrentValue != 0 || yCurrentValue != 0 || !isInCenter) {
                    socketio.emit('data', { type: 'joystick', x: -xCurrentValue, y: parseInt(yCurrentValue) });
                    if (xCurrentValue == 0 && yCurrentValue == 0) isInCenter = true;
                    else isInCenter = false;
                }
            }
        }, 200);
        joystick_created = true;
    }
}

createJoystick();

window.addEventListener("orientationchange", function () {
    createJoystick();
}, false);


use_arrow_dir = [0, 0, 0, 0];

document.onkeydown = function (e) {
    if (!document.getElementById('canvas_joystick').classList.contains('disable')) {
        switch (e.key) {
            case "ArrowLeft":
                x_y_arrow[0] = -100;
                if (use_arrow_dir[0] == 0) {
                    use_arrow_dir[0] = 1;
                    use_arrow += 1;
                }
                break;

            case "ArrowUp":
                x_y_arrow[1] = 100;
                if (use_arrow_dir[1] == 0) {
                    use_arrow_dir[1] = 1;
                    use_arrow += 1;
                }
                break;

            case "ArrowRight":
                x_y_arrow[0] = 100;
                if (use_arrow_dir[2] == 0) {
                    use_arrow_dir[2] = 1;
                    use_arrow += 1;
                }
                break;

            case "ArrowDown":
                x_y_arrow[1] = -100;
                if (use_arrow_dir[3] == 0) {
                    use_arrow_dir[3] = 1;
                    use_arrow += 1;
                }
                break;

            default: return; // exit this handler for other keys
        }
    }

    e.preventDefault(); // prevent the default action (scroll / move caret)
};

document.onkeyup = function (e) {
    if (!document.getElementById('canvas_joystick').classList.contains('disable')) {
        switch (e.key) {
            case "ArrowLeft":
                x_y_arrow[0] = 0;
                use_arrow_dir[0] = 0;
                break;

            case "ArrowUp":
                x_y_arrow[1] = 0;
                use_arrow_dir[1] = 0;
                break;

            case "ArrowRight":
                x_y_arrow[0] = 0;
                use_arrow_dir[2] = 0;
                break;

            case "ArrowDown":
                x_y_arrow[1] = 0;
                use_arrow_dir[3] = 0;
                break;

            default: return; // exit this handler for other keys
        }
    }
    use_arrow -= 1;
    e.preventDefault(); // prevent the default action (scroll / move caret)
};