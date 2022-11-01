var socketio = io.connect('http://' + document.domain + ':' + location.port + '/server');
socketio.on("reconnect_attempt", (attempt) => {
    if(attempt > 2) location.reload();
});

var joystick_created = false;

function createJoystick(){
    if(!joystick_created && window.orientation==0){
        let value = document.getElementById('joystick').clientHeight;
        var joy = new JoyStick('joystick',{title:"canvas_joystick",autoReturnToCenter:true,width:value,height:value});

        var isInCenter = false;

        setInterval(function(){ 
            var xCurrentValue = joy.GetX();
            var yCurrentValue = joy.GetY();
            let style_main = getComputedStyle(document.getElementById('main')).display
            if(!document.getElementById('canvas_joystick').classList.contains("disable") && style_main != 'none'){
                if(xCurrentValue!=0 || yCurrentValue!=0 || !isInCenter){
                    socketio.emit('data', {type: 'joystick', x : -xCurrentValue , y : parseInt(yCurrentValue)});
                    if(xCurrentValue==0 && yCurrentValue==0) isInCenter=true;
                    else  isInCenter=false;
                }
            }
        }, 200);
        joystick_created = true;
    }
}

createJoystick();

window.addEventListener("orientationchange", function() {
    createJoystick();
  }, false);