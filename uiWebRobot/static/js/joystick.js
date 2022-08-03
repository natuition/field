var socketio = io.connect('http://' + document.domain + ':' + location.port + '/server');
socketio.on("reconnect_attempt", (attempt) => {
    if(attempt > 2) location.reload();
});

var joystick_created = false;

function createJoystick(){
    let value = document.getElementById('joystick').clientHeight;
    var joy = new JoyStick('joystick',{title:"canvas_joystick",autoReturnToCenter:true,width:value,height:value});

    var isInCenter = false;

    setInterval(function(){ 
        var xCurrentValue = joy.GetX();
        var yCurrentValue = joy.GetY();
        if(!document.getElementById('canvas_joystick').classList.contains("disable")){
            if(xCurrentValue!=0 || yCurrentValue!=0 || !isInCenter){
                socketio.emit('data', {type: 'joystick', x : -xCurrentValue , y : parseInt(yCurrentValue)});
                if(xCurrentValue==0 && yCurrentValue==0) isInCenter=true;
                else  isInCenter=false;
            }
        }
    }, 200);
    joystick_created = true;
}

if(window.orientation==0) createJoystick();

window.addEventListener("orientationchange", function() {
    if(window.orientation==0 && !joystick_created) createJoystick();
  }, false);