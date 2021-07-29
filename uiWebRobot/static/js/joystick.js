var socketio = io.connect('http://' + document.domain + ':' + location.port + '/server');
socketio.on("reconnect_attempt", (attempt) => {
    if(attempt > 4) location.reload();
});

var value = $(document.querySelector('.main')).width() * 0.5

var joy = new JoyStick('joystick',{title:"canvas",autoReturnToCenter:true,width:value,height:value});

document.getElementById('joystick').style.width = value + "px"
document.getElementById('joystick').style.height = value + "px"

var isInCenter = false;

setInterval(function(){ 
    var xCurrentValue = joy.GetX();
    var yCurrentValue = joy.GetY();
    if(!document.getElementById('canvas').classList.contains("disable")){
        if(xCurrentValue==0 && yCurrentValue==0&&!isInCenter){
            //console.log("X:"+0 + ", Y:"+0); 
            socketio.emit('data', {type: 'joystick', x : -xCurrentValue , y : yCurrentValue});
            isInCenter=true;
        }else{
            //console.log("X:"+xCurrentValue + ", Y:"+yCurrentValue); 
            socketio.emit('data', {type: 'joystick', x : -xCurrentValue , y : yCurrentValue});
            isInCenter=false;
        }
    }
}, 200);