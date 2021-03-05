var socketio = io.connect('http://' + document.domain + ':' + location.port + '/server');

var value = $(document.querySelector('.main')).width() * 0.5

var joy = new JoyStick('joystick',{title:"canvas",autoReturnToCenter:true,width:value,height:value});

document.getElementById('joystick').style.width = value + "px"
document.getElementById('joystick').style.height = value + "px"

var xLastValue = joy.GetX();
var yLastValue = joy.GetY();

var isInCenter = false;

setInterval(function(){ 
    var xCurrentValue = joy.GetX();
    var yCurrentValue = joy.GetY();
    if(!document.getElementById('canvas').classList.contains("disable")){
        if(xLastValue != xCurrentValue || yLastValue != yCurrentValue ){
            //console.log("X:"+xCurrentValue + ", Y:"+yCurrentValue); 
            socketio.emit('data', {type: 'joystick', x : -xCurrentValue , y : yCurrentValue});
            isInCenter=false;
        }else if(xCurrentValue==0 && yCurrentValue==0&&!isInCenter){
            //console.log("X:"+0 + ", Y:"+0); 
            socketio.emit('data', {type: 'joystick', x : -xCurrentValue , y : yCurrentValue});
            isInCenter=true;
        }
    }
}, 200);