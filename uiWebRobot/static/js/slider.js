var rng=document.getElementById('r1'); 
var span=document.getElementById('rangeCol');

function range() {
    setValueNumberOfRange(rng.value);
}

function setValueNumberOfRange(number){
    if(number < 100){
        if(number < 10){
            span.innerHTML="&ensp;" + number.toString();
        }else{
            span.innerHTML="&nbsp;" + number.toString();
        }
    }else{
        span.innerHTML=number;
    }
    if(document.getElementsByName("Newfield")[0].id == "ValidateZone"){
        sliderValue = document.getElementById("r1").value;
        socketio.emit('data', {type: "modifyZone", value : sliderValue});
    }
}



setValueNumberOfRange(rng.value);