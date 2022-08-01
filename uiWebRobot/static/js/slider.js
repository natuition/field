function range(range_name, span_name) {
    let span=document.getElementById(span_name);
    let rng=document.getElementById(range_name); 
    setValueNumberOfRange(rng.value, span);
}

function setValueNumberOfRange(number, span){
    if(number < 100){
        if(number < 10){
            if(span.classList.contains("no_format")) span.innerHTML=number.toString();
            else span.innerHTML="&ensp;" + number.toString();
        }else{
            if(span.classList.contains("no_format")) span.innerHTML=number.toString();
            else span.innerHTML="&nbsp;" + number.toString();
        }
    }else{
        span.innerHTML=number;
    }
    if(document.getElementsByName("Newfield")[0] !== undefined) if(document.getElementsByName("Newfield")[0].id == "ValidateZone"){
        sliderValue = document.getElementById("r1").value;
        socketio.emit('data', {type: "modifyZone", value : sliderValue});
    }
}