const newFieldButton = document.querySelector('#Newfield');
const startButton = document.querySelector('#Start');
const continueButton = document.querySelector('#Continue');
const stopButton = document.querySelector('#Stop');

if(newFieldButton != null) newFieldButton.addEventListener('click', clickHandler);
if(startButton != null) startButton.addEventListener('click', clickHandler);
if(continueButton != null) continueButton.addEventListener('click', clickHandler);
if(stopButton != null) stopButton.addEventListener('click', clickHandler);

buttonStart = false

var statusButtons = [false,false,false,false]
var socketButton = io.connect('http://' + document.domain + ':' + location.port + '/button');

function timeout(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

socketButton.on('field', function(dataServ) {
    if(dataServ["status"] == "pushed"){
        $('#Continue').attr('disabled', 'disabled');
        $('#Continue').addClass('disabled');
        $('.touchArrow').removeClass('arrowStop');
        $('.touchArrow').removeClass('arrowOn');
        $('.touchArrow').addClass('arrowOff');
        $('#Start').attr('disabled', 'disabled');
        $('#Start').addClass('disabled');
        $('#Newfield').addClass('active');
        $('#Newfield').attr('disabled', 'disabled');
    }else if(dataServ["status"] == "inRun"){
        divButton = document.getElementById("Newfield")
        divButton.id = "Stop";
        $(divButton.firstElementChild).text((ui_languages["Stop"])[ui_language]);
        $(divButton).css("background-color", "red");
        $(divButton).removeClass('finished');
        $(divButton).removeClass('active');
        $(divButton).removeAttr('disabled');
    }else if(dataServ["status"] == "finish"){
        divButton = document.getElementById("Stop")
        divButton.id = "Newfield";
        $(divButton.firstElementChild).text((ui_languages["New zone"])[ui_language]);
        $(divButton).removeAttr('style');
        $(divButton).removeClass('finished');
        $(divButton).removeClass('active');
        $(divButton).removeAttr('disabled');
        $('.arrowOff').addClass('arrow');
        $('.arrow').removeClass('arrowOff');
        $('.arrow').addClass('arrowStop');    
        $('#Start').removeAttr('disabled');   
        $('#Start').removeClass('disabled');
    }
});

socketButton.on('continue', function(dataServ) {
    if(dataServ["status"] == "pushed"){
        $('#Start').attr('disabled', 'disabled');
        $('#Start').addClass('disabled');
        $('.arrow').removeClass('arrowStop');
        $('.arrow').removeClass('arrowOn');
        $('.arrow').addClass('arrowOff');
        $('#Newfield').attr('disabled', 'disabled');
        $('#Newfield').addClass('disabled');
        $('#Continue').addClass('active');
        $('#Continue').attr('disabled', 'disabled');
        buttonStart = true
    }else if(dataServ["status"] == "finish"){
        $('#Continue').addClass('finished');
        setTimeout(() => { 
            divButton = document.getElementById("Continue")
            divButton.id = "Stop";
            $(divButton.firstElementChild).text((ui_languages["Stop"])[ui_language]);
            $(divButton).css("background-color", "red");
            $(divButton).removeClass('finished');
            $(divButton).removeClass('active');
            $(divButton).removeAttr('disabled');
        }, 2000);
    }
});

socketButton.on('start', function(dataServ) {
    if(dataServ["status"] == "pushed"){
        $('#Continue').attr('disabled', 'disabled');
        $('#Continue').addClass('disabled');
        $('.arrow').removeClass('arrowStop');
        $('.arrow').removeClass('arrowOn');
        $('.arrow').addClass('arrowOff');
        $('#Newfield').attr('disabled', 'disabled');
        $('#Newfield').addClass('disabled');
        $('#Start').addClass('active');
        $('#Start').attr('disabled', 'disabled');
        buttonStart = true
    }else if(dataServ["status"] == "finish"){
        $('#Start').addClass('finished');
        setTimeout(() => { 
            divButton = document.getElementById("Start")
            divButton.id = "Stop";
            $(divButton.firstElementChild).text((ui_languages["Stop"])[ui_language]);
            $(divButton).css("background-color", "red");
            $(divButton).removeClass('finished');
            $(divButton).removeClass('active');
            $(divButton).removeAttr('disabled');
        }, 2000);
    }
});

socketButton.on('stop', function(dataServ) {
    if(dataServ["status"] == "pushed"){
        $('#Stop').addClass('active');
        buttonStart = true
        setTimeout(() => { 
          document.location.reload(true);
        }, 4000);
    }else if(dataServ["status"] == "pushedF"){
        $('#Stop').addClass('active');
    }
});

function clickHandler() {
    statusButtons = [false,false,false,false]
    sliderValue = document.getElementById("slider-long").value
    if(this.id=="Newfield" || this.id=="NewField"){
        socketButton.emit('field', { status : "pushed", value : sliderValue});
    }else if(this.id=="Start"){
        socketButton.emit('start', { status : "pushed"});
    }else if(this.id=="Stop"){
        socketButton.emit('stop', { status : "pushed"});
    }else if(this.id=="Continue"){
        socketButton.emit('continue', { status : "pushed"});
    }
}