const status = document.querySelector('.status')
const statusActive = document.querySelector('.status__active')
const statusTitle = document.querySelector('.status__active--title')
const generateField = document.querySelector('.ruler')
const socketButton = io.connect('http://' + document.domain + ':' + location.port + '/button');
const socketBroadcast = io.connect('http://' + document.domain + ':' + location.port + '/broadcast');

var newFieldButton = document.querySelector('#Newfield');
const startButton = document.querySelector('#Start');
const continueButton = document.querySelector('#Continue');
const stopButton = document.querySelector('#Stop');
const wheelButton = document.querySelector('#Wheel');
const auditButton = document.querySelector('#Audit');

if(auditButton != null) auditButton.addEventListener('click', changeMode);
if(newFieldButton != null) newFieldButton.addEventListener('click', clickHandler);
else{
    newFieldButton = document.querySelector('#ValidateZone');
    if(newFieldButton != null) newFieldButton.addEventListener('click', clickHandler);
}
if(startButton != null) startButton.addEventListener('click', clickHandler);
if(continueButton != null) continueButton.addEventListener('click', clickHandler);
if(stopButton != null) stopButton.addEventListener('click', clickHandler);
if(wheelButton != null) wheelButton.addEventListener('click', clickHandler);

var header_map = document.querySelector('.ruler');
document.getElementById('map__header').style.width = $(header_map).width() + "px";

var audit = false;

function clickHandler() {
    if(this.id=="Newfield"){
        sliderValue = document.getElementById("r1").value
        socketio.emit('data', {type: "field", value : sliderValue});
    }else if(this.id=="ValidateZone"){
        sliderValue = document.getElementById("r1").value
        socketio.emit('data', {type: "validerZone", value : sliderValue});
    }else if(this.id=="Start"){
        socketio.emit('data', {type: "start", audit : audit});
    }else if(this.id=="Stop"){
        socketio.emit('data', {type: "stop"});
    }else if(this.id=="Continue"){
        socketio.emit('data', {type: "continue", audit : audit});
    }else if(this.id=="Wheel" && !this.classList.contains("disabled-wheel")){
        socketio.emit('data', {type: "wheel"});
    }
}

socketButton.on('start', function(dataServ) {
    if(dataServ["status"] == "pushed"){

        $('.begin__button--continue').addClass('disabled');
        $('.begin__button--continue').attr('disabled', '');

        $('#canvas').addClass('disable'); 

        $('.begin__button--start').addClass('active');
        $('.begin__button--start').attr('disabled', '');

        $(auditButton).addClass('fix');
    }
});

socketButton.on('continue', function(dataServ) {
    if(dataServ["status"] == "pushed"){
        $('.begin__button--start').addClass('disabled');
        $('.begin__button--start').attr('disabled', '');

        $('#canvas').addClass('disable'); 

        $('.begin__button--continue').addClass('active');
        $('.begin__button--continue').attr('disabled', '');

        $(auditButton).addClass('fix');
    }
});

socketButton.on('startMain', function(dataServ) {
    if(dataServ["status"] == "finish"){
        $(document.getElementsByClassName('active')[0]).addClass('finished');
        clearStats();
        setTimeout(() => { 
            button = document.getElementsByClassName('active')[0]
            button.id = "Stop";
            $(button.firstElementChild).text((ui_languages["Stop"])[ui_language]);
            $(button).removeClass('finished');
            $(button).removeClass('active');
            $(button).removeClass('begin__button--start');
            $(button).removeClass('begin__button--continue');
            $(button).addClass("begin__button--stop");
            $(button).addClass("unselectable");
            $(button).removeAttr('disabled');
            status.classList.remove('display-flex')
            status.classList.add('display-none')
            generateField.classList.remove('display-flex')
            generateField.classList.add('display-none')
            statusActive.classList.remove('display-none')
            statusActive.classList.add('display-flex')
            if(dataServ["audit"]){
                statusTitle.classList.add('display-block')
                statusTitle.classList.remove('display-none')
            }else{
                statusTitle.classList.add('display-none')
                statusTitle.classList.remove('display-block')
            }
        }, 2000);
    }
});

socketButton.on('stop', function(dataServ) {
    if(dataServ["status"] == "pushed"){
        $('#Stop').addClass('active');
    }else if(dataServ["status"] == "finish"){
        $(document.getElementsByClassName('active')[0]).addClass('finished');
        setTimeout(() => { 
            button = document.getElementsByClassName('active')[0];
            button.id = button.name;
            $(button.firstElementChild).text((ui_languages[button.name])[ui_language]);
            $(button).removeClass('finished');
            $(button).removeClass('active');
            $(button).removeClass("begin__button--stop");
            $(button).addClass("begin__button--"+button.name.toLowerCase());
            $(button).addClass("unselectable");
            $(button).removeAttr('disabled');

            status.classList.remove('display-none')
            status.classList.add('display-flex')

            generateField.classList.remove('display-none')
            generateField.classList.add('display-flex')

            statusActive.classList.remove('display-flex')
            statusActive.classList.add('display-none')

            $('#canvas').removeClass('disable'); 

            otherButton = button.name=='Start'?"continue":"start";

            $('.begin__button--'+otherButton).removeClass('disabled');
            $('.begin__button--'+otherButton).removeAttr('disabled', '');

            auditButton.classList.remove('fix');
            auditButton.setAttribute('src', '/static/extraction.png');
            auditButton.classList.remove("disable-extraction");
            audit = false;

            newFieldButton.classList.remove("disabled");
            newFieldButton.removeAttribute("disabled")
            wheelButton.classList.remove("disabled-wheel");
        }, 2000);
    }
});


socketButton.on('field', function(dataServ) {
    if(dataServ["status"] == "pushed"){

        $('.begin__button--continue').addClass('disabled');
        $('.begin__button--continue').attr('disabled', '');
        $('.begin__button--start').addClass('disabled');
        $('.begin__button--start').attr('disabled', '');
        $('#Audit').addClass('disable-switcher-audit');
        $('#Newfield').addClass('active');
        $('#Newfield').attr('disabled', '');
        $('#r1').attr('disabled', '');

    }else if(dataServ["status"] == "inRun"){

        divButton = document.getElementById("Newfield")
        divButton.id = "Stop";
        $(divButton.firstElementChild).text((ui_languages["Stop"])[ui_language]);
        $(divButton).addClass("arret");
        $(divButton).removeClass('finished');
        $(divButton).removeClass('active');
        $(divButton).removeAttr('disabled');

    }else if(dataServ["status"] == "finish"){

        divButton = document.getElementById("Stop")
        divButton.id = "ValidateZone";
        $(divButton.firstElementChild).text((ui_languages["Validate zone"])[ui_language]);
        $(divButton).removeClass('finished');
        $(divButton).removeClass('active');
        $(divButton).removeClass('arret');
        $(divButton).removeAttr('disabled');
        $('#r1').removeAttr('disabled');

    }else if(dataServ["status"] == "validate"){

        divButton = document.getElementById("ValidateZone")
        divButton.id = "Newfield";
        $(divButton.firstElementChild).text((ui_languages["New zone"])[ui_language]);
        $('#Start').removeAttr('disabled');   
        $('#Start').removeClass('disabled');
        $('#Audit').removeClass('disable-switcher-audit');
        wheelButton.classList.remove("disabled-wheel");

    }
});

socketBroadcast.on('audit', function(data) {
    if(data["audit"]){
        auditButton.setAttribute('src', '/static/extraction-disable.png');
        $(auditButton).addClass("disable-extraction");
        audit = true;
    }else{
        auditButton.setAttribute('src', '/static/extraction.png');
        $(auditButton).removeClass("disable-extraction");
        audit = false;
    }
});

function changeMode(){
    if(!this.classList.contains("fix") && !this.classList.contains("disable-switcher-audit")){
        if(!this.classList.contains("disable-extraction")){
            if(confirm((ui_languages["Audit ?"])[ui_language])) {
                socketBroadcast.emit('data', {type: "audit", audit: true});
            }
        }else{
            socketBroadcast.emit('data', {type: "audit", audit: false});
        }
    }
}
