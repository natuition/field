const statu = document.querySelector('.status')
const statusActive = document.querySelector('.status__active')
const statusTitle = document.querySelector('.status__active--title')
const generateField = document.querySelector('.ruler')
const socketButton = io.connect('http://' + document.domain + ':' + location.port + '/button');
socketButton.on("reconnect_attempt", (attempt) => {
    if(attempt > 2) location.reload();
});
const socketBroadcast = io.connect('http://' + document.domain + ':' + location.port + '/broadcast');
socketBroadcast.on("reconnect_attempt", (attempt) => {
    if(attempt > 2) location.reload();
});

var newFieldButton = document.querySelector('#Newfield');
const startButton = document.querySelector('#Start');
const continueButton = document.querySelector('#Continue');
const stopButton = document.querySelector('#Stop');
const wheelButton = document.querySelector('#Wheel');
//const auditButton = document.querySelector('#Audit');
const removeFieldButton = document.querySelector('#RemoveField');
const choose_field_selector = document.querySelector('#field_selector');

//if(auditButton != null) auditButton.addEventListener('click', changeMode);
if(newFieldButton != null) newFieldButton.addEventListener('click', clickHandler);
else{
    newFieldButton = document.querySelector('#ValidateZone');
    if(newFieldButton != null) newFieldButton.addEventListener('click', clickHandler);
}
if(startButton != null) startButton.addEventListener('click', clickHandler);
if(continueButton != null) continueButton.addEventListener('click', clickHandler);
if(stopButton != null) stopButton.addEventListener('click', clickHandler);
if(wheelButton != null) wheelButton.addEventListener('click', clickHandler);
if(removeFieldButton != null) removeFieldButton.addEventListener('click', clickHandler);

if(choose_field_selector != null){
    choose_field_selector.onchange = (event)=>{
        var inputText = event.target.value;
        socketio.emit('data', {type: "getField", field_name : inputText});
    }
}

var audit = false;

var reloader = 0;

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
    }else if(this.id=="RemoveField"){
        if (confirm((ui_languages["Check_remove_zone"])[ui_language])) {
            socketio.emit('data', {type: "removeField", field_name : choose_field_selector.value});
        }   
    }
}

socketButton.on('start', function(dataServ) {
    if(dataServ["status"] == "pushed"){

        $('.begin__button--continue').addClass('disabled');
        $('.begin__button--continue').attr('disabled', '');

        $('#canvas_joystick').addClass('disable'); 

        $('.begin__button--start').addClass('active');
        $('.begin__button--start').attr('disabled', '');

        //$(auditButton).addClass('fix');
    }
});

socketButton.on('continue', function(dataServ) {
    if(dataServ["status"] == "pushed"){
        $('.begin__button--start').addClass('disabled');
        $('.begin__button--start').attr('disabled', '');

        $('#canvas_joystick').addClass('disable'); 

        $('.begin__button--continue').addClass('active');
        $('.begin__button--continue').attr('disabled', '');

        //$(auditButton).addClass('fix');
    }
});

socketButton.on('startMain', function(dataServ) {
    if(dataServ["status"] == "finish"){
        //verif_iframe_start();
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
            statu.classList.remove('display-flex')
            statu.classList.add('display-none')
            generateField.classList.remove('display-flex')
            generateField.classList.add('display-none')
            statusActive.classList.remove('display-none')
            statusActive.classList.add('display-flex')
            /*if(dataServ["audit"]){
                statusTitle.classList.add('display-block')
                statusTitle.classList.remove('display-none')
            }else{
                statusTitle.classList.add('display-none')
                statusTitle.classList.remove('display-block')
            }*/
            socketio.emit('data', {type: "getStats"});
            if(dataServ["first_point_no_extractions"]){
                sendAlert("first_point_no_extractions", (ui_languages["first_point_no_extractions"])[ui_language], false) 
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

            statu.classList.remove('display-none')
            statu.classList.add('display-flex')

            generateField.classList.remove('display-none')
            generateField.classList.add('display-flex')

            statusActive.classList.remove('display-flex')
            statusActive.classList.add('display-none')

            $('#canvas_joystick').removeClass('disable'); 

            otherButton = button.name=='Start'?"continue":"start";

            $('.begin__button--'+otherButton).removeClass('disabled');
            $('.begin__button--'+otherButton).removeAttr('disabled', '');

            /*auditButton.classList.remove('fix');
            auditButton.setAttribute('src', '/static/extraction.png');
            auditButton.classList.remove("disable-extraction");*/
            audit = false;

            newFieldButton.classList.remove("disabled");
            newFieldButton.removeAttribute("disabled");
            wheelButton.classList.remove("disabled-wheel");
            //document.getElementById("webCamStream").remove();
            //verif_iframe_start();

            document.getElementById('map__header').contentWindow.location.reload();
        }, 2000);
    }
});

socketButton.on('field', function(dataServ) {
    if(dataServ["status"] == "pushed"){

        $('.begin__button--continue').addClass('disabled');
        $('.begin__button--continue').attr('disabled', '');
        $('.begin__button--start').addClass('disabled');
        $('.begin__button--start').attr('disabled', '');
        //$('#Audit').addClass('disable-switcher-audit');
        $('#Newfield').addClass('active');
        $('#Newfield').attr('disabled', '');
        $('#r1').attr('disabled', '');

        $('#RemoveField').addClass('disabled');
        $('#RemoveField').attr('disabled', '');
        $('#trash').attr('fill',"#5d61646b");

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

    }else if(dataServ["status"] == "validate_name"){

        divButton = document.getElementById("ValidateZone")
        $(divButton).addClass('active');
        $(divButton).attr('disabled', '');

        var currentdate = new Date(); 
        var datetime = String(currentdate.getDate()).padStart(2, '0') + "/"
                        + String(currentdate.getMonth()+1).padStart(2, '0')  + "/" 
                        + currentdate.getFullYear() + " "  
                        + String(currentdate.getHours()).padStart(2, '0') + ":"  
                        + String(currentdate.getMinutes()).padStart(2, '0') + ":" 
                        + String(currentdate.getSeconds()).padStart(2, '0');
        let field_name = prompt((ui_languages["Choose_field_name"])[ui_language], "");
        if (field_name == null || field_name == "") {
            field_name = datetime;
        }

        socketio.emit('data', {type: "field_name", name : field_name});

        reloader = setTimeout(()=>{ 
            document.location.reload();
            reloader = 0;
        }, 1000);

    }else if(dataServ["status"] == "validate"){

        divButton = document.getElementById("ValidateZone")
        divButton.id = "Newfield";
        $(divButton.firstElementChild).text((ui_languages["New zone"])[ui_language]);
        $(divButton).removeClass('active');
        $(divButton).removeAttr('disabled');
        $('#Start').removeAttr('disabled');   
        $('#Start').removeClass('disabled');
        //$('#Audit').removeClass('disable-switcher-audit');

        $('#RemoveField').removeAttr('disabled');
        $('#RemoveField').removeClass('disabled');
        $('#trash').attr('fill',"#FFF");
        wheelButton.classList.remove("disabled-wheel");

    }
});

socketBroadcast.on('reloader', function(dataServ) {
    if(!dataServ["status"]){
        if(reloader != 0){
            clearTimeout(reloader);
            reloader = 0;
        }
    }
});

socketBroadcast.on('audit', function(data) {
    if(data["audit"]){
        /*auditButton.setAttribute('src', '/static/extraction-disable.png');
        $(auditButton).addClass("disable-extraction");*/
        audit = true;
    }else{
        /*auditButton.setAttribute('src', '/static/extraction.png');
        $(auditButton).removeClass("disable-extraction");*/
        audit = false;
    }
});

function changeMode(){
    /*if(!this.classList.contains("fix") && !this.classList.contains("disable-switcher-audit")){
        if(!this.classList.contains("disable-extraction")){
            if(confirm((ui_languages["Audit ?"])[ui_language])) {
                socketBroadcast.emit('data', {type: "audit", audit: true});
            }
        }else{
            socketBroadcast.emit('data', {type: "audit", audit: false});
        }
    }*/
}
var iframe_verif;

function verif_iframe_stop() {
    clearInterval(iframe_verif);
}

function verif_iframe_start(){
    iframe_verif = setInterval(() => {
        if(document.getElementById('conteneur_stats') != null);
        if(document.getElementById('conteneur_stats').classList.contains("display-flex"));
        if(document.getElementById("webCamStream")) document.getElementById("webCamStream").remove();
        try {
            $.ajax({
                type : "HEAD",
                async : true,
                url : 'http://' + document.domain + ':8888'
            })
            .done(function() {
                var url = new URL(window.location.href);
                console.log(url.searchParams.get('notIframe'))
                if (!url.searchParams.get('notIframe')) {
                    if(document.getElementById("webCamStream")){
                        document.getElementById("webCamStream").src = 'http://' + document.domain + ':8888';
                    }else{
                        var ifrm = document.createElement("iframe");
                        ifrm.id = "webCamStream";
                        ifrm.src = 'http://' + document.domain + ':8888';
                        ifrm.width = "414px";
                        ifrm.height = "250px";
                        ifrm.style = "margin: 0 auto;display:block; border: none; border-radius: 20px; max-width: 374px; min-height: 210px; width: 100%; height: 100%;";
                        $( "#conteneur_stats" ).append(ifrm);
                    }
                }
                verif_iframe_stop();
            })
        } catch(e) {}
    }, 5000);
}


window.addEventListener("load", function(event) {
    socketio.emit('data', {type: "getInputVoltage"});
});