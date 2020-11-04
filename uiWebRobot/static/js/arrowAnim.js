function insertAfter(referenceNode, newNode) {
    referenceNode.parentNode.insertBefore(newNode, referenceNode.nextSibling);
}  

function getIdOnList(id){
    if(id == "arrow_left") return 0;
    if(id == "arrow_right") return 1;
    if(id == "arrow_up") return 2;
    if(id == "arrow_down") return 3;
}

var socketNav = io.connect('http://' + document.domain + ':' + location.port + '/navigation');

socketNav.on('direction', function(dataServ) {
    id = dataServ["direction"]
    if(dataServ["status"]){
        $('#'+id).addClass('arrowOn')
        if (id == "arrow_left"){
            $('#arrow_right').removeClass('arrowOn');
            $('#arrow_right').removeClass('arrowStop');
            $('#arrow_right').addClass('arrowOff');
        }else if( id == "arrow_right"){
            $('#arrow_left').removeClass('arrowOn');
            $('#arrow_left').removeClass('arrowStop');
            $('#arrow_left').addClass('arrowOff');
        } 
        statusButtons[getIdOnList(id)] = true
    }else{
        $('#'+id).removeClass('arrowOn')
        if (id == "arrow_left"){
            $('#arrow_right').removeClass('arrowOff');
            $('#arrow_right').addClass('arrowStop');
        }else if( id == "arrow_right"){
            $('#arrow_left').removeClass('arrowOff');
            $('#arrow_left').addClass('arrowStop');
        }
        statusButtons[getIdOnList(id)] = false
    }
});

socketNav.on('propulsion', function(dataServ) {
    id = dataServ["propulsion"]
    if(dataServ["status"]){
        $('#'+id).addClass('arrowOn')
        if (id == "arrow_up"){
            $('#arrow_down').removeClass('arrowOn');
            $('#arrow_down').removeClass('arrowStop');
            $('#arrow_down').addClass('arrowOff');
        }else if( id == "arrow_down"){
            $('#arrow_up').removeClass('arrowOn');
            $('#arrow_up').removeClass('arrowStop');
            $('#arrow_up').addClass('arrowOff');
        }
        statusButtons[getIdOnList(id)] = true
    }else{
        $('#'+id).removeClass('arrowOn')
        if (id == "arrow_up"){
            $('#arrow_down').removeClass('arrowOff');
            $('#arrow_down').addClass('arrowStop');
        }else if( id == "arrow_down"){
            $('#arrow_up').removeClass('arrowOff');
            $('#arrow_up').addClass('arrowStop');
        }
        statusButtons[getIdOnList(id)] = false
    }
});

function clickArrow(id){
    if(!buttonStart && !$('#'+id).hasClass('arrowOff')){
        socketNav.emit('direction', { direction : id , status : !statusButtons[getIdOnList(id)]});
    }
}

var worker = new Worker("js/worker.js");
worker.postMessage(['init',document.domain,location.port]);

$(".touchArrow").on("touchstart", function(e) {
    worker.postMessage(['param','go']);
    e.preventDefault();
    id = this.id
    if(!buttonStart && !$('#'+id).hasClass('arrowOff')){
        socketNav.emit('propulsion', { propulsion : id , status : !statusButtons[getIdOnList(id)]});
    }
});

$(".touchArrow").on("touchend", function(e) {
    worker.postMessage(['param','pause']);
    e.preventDefault();
    id = this.id
    if(!buttonStart && !$('#'+id).hasClass('arrowOff')){
        socketNav.emit('propulsion', { propulsion : id , status : !statusButtons[getIdOnList(id)]});
    }
});
