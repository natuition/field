var pieChart;
var pie = document.getElementById('pieChart').getContext('2d');

var labels = [
    ""
];
var data = [
    0
];

var updaterTime = null;
var lastTime;
var previous_sessions_working_time = null;

window.onload = function() {
    pieChart = new Chart(pie, {
        type: 'doughnut',
        data: {
            labels: labels,
            datasets: [
                {
                    data: data,
                    borderColor: ['rgba(0, 0, 0, 1)', 'rgba(0, 0, 0, 1)', 'rgba(0, 0, 0, 1)', 'rgba(0, 0, 0, 1)', 'rgba(0, 0, 0, 1)', 'rgba(0, 0, 0, 1)'],
                    backgroundColor: ['rgba(71, 168, 87, 0.7)', 'rgba(61, 138, 87, 0.7)', 'rgba(51, 108, 87, 0.7)', 'rgba(41, 78, 87, 0.7)', 'rgba(31, 48, 87, 0.7)', 'rgba(21, 18, 87, 0.7)'],
                }
            ]
        },
        options: {
            legend: {
                display: false,
                position: 'right',
                align: 'center',
                fontSize: 20
            },
            plugins: {
                doughnutlabel: {
                    labels: [
                        {
                            text: 'Total : ',
                            font: {
                                size: 20
                            }
                        },
                        {
                            text: '0',
                            font: {
                                size: 25,
                                weight: 'bold'
                            }
                        }
                    ]
                }
            }
        }
    });

    socketio.emit('data', {type: "getStats"});
};

function removeAllChildNodes(parent) {
    while (parent.firstChild) {
        parent.removeChild(parent.firstChild);
    }
}

function transformTime(){
    var date = new Date(lastTime);
    if(previous_sessions_working_time != null){
        var date = new Date(date - previous_sessions_working_time*1000);
    }
    var today = new Date(); 
    var timeSec = today-date;
    var timeDate = new Date(timeSec); 
    return String(timeDate.getUTCHours()).padStart(2, '0') + ":" + String(timeDate.getUTCMinutes()).padStart(2, '0') + ":" + String(timeDate.getUTCSeconds()).padStart(2, '0');
}

function setTimeInHTML(){
    var timeValue = transformTime();
    var statsTimeCount = document.getElementsByClassName("status__time--top")[0];
    removeAllChildNodes(statsTimeCount);
    var p = document.createElement("p");
    var span = document.createElement("span");
    span.id = "workTimeValue";
    var timeNode = document.createTextNode("Time: ");
    var timeValueNode = document.createTextNode(timeValue);
    span.appendChild(timeValueNode);
    p.appendChild(timeNode);
    p.appendChild(span);
    p.id = "workTime";
    statsTimeCount.appendChild(p);
}

socketio.on('statistics', function(dataServ) {
    lastTime = dataServ["time"];
    previous_sessions_working_time = dataServ["previous_sessions_working_time"]
    if(updaterTime !== null) clearInterval(updaterTime);
    setTimeInHTML();
    updaterTime = window.setInterval(()=>{
        var timeValue = transformTime();
        $("#workTimeValue").text(timeValue);
    }, 1000);
    if(dataServ["weeds"] != undefined){
        var number = 1;
        var count = 0;
        var statsPlants = document.getElementsByClassName("status__time--bottom")[0];
        removeAllChildNodes(statsPlants);
        data = []
        labels = []
        console.log(dataServ)
        for (var key in dataServ["weeds"]) {
            data.push(dataServ["weeds"][key]);
            key = key.replace(/\s+/g, '');
            labels.push((ui_languages[key])[ui_language]);
            if(document.getElementsByClassName("status__bottom--"+number).length == 0){
                var div = document.createElement("div");
                var p = document.createElement("p");
                var span = document.createElement("span");
                var plante = document.createTextNode((ui_languages[key])[ui_language]+":");
                var c = dataServ["weeds"][key];
                var numberS = document.createTextNode(c);
                count+=c;
                span.appendChild(numberS);
                p.appendChild(plante);
                p.appendChild(span);
                div.appendChild(p);
                div.classList.add("status__bottom--"+number);
                statsPlants.appendChild(div);
            }
            number++;
        }
        pieChart.data.datasets[0].data = data
        pieChart.data.labels = labels
        pieChart.options.plugins.doughnutlabel.labels[1].text = count+" "
        pieChart.update();

        /*var p = document.createElement("p");
        var span = document.createElement("span");
        var timeS = document.createTextNode("Total: ");
        var timeValue = document.createTextNode(count);
        span.appendChild(timeValue)
        p.appendChild(timeS)
        p.appendChild(span)
        p.id = "workTime"
        statsTimeCount.appendChild(p)*/   
    }
});

function clearStats(){
    pieChart.data.datasets[0].data = []
    pieChart.data.labels = []
    pieChart.options.plugins.doughnutlabel.labels[1].text = 0+" "
    pieChart.update();
    removeAllChildNodes(document.getElementsByClassName("status__time--bottom")[0]);
}