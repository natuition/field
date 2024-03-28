var sensorChart;
var sensorChartHtml = document.getElementById('sensorChart').getContext('2d');

var x_cpt = 0;
var x_data = [];
var data_iq = [];
var data_rpm = [];

function addData(chart, label, newData) {
    chart.data.labels.push(label);
    var cpt = 0;
    chart.data.datasets.forEach((dataset) => {
        dataset.data.push(newData[cpt]);
        cpt += 1;
    });
    chart.update();
}

function setData(chart, label, newData) {
    chart.data.labels = label;
    var cpt = 0;
    chart.data.datasets.forEach((dataset) => {
        dataset.data = newData[cpt];
        cpt += 1;
    });
    chart.update();
}

function clearAllData(chart) {
    chart.data.labels.length = 0;
    chart.data.datasets.forEach((dataset) => {
        dataset.data.length = 0;
    });
    chart.update();
}

socketio.on('statistics', function (dataServ) {
    if (dataServ["operation"] == "clear") {
        clearAllData(sensorChart);
        x_cpt = 0;
    } else if (dataServ["operation"] == "add") {
        addData(sensorChart, x_cpt, dataServ["y_data"]);
    } else if (dataServ["operation"] == "set") {
        setData(sensorChart, dataServ["x_data"], dataServ["y_data"]);
    }
});

window.onload = function () {
    sensorChart = new Chart(sensorChartHtml, {
        type: 'line',
        data: {
            labels: x_data,
            datasets: [
                {
                    label: "Torque",
                    data: data_iq,
                    borderColor: 'rgba(255, 0, 0)',
                    backgroundColor: 'rgba(255, 0, 0, 0)',
                    yAxisID: 'y_iq',
                    pointRadius: 0
                },
                {
                    label: "RPM",
                    data: data_rpm,
                    borderColor: 'rgba(0, 0, 255)',
                    backgroundColor: 'rgba(0, 0, 255, 0)',
                    yAxisID: 'y_rpm',
                    pointRadius: 0
                }
            ]
        },
        options: {
            responsive: true,
            stacked: false,
            elements: {
                point: {
                    radius: 0
                }
            },
            legend: {
                display: false
            },
            scales: {
                xAxes: [{
                    display: true,
                    ticks: {
                        fontColor: "white",
                        fontSize: 14
                    }
                }],
                yAxes: [
                    {
                        type: 'linear',
                        display: true,
                        position: 'right',
                        scaleLabel: {
                            display: true,
                            labelString: 'Torque',
                            fontColor: '#FF0000',
                            fontSize: 14
                        },
                        id: "y_iq",
                        gridLines: {
                            display: false
                        },
                        ticks: {
                            suggestedMin: 0,
                            suggestedMax: 7,
                        },
                    },
                    {
                        type: 'linear',
                        display: true,
                        position: 'left',
                        scaleLabel: {
                            display: true,
                            labelString: 'RPM',
                            fontColor: '#0000FF',
                            fontSize: 14
                        },
                        ticks: {
                            suggestedMin: -2000,
                            suggestedMax: 1500,
                        },
                        id: "y_rpm"
                    },
                ]
            }
        }
    });
};