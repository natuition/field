let live_x_labels = []; // Labels for x axe
let live_data_temp = []; // Y axes
let live_data_rpm = []; // Y axes
let live_data_torque = []; // Y axes

let initialTimestamp = null;


// Listenning of the socketio channel
socketio.on('analyse_data_vesc', function(dataArray) {
    updateLiveGraph(dataArray);
});


function updateLiveGraph(dataArray) {
    dataArray.forEach((dataPoint) => {
        console.log(dataPoint);
        const temp = dataPoint.temp_motor_filtered;
        const erpm = dataPoint.rpm;
        const rpm = erpm / 7 // RPM calculating
        const current = dataPoint.avg_motor_current;
        const torque = current * 0.081; // Torque calculating
        const currentTime = dataPoint.timestamp;

        live_x_labels.push(currentTime);
        live_data_temp.push(temp);
        live_data_rpm.push(rpm);
        live_data_torque.push(torque);


        // Remove point olders than 10 seconds
        const currentTimestamp = dataArray[dataArray.length - 1].timestamp;
        while (live_x_labels.length > 0 && (currentTimestamp - live_x_labels[0]) > 10) {
            live_x_labels.shift();
            live_data_temp.shift();
            live_data_rpm.shift();
            live_data_torque.shift();
        }

        myLiveChart.update();
    });
}



const ctx_live_chart = document.getElementById('myLiveChart').getContext('2d');
const myLiveChart = new Chart(ctx_live_chart, {
    type: 'line',
    data: {
        labels: live_x_labels,
        datasets: [
            {
                label: 'Temperature (°C)',
                data: live_data_temp,
                borderColor: 'rgba(75, 192, 192, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_temp',
            },
            {
                label: 'Speed (RPM)',
                data: live_data_rpm,
                borderColor: 'rgba(123, 12, 12, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_rpm',
            },
            {
                label: 'Torque (N/M)',
                data: live_data_torque,
                borderColor: 'rgba(12, 234, 12, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_torque',
            }
        ]
    },
    options: {
        scales: {
            xAxes: [{
                position: 'bottom',
                title: {
                    display: true,
                    text: 'Temps (s)',
                },
                ticks: {
                    callback: function(value, index, ticks) {
                        if(index%8==0) {
                            return (value - ticks[0]).toFixed(2);
                        }
                        return "" ;
                    }
                }
            }],
            yAxes: [
                {
                    id: "y_temp",
                    display : true,
                    position : "right",
                    beginAtZero: true,
                    scaleLabel: {
                        display: true,
                        labelString: 'Temperature (°C)',
                        fontColor: 'rgba(75, 192, 192, 1)',
                        fontSize: 14
                    },
                    ticks: {
                        suggestedMin: 0,
                        suggestedMax: 150,
                    },
                },
                {
                    id: "y_rpm",
                    display : true,
                    position : "right",
                    beginAtZero: true,
                    scaleLabel: {
                        display: true,
                        labelString: 'Speed (RPM)',
                        fontColor: 'rgba(123, 12, 12, 1)',
                        fontSize: 14
                    },
                    ticks: {
                        suggestedMin: -2000,
                        suggestedMax: 2000,
                    },
                },
                {
                    id: "y_torque",
                    display : true,
                    position : "right",
                    beginAtZero: true,
                    scaleLabel: {
                        display: true,
                        labelString: 'Torque (N/m)',
                        fontColor: 'rgba(12, 234, 12, 1)',
                        fontSize: 14
                    },
                    ticks: {
                        suggestedMin: -1,
                        suggestedMax: 10,
                    },
                }
            ]
        }
    }
});



let shouldUpdateRegisteredGraph = true;

let registred_x_labels = []
let registred_data_temp =[]
let registred_data_rpm =[]
let registred_data_torque =[]

let initialTimestampRegistered = null;


// Listenning of the socketio channel
socketio.on('analyse_extraction_pattern', function(dataArray) {
    console.log(dataArray);
    if(shouldUpdateRegisteredGraph) {
        shouldUpdateRegisteredGraph = false
        document.getElementById('loadingIcon').style.display = 'none';
        updateRegistredGraph(dataArray);
    };

});

function updateRegistredGraph(dataArray) {
    // Erase the old graph
    myRegistredChart.data.labels.pop()
    myRegistredChart.data.datasets.forEach((dataset) => {
        dataset.data.pop();
    });
    registred_x_labels.length = 0;
    registred_data_temp.length = 0;
    registred_data_rpm.length = 0;
    registred_data_torque.length = 0;

    dataArray.forEach((dataPoint) => {
        const temp = dataPoint.temp_motor_filtered;
        const erpm = dataPoint.rpm;
        const rpm = erpm / 7; // RPM calculating
        const current = dataPoint.avg_motor_current;
        const torque = current * 0.081; // Torque calculating
        const currentTime = dataPoint.timestamp;

        registred_x_labels.push(currentTime);
        registred_data_temp.push(temp);
        registred_data_rpm.push(rpm);
        registred_data_torque.push(torque);
    });

    myRegistredChart.update();
}





// Registred charts settings
const ctx_registred_chart = document.getElementById('myRegistredChart').getContext('2d');
const myRegistredChart = new Chart(ctx_registred_chart, {
    type: 'line',
    data: {
        labels: registred_x_labels,
        datasets: [
            {
                label: 'Temperature (°C)',
                data: registred_data_temp,
                borderColor: 'rgba(75, 192, 192, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_temp',
            },
            {
                label: 'Speed (RPM)',
                data: registred_data_rpm,
                borderColor: 'rgba(123, 12, 12, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_rpm',
            },
            {
                label: 'Torque (N/M)',
                data: registred_data_torque,
                borderColor: 'rgba(12, 234, 12, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_torque',
            }
        ]
    },
    options: {
        scales: {
            xAxes: [{
                position: 'bottom',
                title: {
                    display: true,
                    text: 'Temps (s)',
                },
                ticks: {
                    callback: function(value, index, ticks) {
                        if(index%8==0) {
                            return (value - ticks[0]).toFixed(2);
                        }
                        return "" ;
                    }
                }
            }],
            yAxes: [
                {
                    id: "y_temp",
                    display : true,
                    position : "right",
                    beginAtZero: true,
                    scaleLabel: {
                        display: true,
                        labelString: 'Temperature (°C)',
                        fontColor: 'rgba(75, 192, 192, 1)',
                        fontSize: 14
                    },
                    ticks: {
                        suggestedMin: 0,
                        suggestedMax: 150,
                    },
                },
                {
                    id: "y_rpm",
                    display : true,
                    position : "right",
                    beginAtZero: true,
                    scaleLabel: {
                        display: true,
                        labelString: 'Speed (RPM)',
                        fontColor: 'rgba(123, 12, 12, 1)',
                        fontSize: 14
                    },
                    ticks: {
                        suggestedMin: -2000,
                        suggestedMax: 2000,
                    },
                },
                {
                    id: "y_torque",
                    display : true,
                    position : "right",
                    beginAtZero: true,
                    scaleLabel: {
                        display: true,
                        labelString: 'Torque (N/m)',
                        fontColor: 'rgba(12, 234, 12, 1)',
                        fontSize: 14
                    },
                    ticks: {
                        suggestedMin: -1,
                        suggestedMax: 10,
                    },
                }
            ]
        }
    }
});




// Mask and unmask curve and axes
function toggleDatasetVisibility(datasetIndex, checkbox, axisId) {
    myLiveChart.data.datasets[datasetIndex].hidden = !checkbox.checked;
    const liveAxis = myLiveChart.options.scales.yAxes.find(axis => axis.id === axisId);
    if (liveAxis) {
        liveAxis.display = checkbox.checked;
    }
    myLiveChart.update();

    myRegistredChart.data.datasets[datasetIndex].hidden = !checkbox.checked;
    const registredAxis = myRegistredChart.options.scales.yAxes.find(axis => axis.id === axisId);
    if (registredAxis) {
        registredAxis.display = checkbox.checked;
    }
    myRegistredChart.update();
}

// Attach event listener to html checkbox
document.getElementById('toggleTemp').addEventListener('change', function() {
    toggleDatasetVisibility(0, this, 'y_temp'); // 0 is the position of the dataset of temperature in the chart
});

document.getElementById('toggleRPM').addEventListener('change', function() {
    toggleDatasetVisibility(1, this, 'y_rpm'); // 1 is the position of the dataset of rpm in the chart
});

document.getElementById('toggleTorque').addEventListener('change', function() {
    toggleDatasetVisibility(2, this, 'y_torque'); // 2 is the position of the dataset of torque in the chart
});

document.getElementById('newSignalButton').addEventListener('click', function() {
    shouldUpdateRegisteredGraph = true;
    document.getElementById('loadingIcon').style.display = 'block';
});