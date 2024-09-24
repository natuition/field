let live_x_labels = []; // Labels for x axe
let live_data_temp = []; // Y axes
let live_data_rpm = []; // Y axes
let live_data_torque = []; // Y axes

let is_analysing_extraction = false
let timerActive = false;
let shouldUpdateRegisteredGraph = true;

let registred_x_labels = []
let registred_data_temp =[]
let registred_data_rpm =[]
let registred_data_torque =[]

// Live charts settings
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
            x: {
                type: 'linear',
                position: 'bottom',
                title: {
                    display: true,
                    text: 'Temps (s)',
                }
            },
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


function updateGraph(dataArray) {
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

        if (is_analysing_extraction) {
            registred_x_labels.push(currentTime);
            registred_data_temp.push(temp);
            registred_data_rpm.push(rpm);
            registred_data_torque.push(torque);
        }

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

// Listenning of the socketio channel
socketio.on('analyse_data_vesc', function(dataArray) {
    updateGraph(dataArray);
});


socketio.on('analyse_data_vesc_instruction', function(instruction) {
    
    if (JSON.parse(instruction) == "start_extraction_analyse") {
        if(shouldUpdateRegisteredGraph == false) {
            return;
        }
        if (is_analysing_extraction) {
            return;
        }

        myRegistredChart.data.labels.pop()
            myRegistredChart.data.datasets.forEach((dataset) => {
                dataset.data.pop();
            });
            registred_x_labels.length = 0;
            registred_data_temp.length = 0;
            registred_data_rpm.length = 0;
            registred_data_torque.length = 0;
        is_analysing_extraction = true;
        timerActive = true;

        setTimeout(function() {
            myRegistredChart.update();
            timerActive = false;
            is_analysing_extraction = false;
        }, 5000);
    }
});




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
            x: {
                type: 'linear',
                position: 'bottom',
                title: {
                    display: true,
                    text: 'Temps (s)',
                }
            },
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

    const axis = myLiveChart.options.scales.yAxes.find(axis => axis.id === axisId);
    if (axis) {
        axis.display = checkbox.checked;
    }

    myLiveChart.update();
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

document.getElementById('updateRegisteredGraph').addEventListener('change', function() {
    shouldUpdateRegisteredGraph = this.checked; // Met à jour la variable en fonction de l'état de la checkbox
});
