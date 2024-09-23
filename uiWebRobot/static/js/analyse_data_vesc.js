let x_labels = []; // Labels for x axe
let data_temp = []; // Y axes
let data_rpm = []; // Y axes
let data_torque = []; // Y axes

// Charts settings
const ctx = document.getElementById('myChart').getContext('2d');
const myChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: x_labels,
        datasets: [
            {
                label: 'Temperature (°C)',
                data: data_temp,
                borderColor: 'rgba(75, 192, 192, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_temp',
            },
            {
                label: 'Speed (RPM)',
                data: data_rpm,
                borderColor: 'rgba(123, 12, 12, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_rpm',
            },
            {
                label: 'Torque (N/M)',
                data: data_torque,
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

        x_labels.push(currentTime);
        data_temp.push(temp);
        data_rpm.push(rpm);
        data_torque.push(torque);

        // Remove point olders than 10 seconds
        const currentTimestamp = dataArray[dataArray.length - 1].timestamp;
        while (x_labels.length > 0 && (currentTimestamp - x_labels[0]) > 10) {
            x_labels.shift();
            data_temp.shift();
            data_rpm.shift();
            data_torque.shift();
        }

        myChart.update();
    });
}

// Listenning of the socketio channel
socketio.on('analyse_data_vesc', function(dataArray) {
    updateGraph(dataArray);
});


// Mask and unmask curve and axes
function toggleDatasetVisibility(datasetIndex, checkbox, axisId) {
    myChart.data.datasets[datasetIndex].hidden = !checkbox.checked;

    const axis = myChart.options.scales.yAxes.find(axis => axis.id === axisId);
    if (axis) {
        axis.display = checkbox.checked;
    }

    myChart.update();
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
