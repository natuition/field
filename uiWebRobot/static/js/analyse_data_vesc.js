const socketio = io.connect('http://' + document.domain + ':' + location.port + '/server');

let shouldUpdateRegisteredGraph = true;
let receivedData = [];

let registred_x_labels = []
let registred_data_temp = []
let registred_data_rpm = []
let registred_data_torque = []

let nb_capture_before = 0;
let nb_capture_over = 0;
let nb_capture_after = 0;
let threshold = 0;

// Listen to data from the server via socketio
socketio.on('penetrometry_datas', function(data) {
    console.log("Extraction data received:");
    console.log(data);

    receivedData.push(...data.captures) // Decompose the array and push it
    if(data.is_last_slice){
        if (shouldUpdateRegisteredGraph) {
            console.log("Updating the graph");
            shouldUpdateRegisteredGraph = false;
    
            // Hide the loading container, display the button and parameters
            document.getElementById('loadingContainer').style.display = 'none';
            document.getElementById('paramContainer').style.display = 'block';
    
            // Update the parameter fields with the received data
            nb_capture_before = data.detection_parameters.nb_capture_before;
            nb_capture_over = data.detection_parameters.nb_capture_over;
            nb_capture_after = data.detection_parameters.nb_capture_after;
            threshold = data.detection_parameters.threshold;
    
            document.getElementById('nbCaptureBefore').value = nb_capture_before;
            document.getElementById('nbCaptureOver').value = nb_capture_over;
            document.getElementById('nbCaptureAfter').value = nb_capture_after;
            document.getElementById('threshold').value = threshold;
    
            // Update the graph with new data
            updateRegistredGraph(receivedData);
        }
    }
});

function updateRegistredGraph(dataArray) {
    let initialTimestamp = dataArray[0].timestamp;

    dataArray.forEach((dataPoint) => {
        const temp = dataPoint.temp_motor_filtered;
        const rpm = dataPoint.rpm_meca;
        const torque = dataPoint.torque;
        const currentTime = ((dataPoint.timestamp - initialTimestamp).toFixed(3)) * 1000;

        registred_x_labels.push(currentTime);
        registred_data_temp.push(temp);
        registred_data_rpm.push(rpm);
        registred_data_torque.push(torque);
    });
    myRegistredChart.data.datasets[0].data = registred_data_torque.map((v, i) => ({ x: registred_x_labels[i], y: v }))
    myRegistredChart.data.datasets[1].data = registred_data_rpm.map((v, i) => ({ x: registred_x_labels[i], y: v }))
    myRegistredChart.data.datasets[2].data = registred_data_temp.map((v, i) => ({ x: registred_x_labels[i], y: v }))
    myRegistredChart.options.annotation.annotations[0].value = threshold
    myRegistredChart.update();
}


// Registered chart settings
const ctx_registred_chart = document.getElementById('myRegistredChart').getContext('2d');
const myRegistredChart = new Chart(ctx_registred_chart, {
    type: 'scatter',
    data: {
        datasets: [
            {
                label: (ui_languages["torque"])[ui_language],
                data: registred_data_torque,
                borderColor: 'rgba(12, 234, 12, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_torque',
                showLine: true,
            },
            {
                label: (ui_languages["speed"])[ui_language],
                data: registred_data_rpm,
                borderColor: 'rgba(123, 12, 12, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_rpm',
                showLine: true,
            },
            {
                label: (ui_languages["temperature"])[ui_language],
                data: registred_data_temp,
                borderColor: 'rgba(75, 192, 192, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_temp',
                showLine: true,
            }
        ]
    },
    options: {
        scales: {
            xAxes: [{
                display: true,
                position: 'bottom',
                ticks: {
                  stepSize: 1,
                  autoSkip: false,
                  callback: function(value) {
                    if(value % 500 == 0 ) return value 
                    if(value % 100 == 0 ) return "" 
                    else return null
                   },
                  min: 0,
                  max: registred_x_labels[registred_x_labels.length - 1],
                  maxRotation: 0
                }
              }],
            yAxes: [
                {
                    id: "y_torque",
                    display : true,
                    position : "right",
                    beginAtZero: true,
                    scaleLabel: {
                        display: true,
                        labelString: (ui_languages["torque"])[ui_language],
                        fontColor: 'rgba(12, 234, 12, 1)',
                        fontSize: 14
                    },
                    ticks: {
                        suggestedMin: -4,
                        suggestedMax: 4,
                    },
                },
                {
                    id: "y_rpm",
                    display : true,
                    position : "right",
                    beginAtZero: true,
                    scaleLabel: {
                        display: true,
                        labelString: (ui_languages["speed"])[ui_language],
                        fontColor: 'rgba(123, 12, 12, 1)',
                        fontSize: 14
                    },
                    ticks: {
                        suggestedMin: -2000,
                        suggestedMax: 2000,
                    },
                },
                {
                    id: "y_temp",
                    display : true,
                    position : "right",
            
                    // Text to display in label - default is null
                    beginAtZero: true,
                    scaleLabel: {
                        display: true,
                        labelString: (ui_languages["temperature"])[ui_language],
                        fontColor: 'rgba(75, 192, 192, 1)',
                        fontSize: 14
                    },
                    ticks: {
                        suggestedMin: -50,
                        suggestedMax: 50,
                    },
                }
            ]
        },
        legend: {
            display: true,
            position: 'chartArea',
            align: 'end',
            labels: {
                boxWidth: 100,
                boxHeight: 100,
                generateLabels: function(chart) {
                    // Return legend items with only the colored box
                    return chart.data.datasets.map(function(dataset, i) {
                        return {
                            // Remove the text (label)
                            text: '',
                            fillStyle: dataset.borderColor, // Color of the box
                            strokeStyle: dataset.borderColor,
                            lineWidth: 2,
                            hidden: !chart.isDatasetVisible(i),
                            datasetIndex: i
                        };
                    });
                },
            },
        },
        annotation: {
            annotations: [{
                type: 'line',
                mode: 'horizontal',
                scaleID: 'y_rpm',
                value: threshold,
                borderColor: 'red',
                borderWidth: 2,
                label: {
                    // Background color of label, default below
                    backgroundColor: 'red',
                    fontSize: 12,
                    fontColor: "#234321",
                    xPadding: 6,
                    yPadding: 6,
                    cornerRadius: 6,
                    position: "left",
                    xAdjust: 0,
                    yAdjust: 0,
                    enabled: true,
                    content: (ui_languages["seuil vitesse"])[ui_language]
                },
            }]
        }
    }
});


document.getElementById('newSignalButton').addEventListener('click', function() {
    if(isInputsValid()){
        shouldUpdateRegisteredGraph = true;
        document.getElementById('loadingContainer').style.display = 'block';
        // Clear the old graph
        myRegistredChart.data.datasets.forEach((dataset) => {
            dataset.data = [];  // Reset the dataset data array
        });
        
        // Clear the source arrays used for data
        registred_x_labels = [];
        registred_data_torque = [];
        registred_data_rpm = [];
        registred_data_temp = [];
        receivedData = [];
        myRegistredChart.update();

        // Retrieve the current parameter values
        nb_capture_before = parseInt(document.getElementById('nbCaptureBefore').value);
        nb_capture_over = parseInt(document.getElementById('nbCaptureOver').value);
        nb_capture_after = parseInt(document.getElementById('nbCaptureAfter').value);
        threshold = parseFloat(document.getElementById('threshold').value);

        // Send the new parameters to the server for the next detection
        socketio.emit('data', {
            type: 'penetrometry_new_params',
            nb_capture_before: nb_capture_before,
            nb_capture_over: nb_capture_over,
            nb_capture_after: nb_capture_after,
            threshold: threshold
        });
        console.log("New parameters sent to the server:", {
            'type': 'penetrometry_new_params',
            'nb_capture_before': nb_capture_before,
            'nb_capture_over': nb_capture_over,
            'nb_capture_after': nb_capture_after,
            'threshold': threshold
        });
    }
});


function isInputsValid() {
    const nbCaptureBefore = document.getElementById('nbCaptureBefore');
    const nbCaptureOver = document.getElementById('nbCaptureOver');
    const nbCaptureAfter = document.getElementById('nbCaptureAfter');
    const threshold = document.getElementById('threshold');

    const nbCaptureBeforeError = document.getElementById('nbCaptureBeforeError');
    const nbCaptureOverError = document.getElementById('nbCaptureOverError');
    const nbCaptureAfterError = document.getElementById('nbCaptureAfterError');
    const thresholdError = document.getElementById('thresholdError');

    // Validation for nbCaptureBefore field (between 1 and 100)
    if (isNaN(nbCaptureBefore.value) || nbCaptureBefore.value < 1 || nbCaptureBefore.value > 100) {
        nbCaptureBeforeError.style.display = 'block';
        return false;
    } else {
        nbCaptureBeforeError.style.display = 'none';
    }

    // Validation for nbCaptureOver field (between 1 and 50)
    if (isNaN(nbCaptureOver.value) || nbCaptureOver.value < 1 || nbCaptureOver.value > 50) {
        nbCaptureOverError.style.display = 'block';
        return false;
    } else {
        nbCaptureOverError.style.display = 'none';
    }

    // Validation for nbCaptureAfter field (between 1 and 200)
    if (isNaN(nbCaptureAfter.value) || nbCaptureAfter.value < 1 || nbCaptureAfter.value > 200) {
        nbCaptureAfterError.style.display = 'block';
        return false
    } else {
        nbCaptureAfterError.style.display = 'none';
    }

    // Validation for threshold field (between 1 and 2000)
    if (isNaN(threshold.value) || threshold.value < 1 || threshold.value > 2000) {
        thresholdError.style.display = 'block';
        return false
    } else {
        thresholdError.style.display = 'none';
    }

    return true;
}
