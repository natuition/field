let shouldUpdateRegisteredGraph = true;
let receivedData = [];

let registred_x_labels = []
let registred_data_temp =[]
let registred_data_rpm =[]
let registred_data_torque =[]

let nb_capture_before = 0;
let nb_capture = 0;
let nb_capture_after = 0;
let threshold = 0;

// Écoute des données du serveur via socketio
socketio.on('analyse_extraction_pattern', function(data) {
    console.log("Données d'extraction reçues :");
    console.log(data);

    receivedData.push(...data.captures) // Decompose the array and push it
    if(data.is_last_slice){
        if (shouldUpdateRegisteredGraph) {
            console.log("Actualisation du graphique");
            shouldUpdateRegisteredGraph = false;
    
            // Masquer le conteneur de chargement, afficher le bouton et les paramètres
            document.getElementById('loadingContainer').style.display = 'none';
            document.getElementById('paramContainer').style.display = 'block';
    
            // Mettre à jour les champs de paramétrage avec les données reçues
            nb_capture_before = data.detection_parameters.nb_capture_before;
            nb_capture = data.detection_parameters.nb_capture;
            nb_capture_after = data.detection_parameters.nb_capture_after;
            threshold = data.detection_parameters.threshold;
    
            document.getElementById('nbCaptureBefore').value = nb_capture_before;
            document.getElementById('nbCapture').value = nb_capture;
            document.getElementById('nbCaptureAfter').value = nb_capture_after;
            document.getElementById('threshold').value = threshold;
    
            // Mettre à jour le graphique avec les nouvelles données
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
        const currentTime = ((dataPoint.timestamp - initialTimestamp).toFixed(3))*1000;

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


// Registred charts settings
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
                        suggestedMin: -10,
                        suggestedMax: 10,
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
                        suggestedMin: -100,
                        suggestedMax: 100,
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
                    // Return the legend items with only the colored box
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
        // Erase the old graph
        myRegistredChart.data.datasets.forEach((dataset) => {
            dataset.data = [];  // Réinitialiser le tableau de données du dataset
        });
        
        // Vider les tableaux sources utilisés pour les données
        registred_x_labels = [];
        registred_data_torque = [];
        registred_data_rpm = [];
        registred_data_temp = [];
        receivedData = [];
        myRegistredChart.update();

        // Récupérer les valeurs actuelles des paramètres
        nb_capture_before = parseInt(document.getElementById('nbCaptureBefore').value);
        nb_capture = parseInt(document.getElementById('nbCapture').value);
        nb_capture_after = parseInt(document.getElementById('nbCaptureAfter').value);
        threshold = parseFloat(document.getElementById('threshold').value);

        // Envoyer les nouveaux paramètres au serveur pour la prochaine détection
        socketio.emit('data', {
            'type': 'trigger_analyse_vesc',
            'nb_capture_before': nb_capture_before,
            'nb_capture': nb_capture,
            'nb_capture_after': nb_capture_after,
            'threshold': threshold
        });
    }
});


function isInputsValid() {
    const nbCaptureBefore = document.getElementById('nbCaptureBefore');
    const nbCapture = document.getElementById('nbCapture');
    const nbCaptureAfter = document.getElementById('nbCaptureAfter');
    const threshold = document.getElementById('threshold');

    const nbCaptureBeforeError = document.getElementById('nbCaptureBeforeError');
    const nbCaptureError = document.getElementById('nbCaptureError');
    const nbCaptureAfterError = document.getElementById('nbCaptureAfterError');
    const thresholdError = document.getElementById('thresholdError');

    // Validation du champ nbCaptureBefore (entre 1 et 100)
    if (isNaN(nbCaptureBefore.value) || nbCaptureBefore.value < 1 || nbCaptureBefore.value > 100) {
        nbCaptureBeforeError.style.display = 'block';
        return false;
    } else {
        nbCaptureBeforeError.style.display = 'none';
    }

    // Validation du champ nbCapture (entre 1 et 50)
    if (isNaN(nbCapture.value) || nbCapture.value < 1 || nbCapture.value > 50) {
        nbCaptureError.style.display = 'block';
        return false;
    } else {
        nbCaptureError.style.display = 'none';
    }

    // Validation du champ nbCaptureAfter (entre 1 et 200)
    if (isNaN(nbCaptureAfter.value) || nbCaptureAfter.value < 1 || nbCaptureAfter.value > 200) {
        nbCaptureAfterError.style.display = 'block';
        return false
    } else {
        nbCaptureAfterError.style.display = 'none';
    }

    // Validation du champ threshold (entre 1 et 2000)
    if (isNaN(threshold.value) || threshold.value < 1 || threshold.value > 2000) {
        thresholdError.style.display = 'block';
        return false
    } else {
        thresholdError.style.display = 'none';
    }

    return true;
}


