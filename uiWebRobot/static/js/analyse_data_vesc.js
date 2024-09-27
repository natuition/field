let shouldUpdateRegisteredGraph = true;

let registred_x_labels = []
let registred_data_temp =[]
let registred_data_rpm =[]
let registred_data_torque =[]


let capturesFrequency = 0;
let rpmThreshold;
let captureCountOverThreshold = 0;
let captureCountBeforeDetection = 0;
let captureCountAfterDetection = 0;

// Listenning of the socketio channel
socketio.on('analyse_extraction_pattern', function(data) {
    console.log("Données d'extraction reçues :");
    console.log(data);
    
    if (shouldUpdateRegisteredGraph) {
        console.log("Actualisation du graphique");
        shouldUpdateRegisteredGraph = false;
        
        // Masquer le conteneur de chargement et afficher le bouton
        document.getElementById('loadingContainer').style.display = 'none';
        document.getElementById('newSignalButton').style.display = 'block';
        
        // Stocker les paramètres dans des variables globales
        capturesFrequency = data.detection_parameters.captures_frequency;
        rpmThreshold = data.detection_parameters.rpm_threshold;
        captureCountOverThreshold = data.detection_parameters.capture_count_over_threshold;
        captureCountBeforeDetection = data.detection_parameters.capture_count_before_detection;
        captureCountAfterDetection = data.detection_parameters.capture_count_after_detetion;

        // Mettre à jour le texte dans le HTML
        document.getElementById('captures_frequency').innerText = capturesFrequency;
        document.getElementById('rpm_threshold').innerText = rpmThreshold;
        document.getElementById('capture_count_over_threshold').innerText = captureCountOverThreshold;
        document.getElementById('capture_count_before_detection').innerText = captureCountBeforeDetection;
        document.getElementById('capture_count_after_detection').innerText = captureCountAfterDetection;

        // Mettre à jour le graphique avec les nouvelles données
        updateRegistredGraph(data.captures);
    }
});

function updateRegistredGraph(dataArray) {

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
                label: (ui_languages["torque"])[ui_language],
                data: registred_data_torque,
                borderColor: 'rgba(12, 234, 12, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_torque',
            },
            {
                label: (ui_languages["speed"])[ui_language],
                data: registred_data_rpm,
                borderColor: 'rgba(123, 12, 12, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_rpm',
            },
            {
                label: (ui_languages["temperature"])[ui_language],
                data: registred_data_temp,
                borderColor: 'rgba(75, 192, 192, 1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'y_temp',
            }
        ]
    },
    options: {
        scales: {
            xAxes: [{
                position: 'bottom',
                scaleLabel: {
                    display: true,
                    labelString: (ui_languages["time"])[ui_language],
                    fontSize: 14
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
                value: 500,
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
    shouldUpdateRegisteredGraph = true;
    this.style.display = 'none'; // Disable the button
    document.getElementById('loadingContainer').style.display = 'block';
    // Erase the old graph
    myRegistredChart.data.labels.pop()
    myRegistredChart.data.datasets.forEach((dataset) => {
        dataset.data.pop();
    });
    registred_x_labels.length = 0;
    registred_data_temp.length = 0;
    registred_data_rpm.length = 0;
    registred_data_torque.length = 0;
    myRegistredChart.update();

});