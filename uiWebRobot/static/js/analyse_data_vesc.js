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
                },
                {
                    id: "y_torque",
                    display : true,
                    position : "right",
                    beginAtZero: true,
                    scaleLabel: {
                        display: true,
                        labelString: 'Torque (N/M)',
                        fontColor: 'rgba(12, 234, 12, 1)',
                        fontSize: 14
                    },
                }
            ]
        }
    }
});

// Fonction pour mettre à jour les données du graphique
function updateGraph(dataArray) {
    // Itération à travers chaque point de données reçu (Array d'objets)
    dataArray.forEach((dataPoint) => {
        // Température, RPM et calcul du torque basé sur le courant
        const temp = dataPoint.temp_motor_filtered;
        const rpm = dataPoint.rpm;
        const current = dataPoint.avg_motor_current;
        const torque = current * 0.081; // Calcul du torque

        // Ajout des nouvelles données
        let currentTime = x_labels.length > 0 ? parseFloat(x_labels[x_labels.length - 1]) + 0.05 : 0; // Incrément de 0.05s pour chaque point
        x_labels.push(currentTime.toFixed(2)); // Arrondir à 2 chiffres

        data_temp.push(temp);
        data_rpm.push(rpm);
        data_torque.push(torque);

        // Limiter l'échelle X à 10 secondes (pour un total de 200 points de données)
        if (currentTime > 10) {
            x_labels.shift(); // Supprime l'ancienne étiquette (temps)
            data_temp.shift();   // Supprime les anciennes valeurs
            data_rpm.shift();
            data_torque.shift();
        }

        // Mise à jour du graphique
        myChart.update();
    });
}

// Écoute des données via Socket.IO
socketio.on('analyse_data_vesc', function(dataArray) {
    console.log(dataArray); // Vérification des données reçues
    updateGraph(dataArray); // Mise à jour du graphique avec les nouvelles données
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
