// Once the DOM is fully loaded
document.addEventListener('DOMContentLoaded', function () {
    // Variable to store the gps quality, default is 0
    let lastGPSQuality = 0;

    // Variable to store the gps quality description, default is "no_gps"
    let qualityDescription = (ui_languages["no_gps"])[ui_language];

    // Open connexion with socket '/gps'
    const socketGPS = io.connect('http://' + document.domain + ':' + location.port + '/gps');

    // Init the HTML content
    const qualityTextElement = document.getElementById('gps_quality_indicator_text');
    if (qualityTextElement){
        qualityTextElement.textContent = qualityDescription;
    }

    // Listenning event on 'updateGPSQuality'
    socketGPS.on('updateGPSQuality', function (gps_quality_value) {
        console.log(gps_quality_value);
        // Convert the value to a integer
        gps_quality_value = parseInt(gps_quality_value, 10);
        if (isNaN(gps_quality_value)) {
            console.error('Erreur : La valeur de qualité GPS n\'est pas un entier valide.');
            return;
        }

        // Modify only if new quality value isn't the same as old quality value
        if (gps_quality_value != lastGPSQuality) {
            lastGPSQuality = gps_quality_value;

            // Change the description of the gps quality
            switch (gps_quality_value) {
                case 0:
                    // 0 = Pas de correction valide
                    qualityDescription = (ui_languages["no_gps"])[ui_language];
                    updateGlobalGpsQuality("no_gps")
                    break;
                case 1:
                    // 1 = Fix GPS standard
                    qualityDescription = (ui_languages["only_gps"])[ui_language];
                    updateGlobalGpsQuality("only_gps")
                    break;
                case 2:
                    // 2 = Correction différentielle (DGNSS)
                    qualityDescription = (ui_languages["only_gps"])[ui_language];
                    updateGlobalGpsQuality("only_gps")
                    break;
                case 4:
                    // 4 = RTK fixe
                    qualityDescription = (ui_languages["gps_with_rtk"])[ui_language];
                    updateGlobalGpsQuality("gps_with_rtk")
                    break;
                case 5:
                    // 5 = RTK flottant
                    qualityDescription = (ui_languages["only_gps"])[ui_language];
                    updateGlobalGpsQuality("only_gps")
                    break;
                case 6:
                    // 6 = Navigation inertielle (INS)
                    qualityDescription = (ui_languages["no_gps"])[ui_language];
                    updateGlobalGpsQuality("no_gps")
                    break;
                default:
                    // Indicateur inconnu
                    qualityDescription = (ui_languages["no_gps"])[ui_language];
                    updateGlobalGpsQuality("no_gps")
            }

            // Update HTML content
            qualityTextElement.textContent = qualityDescription;
            console.log(`Qualité GPS mise à jour : ${qualityDescription}`);
        }
    });
});

// Custom event for updating the gps quality in other file
function updateGlobalGpsQuality(newQuality) {
    const gpsEvent = new CustomEvent("globalGpsQualityUpdated", { detail: { quality: newQuality } });
    window.dispatchEvent(gpsEvent);
}