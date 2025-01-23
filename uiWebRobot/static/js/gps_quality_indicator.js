// Once the DOM is fully loaded
document.addEventListener('DOMContentLoaded', function () {
    // Variable to store the gps quality, default is 0
    let lastGPSQuality = 0;

    // Variable to store the gps quality description, default is "no_gps"
    let qualityDescription = (ui_languages["no_gps"])[ui_language];

    // Timeout handler for resetting GPS quality
    let gpsQualityTimeout = null;

    // Open connexion with socket '/gps'
    const socketGPS = io.connect('http://' + document.domain + ':' + location.port + '/gps');

    // Init the HTML content
    const qualityTextElement = document.getElementById('gps_quality_indicator_text');
    if (qualityTextElement) {
        qualityTextElement.textContent = qualityDescription;
        $("#gps_quality_indicator_text").parent().css("background-color", "#FF3232");
    }

    // Listenning event on 'updateGPSQuality'
    socketGPS.on('updateGPSQuality', function (gps_quality_value) {
        //console.log(gps_quality_value);
        // Convert the value to a integer
        gps_quality_value = parseInt(gps_quality_value, 10);
        if (isNaN(gps_quality_value)) {
            console.error('Error : The GPS quality value is not a valid integer.');
            return;
        }

        // Reset to 'no_gps' if no update for 5 seconds
        if (gpsQualityTimeout) {
            clearTimeout(gpsQualityTimeout);
        }
        gpsQualityTimeout = setTimeout(function () {
            //console.warn("No GPS quality update received for 5 seconds. Reset to 'no_gps'.");
            lastGPSQuality = 0;
            qualityDescription = (ui_languages["no_gps"])[ui_language];
            updateGlobalGpsQuality("no_gps");
        }, 5000);

        // Modify only if new quality value isn't the same as old quality value
        if (gps_quality_value != lastGPSQuality) {
            lastGPSQuality = gps_quality_value;
            background_color = "#58b166"

            // Change the description of the gps quality
            switch (gps_quality_value) {
                case 0: // 0 = Pas de correction valide
                case 6: // 6 = Navigation inertielle (INS)
                    qualityDescription = (ui_languages["no_gps"])[ui_language];
                    updateGlobalGpsQuality("no_gps");
                    background_color = "#FF3232"
                    break;
                case 1: // 1 = Fix GPS standard
                case 2: // 2 = Correction différentielle (DGNSS)
                case 5: // 5 = RTK flottant
                    qualityDescription = (ui_languages["only_gps"])[ui_language];
                    updateGlobalGpsQuality("only_gps");
                    background_color = "#FF9532"
                    break;
                case 4: // 4 = RTK fixe
                    qualityDescription = (ui_languages["gps_with_rtk"])[ui_language];
                    updateGlobalGpsQuality("gps_with_rtk");
                    break;
                default:
                    //console.warn(`GPS quality value (${gps_quality_value}) not recognized. Falling back to 'no_gps'.`);
                    qualityDescription = (ui_languages["no_gps"])[ui_language];
                    updateGlobalGpsQuality("no_gps");
                    background_color = "#FF3232"
                    break;
            }

            // Update HTML content
            qualityTextElement.textContent = qualityDescription;
            //console.log(`GPS quality updated : ${qualityDescription} <${gps_quality_value}>`);
            $("#gps_quality_indicator_text").parent().css("background-color", background_color);

        }
    });
});

// Custom event for updating the gps quality in other file
function updateGlobalGpsQuality(newQuality) {
    const gpsEvent = new CustomEvent("globalGpsQualityUpdated", { detail: { quality: newQuality } });
    window.dispatchEvent(gpsEvent);
}