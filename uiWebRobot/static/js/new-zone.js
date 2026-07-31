const modalOverlay = document.getElementById('modal-overlay');
const featureSelect = document.getElementById('feature-select');
const featureForm = document.getElementById('feature-form');
const btnCancel = document.getElementById('btn-cancel');

const PROPERTY_KEY_AS_ZONE_NAME = "Name" // Default with SW Maps

let currentFeatures = [];

const extractFiles = async (zip, extension, outputFormat = "string") => {
  const unzippedFiles = [];
  const ZIP = new JSZip();

  try {
    const zipContent = await ZIP.loadAsync(zip);

    for (const [relativePath, entry] of Object.entries(zipContent.files)) {
      if (entry.dir || !relativePath.endsWith(extension)) {
        continue;
      }

      const content = await entry.async(outputFormat);
      unzippedFiles.push(content);
    }

    return unzippedFiles;
  } catch (error) {
    console.error("Error during file extraction:", error)
    throw new Error((ui_languages["zone__err_invalid_file"])[ui_language]);
  }
};

const parseGeoJSON = (jsonStrings) => {
  const parsedFiles = [];

  for (const jsonString of jsonStrings) {
    try {
      parsedFiles.push(JSON.parse(jsonString));
    } catch (error) {
      console.warn("Error while deserializing file, skipping.", error)
    }
  }

  return parsedFiles;
};

const filterFeatures = (collections, predicate) => {
  const features = [];
  for (const collection of collections) {
    try {
      const validFeatures = collection.features.filter(predicate);
      // TODO: assertValidFeatureCollection(collection)
      features.push(...validFeatures);
    } catch (error) {
      console.warn("Invalid feature collection, skipping.", error)
      continue;
    }
  }

  return features;
};

const openModal = (features) => {
  featureSelect.innerHTML = '';
  currentFeatures = features;

  if (features.length === 0) {
    const defaultOption = document.createElement('option');
    defaultOption.value = "";
    defaultOption.textContent = (ui_languages["zone__default_option"])[ui_language];
    featureSelect.appendChild(defaultOption);
    featureSelect.disabled = true;
    featureSelect.removeAttribute('required');
  } else {
    featureSelect.disabled = false;

    features.forEach((feature, index) => {
      const label = feature.properties?.[PROPERTY_KEY_AS_ZONE_NAME] || `${(ui_languages["zone__fallback_label"])[ui_language]}_${index + 1} `;

      const option = document.createElement('option');
      option.value = String(index);
      option.textContent = label;

      if (index === 0) {
        option.selected = true; // Forcer la sélection du premier élément pour iOS WebKit
      }

      featureSelect.appendChild(option);
    });

    featureSelect.value = "0"; // Forcer le DOM à enregistrer la valeur "0"
  }

  modalOverlay.classList.remove('hidden');
};

const onFormSubmit = (e) => {
  e.preventDefault();

  if (currentFeatures.length === 0) return;

  // Sécurité pour iOS : si featureSelect.value est vide ou non-numérique
  const rawValue = featureSelect.value;
  const selectedIndex = rawValue !== "" ? Number(rawValue) : 0;

  const selectedFeature = currentFeatures[selectedIndex];

  if (!selectedFeature) {
    console.error("Aucune feature trouvée à l'index :", selectedIndex);
    return;
  }

  closeModal();
  processSelectedFeature(selectedFeature);
};

const closeModal = () => {
  console.warn("User cancelled zone selection, closing modal.")
  modalOverlay.classList.add('hidden');
  currentFeatures = [];
};

const processSelectedFeature = (feature) => {
  console.log("Selected feature:", feature)

  const featureName = feature.properties?.[PROPERTY_KEY_AS_ZONE_NAME] || (ui_languages["zone__fallback_name"])[ui_language]

  //sendInfo("info_create_field_with_navx", (ui_languages["zone__loading_state"])[ui_language])

  socketBroadcast_.emit('popup_modal', {
    message_name: "info_create_field_with_navx",
    message: (ui_languages["zone__loading_state"])[ui_language],
    type_alert: "alert-success"
  });

  socketio.emit("data", {
    type: "create_field",
    value: feature,
  });
};

featureForm.addEventListener('submit', onFormSubmit);
btnCancel.addEventListener('click', closeModal);

