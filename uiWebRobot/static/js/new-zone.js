const modalOverlay = document.getElementById('modal-overlay');
const featureSelect = document.getElementById('feature-select');
const featureForm = document.getElementById('feature-form');
const btnCancel = document.getElementById('btn-cancel');

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
    console.error(`Erreur lors de la lecture du fichier ${zip.name} :`, error);
    throw new Error(`Le fichier '${zip.name}' est invalide.`);
  }
};

const parseGeoJSON = (jsonStrings) => {
  const parsedFiles = [];

  for (const jsonString of jsonStrings) {
    try {
      parsedFiles.push(JSON.parse(jsonString));
    } catch (error) {
      console.warn("Erreur de désérialisation sur un fichier, on passe :", error);
    }
  }

  return parsedFiles;
};

const filterFeatures = (collections, predicate) => {
  const features = [];
  for (const collection of collections) {
    try {
      const validFeatures = collection.features.filter(predicate);
      features.push(...validFeatures);
    } catch (error) {
      console.warn("Collection de features invalide, on passe :", error);
      continue;
    }
  }

  return features;
};

const openModal = (features) => {
  featureSelect.innerHTML = ''; // Nettoyage des options précédentes
  currentFeatures = features;

  if (features.length === 0) {
    const defaultOption = document.createElement('option');
    defaultOption.value = "";
    defaultOption.textContent = "Aucune entité zone disponible";
    featureSelect.appendChild(defaultOption);
    featureSelect.disabled = true;
  } else {
    featureSelect.disabled = false;

    features.forEach((feature, index) => {
      const name = feature.properties?.Name || `Feature #${index + 1}`;

      const option = document.createElement('option');
      option.value = index; // L'index sert d'identifiant
      option.textContent = name;

      featureSelect.appendChild(option);
    });
  }

  modalOverlay.classList.remove('hidden');
};

const closeModal = () => {
  console.log("Cancel")
  modalOverlay.classList.add('hidden');
  currentFeatures = [];
};

const processSelectedFeature = (feature) => {
  console.log("Entité sélectionnée :", feature);

  const featureName = feature.properties?.Name || "Sans nom";

  socketio.emit("data", {
    type: "create_field",
    value: feature,
  });
};

const onFormSubmit = (e) => {
  e.preventDefault();

  if (currentFeatures.length === 0) return;

  const selectedIndex = Number(featureSelect.value);

  const selectedFeature = currentFeatures[selectedIndex];

  closeModal();

  processSelectedFeature(selectedFeature);
};

featureForm.addEventListener('submit', onFormSubmit);
btnCancel.addEventListener('click', closeModal);

