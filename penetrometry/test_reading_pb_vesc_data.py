import os
import extractions_data_pb2  # Assurez-vous que ce fichier est dans votre chemin Python
import time

def read_protobuf_file(file_name):
    # Vérifier si le fichier existe
    if not os.path.exists(file_name):
        print(f"Le fichier {file_name} n'existe pas.")
        return

    # Ouvrir le fichier en mode binaire et lire les données
    with open(file_name, "rb") as f:
        while True:
            # Lire les données à partir du fichier
            data = f.read()
            if not data:
                break  # Si nous avons atteint la fin du fichier, sortir de la boucle

            # Créer une instance de ExtractionList
            extraction_list = extractions_data_pb2.ExtractionList()
            extraction_list.ParseFromString(data)  # Désérialiser les données

            # Afficher les données d'extraction
            for extraction in extraction_list.extractions:
                print("Extraction récupérée:")
                print(f"RPM: {list(extraction.rpm)}")  # Convertir en liste pour affichage
                print(f"Torque: {list(extraction.torque)}")  # Convertir en liste pour affichage
                print(f"Timestamps: {list(extraction.timestamp)}")  # Convertir en liste pour affichage
                print(f"Latitude: {extraction.latitude}")
                print(f"Longitude: {extraction.longitude}")
                print("-" * 30)  # Ligne de séparation pour les entrées

if __name__ == "__main__":
    # Exemple d'utilisation
    file_name = "datas/data_2024-10-23-16-47-25.pb"  # Remplacez par le nom de votre fichier
    read_protobuf_file(file_name)
