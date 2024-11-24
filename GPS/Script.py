import os
import numpy as np
import pandas as pd
import gmplot

# Définir une fonction pour associer une couleur à chaque qualité
def get_color_for_quality(q):
    if q == 0:
        return None  # No fix
    elif q == 1:
        return 'blue'  # Autonomous GNSS fix
    elif q == 2:
        return 'green'  # Differential GNSS fix
    elif q == 4:
        return 'red'  # RTK Fixed
    elif q == 5:
        return 'orange'  # RTK Float
    elif q == 6:
        return 'purple'  # Estimated/Dead reckoning fix
    else:
        return 'black'  # Default color if undefined

# Fonction pour générer la carte à partir d'un fichier CSV
def generate_map_from_file(file_path, api_key):
    # Charger les données du fichier CSV
    columns = ['latitude', 'longitude', 'altitude', 'quality', 'hacc', 'vacc']
    data = pd.read_csv(file_path, header=None, names=columns)

    # Extraire les colonnes nécessaires
    latitudes = data['latitude']
    longitudes = data['longitude']
    quality = data['quality']  # La colonne de qualité (int8)
    hacc = data['hacc']  # Horizontal accuracy
    vacc = data['vacc']  # Vertical accuracy

    # Centrer la carte (utiliser les moyennes des latitudes et longitudes pour centrer)
    center_lat = latitudes.mean()
    center_lon = longitudes.mean()

    correction_value = 0.00003  # Ajustez cette valeur selon vos besoins
    longitudes = longitudes + correction_value  # Appliquer la correction

    correction_value = 0.00003  # Ajustez cette valeur selon vos besoins
    latitudes = latitudes + correction_value  # Appliquer la correction

    # Initialiser gmplot avec la clé API
    gmap = gmplot.GoogleMapPlotter(center_lat, center_lon, 30, apikey=api_key, map_type="satellite")

    # Tracer les itinéraires (lignes entre les points)
    for i in range(1, len(latitudes)):
        color = get_color_for_quality(quality[i])  # Obtenir la couleur pour la qualité du point
        if color:
            gmap.plot([latitudes[i-1], latitudes[i]], [longitudes[i-1], longitudes[i]], color=color, edge_width=4)

    # Générer le nom du fichier HTML en fonction du nom du fichier CSV
    base_filename = os.path.splitext(os.path.basename(file_path))[0]  # Enlever l'extension du nom de fichier
    html_file = f'map_visualization_{base_filename}.html'  # Nom du fichier de sortie

    # Sauvegarder la carte dans un fichier HTML
    gmap.draw(html_file)

    print(f"La carte pour {file_path} a été générée : {html_file}")

# Fonction principale pour traiter plusieurs fichiers dans un répertoire
def generate_maps_for_directory(directory_path, api_key):
    # Parcourir tous les fichiers dans le répertoire
    for filename in os.listdir(directory_path):
        if filename.endswith(".csv"):  # Vérifier si le fichier est un fichier CSV
            file_path = os.path.join(directory_path, filename)  # Obtenir le chemin complet du fichier
            generate_map_from_file(file_path, api_key)  # Générer la carte pour ce fichier

# Remplacer par votre clé API Google
google_api_key = "AIzaSyAo-m8Mk5lrClWDpeZLStCNDPbmjyPTkYg"

# Spécifier le répertoire où se trouvent vos fichiers CSV
directory_path = "./"  # Remplacez par le chemin vers votre répertoire

# Générer des cartes pour tous les fichiers dans le répertoire
generate_maps_for_directory(directory_path, google_api_key)

