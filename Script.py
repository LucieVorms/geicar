import os
import pandas as pd
import gmplot
import math


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


# Fonction pour interpoler des courbes entre les points
def interpolate_curve(lat1, lon1, lat2, lon2, steps=20):
    """Génère des points intermédiaires entre deux coordonnées."""
    latitudes = []
    longitudes = []
    for t in range(steps + 1):
        fraction = t / steps
        lat = lat1 + (lat2 - lat1) * fraction
        lon = lon1 + (lon2 - lon1) * fraction
        latitudes.append(lat)
        longitudes.append(lon)
    return latitudes, longitudes


# Fonction pour générer la carte à partir d'un fichier CSV
def generate_route_map(file_path, api_key):
    # Charger les données du fichier CSV
    columns = ['latitude', 'longitude', 'altitude', 'quality', 'hacc', 'vacc']
    data = pd.read_csv(file_path, header=None, names=columns)

    # Extraire les colonnes nécessaires
    latitudes = data['latitude']
    longitudes = data['longitude']
    qualities = data['quality']

    # Centrer la carte
    center_lat = latitudes.mean()
    center_lon = longitudes.mean()

    # Initialiser gmplot avec la clé API
    gmap = gmplot.GoogleMapPlotter(center_lat, center_lon, 15, apikey=api_key, map_type="satellite")

    # Tracer les courbes entre les waypoints
    for i in range(len(latitudes) - 1):
        lat1, lon1 = latitudes.iloc[i], longitudes.iloc[i]
        lat2, lon2 = latitudes.iloc[i + 1], longitudes.iloc[i + 1]

        # Interpoler les points pour créer des courbes
        curve_latitudes, curve_longitudes = interpolate_curve(lat1, lon1, lat2, lon2)

        # Tracer une ligne courbée
        gmap.plot(curve_latitudes, curve_longitudes, color='blue', edge_width=2)

    # Ajouter des marqueurs pour chaque waypoint
    for i in range(len(latitudes)):
        lat, lon, q = latitudes.iloc[i], longitudes.iloc[i], qualities.iloc[i]
        color = get_color_for_quality(q)
        if color:
            gmap.marker(lat, lon, color=color, title=f"Waypoint {i + 1}: Quality {q}, Lat {lat}, Lon {lon}")

    # Générer le fichier HTML
    base_filename = os.path.splitext(os.path.basename(file_path))[0]
    html_file = f'route_map_{base_filename}.html'
    gmap.draw(html_file)

    print(f"La carte pour {file_path} a été générée : {html_file}")


# Remplacer par votre clé API Google
google_api_key = "AIzaSyAo-m8Mk5lrClWDpeZLStCNDPbmjyPTkYg"

# Spécifier le chemin du fichier CSV
file_path = "output_filtered.csv"  # Remplacez par votre fichier CSV

# Générer la carte
generate_route_map(file_path, google_api_key)
