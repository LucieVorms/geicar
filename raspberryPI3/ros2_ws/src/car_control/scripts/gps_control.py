#!/usr/bin/env python3


import csv
from rclpy.node import Node
from interfaces.msg import Gnss  # Import du message correct

class GnssListener(Node):
    def __init__(self):
        super().__init__('gnss_listener')
        self.subscription = self.create_subscription(
            Gnss,
            '/gnss_data',
            self.listener_callback,
            10  # Taille du buffer
        )
        self.itinerary = [
            (48.8566, 2.3522),  # Paris
            (48.8584, 2.2945),  # Tour Eiffel
            (48.8589, 2.3176),  # Champ de Mars
        ]
        self.current_target_index = 0  # Index du point de l'itinéraire à suivre
        self.file = open('gnss_data.csv', mode='w', newline='')  # Ouverture du fichier CSV en mode écriture
        self.writer = csv.writer(self.file)
        self.writer.writerow(['latitude', 'longitude', 'altitude', 'quality', 'hacc', 'vacc'])  # En-têtes CSV
    
    def listener_callback(self, msg):
        # Récupère les coordonnées actuelles du véhicule
        current_lat = msg.latitude
        current_lon = msg.longitude
        current_alt = msg.altitude
        current_quality = msg.quality
        current_hacc = msg.hacc
        current_vacc = msg.vacc
        
        # Enregistrer les données GPS reçues dans le fichier CSV
        self.writer.writerow([current_lat, current_lon, current_alt, current_quality, current_hacc, current_vacc])

        # Vérifie la distance par rapport au point de l'itinéraire actuel
        target_lat, target_lon = self.itinerary[self.current_target_index]
        distance = haversine(current_lat, current_lon, target_lat, target_lon)
        
        self.get_logger().info(f"Distance to target: {distance:.2f} km")
        
        # Si la distance est inférieure à 10 mètres, passer au prochain point
        if distance < 0.01:  # 0.01 km = 10 mètres
            self.get_logger().info(f"Arrived at target {self.current_target_index + 1}")
            self.current_target_index += 1
            
            # Si on a terminé l'itinéraire
            if self.current_target_index >= len(self.itinerary):
                self.get_logger().info("End of itinerary reached!")
                self.current_target_index = 0  # Retour au début si nécessaire

    def __del__(self):
        # Fermeture du fichier CSV à la fin
        self.file.close()

