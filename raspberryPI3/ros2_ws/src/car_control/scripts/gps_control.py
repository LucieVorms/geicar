#!/usr/bin/env python3
import csv
from math import radians, cos, sin, sqrt, atan2
import rclpy
from rclpy.node import Node
from interfaces.msg import Gnss  # Import du message correct
from interfaces.msg import GnssStatus
class GnssListener(Node):
    def __init__(self):
        super().__init__('gnss_listener')
        self.subscription = self.create_subscription(
            Gnss,
            '/gnss_data',
            self.listener_callback,
            10  # Taille du buffer
        )
        
        self.publisher = self.create_publisher(GnssStatus, '/gnss_status', 10)  
        # Charger les points GPS depuis un fichier CSV
        self.itinerary = self.load_itinerary_from_csv('gnss_data.csv')
        
        self.current_target_index = 0  # Index du point de l'itinéraire à suivre
        
    def load_itinerary_from_csv(self, file_name):
        # Charger les points GPS depuis un fichier CSV
        itinerary = []
        try:
            with open(file_name, mode='r') as file:
                reader = csv.reader(file)
                for row in reader:
                    if row:  # Ignorer les lignes vides
                        try:
                            # Extraire uniquement la latitude et la longitude
                            lat, lon = float(row[0]), float(row[1])
                            itinerary.append((lat, lon))
                        except ValueError as e:
                            self.get_logger().error(f"Invalid data in row {row}: {e}")
        except FileNotFoundError:
            self.get_logger().error(f"File {file_name} not found.")
        except Exception as e:
            self.get_logger().error(f"Error reading {file_name}: {e}")
        return itinerary
    
    def listener_callback(self, msg):
        # Récupère les coordonnées actuelles du véhicule
        current_lat = msg.latitude
        current_lon = msg.longitude
        current_alt = msg.altitude
        
  
        target_lat, target_lon = self.itinerary[self.current_target_index]
        final_lat, final_lon = self.itinerary[-1]  # Dernier point de l'itinéraire
        distance = self.haversine(current_lat, current_lon, target_lat, target_lon)
        status_msg = GnssStatus()
        status_msg.current_latitude = current_lat
        status_msg.current_longitude = current_lon
        status_msg.current_altitude = current_alt
        status_msg.target_latitude = target_lat
        status_msg.target_longitude = target_lon
        status_msg.distance_to_target = distance
        #self.get_logger().info(f"Next target: Latitude {target_lat}, Longitude {target_lon}")
        #self.get_logger().info(f"Final target: Latitude {final_lat}, Longitude {final_lon}")
        
        # Calculer la distance jusqu'au prochain point
        
        #self.get_logger().info(f"Distance to next target: {distance:.5f} km")
        
        # Si la distance est inférieure à 10 mètres, passer au prochain point
        if distance < 0.0001:  # 0.0001 km = 10 cm
            
            self.current_target_index += 1
            
            # Si on a terminé l'itinéraire
            if self.current_target_index >= len(self.itinerary):
                status_msg.status_message = "End of itinerary reached!"
                #self.get_logger().info("End of itinerary reached!")
                self.current_target_index = 0  # Retour au début si nécessaire
            else : 
                #self.get_logger().info(f"Arrived at target {self.current_target_index + 1}")
                status_msg.status_message = f"Arrived at target {self.current_target_index}. Moving to next target."
        else : 
            status_msg.status_message = f"Navigating to target {self.current_target_index}."
            #self.get_logger().info(f"Starting over. Next target: {self.itinerary[self.current_target_index]}")
        self.publisher.publish(status_msg)
    def haversine(self, lat1, lon1, lat2, lon2):
        # Convertir les coordonnées en radians
        lat1 = radians(lat1)
        lon1 = radians(lon1)
        lat2 = radians(lat2)
        lon2 = radians(lon2)
        # Calcul de la différence des latitudes et longitudes
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        # Haversine formula
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        R = 6371  # Rayon de la Terre en kilomètres
        return R * c  # Distance en kilomètres
def main(args=None):
    rclpy.init(args=args)
    gnss_listener = GnssListener()
    rclpy.spin(gnss_listener)
    gnss_listener.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
