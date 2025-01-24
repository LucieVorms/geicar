#!/usr/bin/env python3
import csv
from math import radians, degrees, cos, sin, sqrt, atan2
import rclpy
from rclpy.node import Node
from interfaces.msg import Gnss 
from interfaces.msg import GnssStatus
from interfaces.msg import EnougthSpace

class GnssListener(Node):
    def __init__(self):
        super().__init__('gnss_listener')
        
        # Create a subscription to listen to GNSS data from the topic '/gnss_data'
        self.subscription = self.create_subscription(
            Gnss,
            '/gnss_data',
            self.gnss_callback,
            10  # Buffer size
        )

        self.subscription = self.create_subscription(
            EnougthSpace,
            '/enough_width_space',
            self.lidar_callback,
            10  # Buffer size
        )

        
        # Create a publisher to send GNSS status messages
        self.publisher = self.create_publisher(GnssStatus, '/gnss_status', 10)
        
        # Load the GPS points from the itinerary CSV file
        self.itinerary = self.load_itinerary_from_csv('gnss_data_test.csv')

        if not self.itinerary:
            self.get_logger().error("No valid GPS points loaded from the itinerary!")
            rclpy.shutdown()
        
        self.current_target_index = 0  # Start at the first target in the itinerary
        self.previous_lat = None
        self.previous_lon = None
        self.lookahead_distance = 75
        # Define fixed offsets for latitude and longitude corrections (adjust as needed)
        self.lat_offset = 0.000025  # Example offset in latitude
        self.lon_offset = 0.000025  # Example offset in longitude

        self.angle = None
        self.space = None

    def load_itinerary_from_csv(self, file_name):
        # Load GPS points from a CSV file
        itinerary = []
        try:
            with open(file_name, mode='r') as file:
                reader = csv.reader(file)
                for row in reader:
                    if row:  # Skip empty rows
                        try:
                            # Extract latitude and longitude from the row
                            lat, lon = float(row[0]), float(row[1])
                            itinerary.append((lat, lon))
                        except ValueError as e:
                            self.get_logger().error(f"Invalid data in row {row}: {e}")
        except FileNotFoundError:
            self.get_logger().error(f"File {file_name} not found.")
        except Exception as e:
            self.get_logger().error(f"Error reading {file_name}: {e}")
        return itinerary
    
    def lidar_callback(self, msg):
        self.space = msg.found
        self.angle_lidar = msg.angle

    def gnss_callback(self, msg):
        # Retrieve the current vehicle coordinates and apply the offset

        if msg.quality == 1 or msg.quality == 2:
            current_lat = msg.latitude
            current_lon = msg.longitude
        else:
            current_lat = msg.latitude + self.lat_offset  # Apply latitude correction
            current_lon = msg.longitude + self.lon_offset  # Apply longitude correction
        
        target_lat, target_lon = self.itinerary[self.current_target_index]

        if self.previous_lat is not None and self.previous_lon is not None:
           # Calculate the distance to the target using the Haversine formula

            reduced_itinerary = self.itinerary[self.current_target_index:]
            current_position = (current_lat, current_lon)

            pursuit_point, updated_index = self.calculate_pursuit_point(reduced_itinerary, current_position, self.lookahead_distance)

            new_global_index = self.current_target_index + updated_index
            if new_global_index > self.current_target_index:
                self.current_target_index = new_global_index


            status_msg = GnssStatus()
            if self.current_target_index == len(self.itinerary) - 1:
                final_target = self.itinerary[-1]
                if self.haversine(current_position[0], current_position[1], final_target[0], final_target[1]) < 50:
                    self.get_logger().info("Destination reached.")
                    status_msg.stop_following = True

            distance = self.haversine(current_lat, current_lon, target_lat, target_lon)
            
            # Calculate the direction (bearing) towards the current position
            current_direction = self.calculate_bearing(self.previous_lat, self.previous_lon, current_lat, current_lon)
            
            # Calculate the direction (bearing) towards the new point
            target_direction = self.calculate_bearing(current_lat, current_lon, pursuit_point[0],pursuit_point[1])  # Example target point
            
            # Calculate the angle difference between current and target directions
            angle_difference = self.calculate_angle_difference(current_direction, target_direction)
            
            if self.space == True:
                if self.angle_lidar > 0.0:
                    if self.angle_lidar > 1.0:
                        angle_difference = degrees(self.angle_lidar)+10
                    else: 
                        angle_difference = degrees(self.angle_lidar)-10
                else:
                    max_angle_difference = 35.0
                    min_angle_difference = -35.0
                    angle_difference = max(min(angle_difference, max_angle_difference), min_angle_difference)

            
            if(distance < self.lookahead_distance):
                status_msg.status_message = f"Arrived at target {self.current_target_index}. Moving to next target."
                status_msg.stop_following = False
            else: 
                status_msg.status_message = f"Navigating to target {self.current_target_index}."
                status_msg.stop_following = False

            if angle_difference > 10:
                status_msg.direction_message = f"Turn Right by {angle_difference:.2f} degrees"
            elif angle_difference < -10:
                status_msg.direction_message = f"Turn Left by {abs(angle_difference):.2f} degrees"
            else:
                status_msg.direction_message = "Go Straight"

            # Create a new status message
            
            status_msg.current_latitude = current_lat
            status_msg.current_longitude = current_lon
            status_msg.target_latitude = target_lat
            status_msg.target_longitude = target_lon
            status_msg.distance_to_target = distance
            status_msg.pursuit_latitude = pursuit_point[0]
            status_msg.pursuit_longitude = pursuit_point[1]
            status_msg.current_direction = float(current_direction)
            status_msg.target_direction = target_direction
            status_msg.turn_angle = angle_difference


            self.publisher.publish(status_msg)
            
        self.previous_lat = current_lat
        self.previous_lon = current_lon


    def haversine(self, lat1, lon1, lat2, lon2):
        # Convert coordinates to radians
        lat1 = radians(lat1)
        lon1 = radians(lon1)
        lat2 = radians(lat2)
        lon2 = radians(lon2)
        
        # Calculate the differences in latitudes and longitudes
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        # Apply the Haversine formula
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        R = 637100000  # Earth radius in kilometers
        return R * c  # Return distance in kilometers

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate the bearing (angle) between two GPS points."""
        lat1, lat2 = map(radians, [lat1, lat2])
        delta_lon = radians(lon2 - lon1)
        x = sin(delta_lon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon)
        return (degrees(atan2(x, y)) + 360) % 360

    def calculate_angle_difference(self, current_direction, target_direction):
        """Calculate the difference in angle between the current and target directions."""
        angle_difference = (target_direction - current_direction + 360) % 360
        if angle_difference > 180:
            angle_difference -= 360
        return angle_difference

    def calculate_pursuit_point(self,path, current_position, lookahead_distance):
        for i in range(len(path) - 1):
            start = current_position
            end = path[i]
            segment_length = self.haversine(current_position[0], current_position[1], end[0], end[1])
            if segment_length >= lookahead_distance:
                ratio = lookahead_distance / segment_length
                pursuit_point_lat = start[0] + ratio * (end[0] - start[0])
                pursuit_point_lon = start[1] + ratio * (end[1] - start[1])
                
                return (pursuit_point_lat, pursuit_point_lon),i
        start = current_position
        end = path[-1]
        segment_length = self.haversine(start[0], start[1], end[0], end[1])
        if segment_length >= lookahead_distance:
            ratio = lookahead_distance / segment_length
            pursuit_point_lat = start[0] + ratio * (end[0] - start[0])
            pursuit_point_lon = start[1] + ratio * (end[1] - start[1])
            return (pursuit_point_lat, pursuit_point_lon), len(path) - 1    
        return path[-1],len(path)-1

def main(args=None):
    rclpy.init(args=args)
    gnss_listener = GnssListener()
    rclpy.spin(gnss_listener)
    gnss_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
