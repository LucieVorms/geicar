import pandas as pd
import math
import gmplot

start_index = 180  # Starting point index
end_index = 179    # Destination point index

# Calculate the bearing (angle) between two GPS points
def calculate_bearing(lat1, lon1, lat2, lon2):
    delta_lon = math.radians(lon2 - lon1)
    lat1, lat2 = math.radians(lat1), math.radians(lat2)
    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360  # Normalize to 0-360 degrees

# Calculate the average bearing from a list of points
def calculate_average_bearing(points):
    bearings = []
    for i in range(len(points) - 1):
        lat1, lon1 = points.iloc[i]['latitude'], points.iloc[i]['longitude']
        lat2, lon2 = points.iloc[i + 1]['latitude'], points.iloc[i + 1]['longitude']
        bearings.append(calculate_bearing(lat1, lon1, lat2, lon2))
    return sum(bearings) / len(bearings)

# Load GPS data from the CSV file
file_path = "./Detect_Direction/GPS/tetest77.csv"
columns = ['latitude', 'longitude', 'altitude', 'quality', 'hacc', 'vacc']
data = pd.read_csv(file_path, header=None, names=columns)

# Apply corrections to GPS data
data['longitude'] += 0.000025
data['latitude'] += 0.000026

# Handle normal and reversed index cases
if start_index < end_index:
    # Normal order
    print("Case 1: Normal order GEI -> STPI")
    current_points = data.iloc[max(0, start_index - 1):start_index + 1]
    next_point = data.iloc[end_index]
else:
    # Reversed order
    print("Case 2: Reversed order STPI -> GEI")
    data = data.iloc[::-1]
    start_index, end_index = len(data) - start_index - 1, len(data) - end_index - 1
    current_points = data.iloc[max(0, start_index - 1):start_index + 1]
    next_point = data.iloc[end_index]

# Calculate current direction (average of 2 points)
current_direction = calculate_average_bearing(current_points)

# Calculate target direction (current position to next position)
current_lat, current_lon = data.iloc[start_index]['latitude'], data.iloc[start_index]['longitude']
next_lat, next_lon = data.iloc[end_index]['latitude'], data.iloc[end_index]['longitude']
target_direction = calculate_bearing(current_lat, current_lon, next_lat, next_lon)

# Calculate angle difference and normalize to -180 to 180
angle_difference = (target_direction - current_direction + 360) % 360
if angle_difference > 180:
    angle_difference -= 360

# Determine the action
if angle_difference > 5:
    action = "Turn Right"
elif angle_difference < -5:
    action = "Turn Left"
else:
    action = "Go Straight"

# Print the results
print(f"Current Position: Latitude = {current_lat}, Longitude = {current_lon}")
print(f"Next Position: Latitude = {next_lat}, Longitude = {next_lon}")
print(f"Current Direction: {current_direction:.2f}°")
print(f"Target Direction: {target_direction:.2f}°")
print(f"Angle Difference: {angle_difference:.2f}°")
print(f"Action: {action}")

# Visualize the map
center_lat = (current_lat + next_lat) / 2
center_lon = (current_lon + next_lon) / 2
gmap = gmplot.GoogleMapPlotter(center_lat, center_lon, 20, apikey="AIzaSyAo-m8Mk5lrClWDpeZLStCNDPbmjyPTkYg")

# Plot the full route
latitudes = data['latitude'].tolist()
longitudes = data['longitude'].tolist()
gmap.plot(latitudes, longitudes, color="orange", edge_width=2)

# Add markers for current and next positions
gmap.marker(current_lat, current_lon, color="red", title="Current Position")
gmap.marker(next_lat, next_lon, color="blue", title="Next Position")

# Save the map as an HTML file
html_file = "direction_angle_visualization.html"
gmap.draw(html_file)

print(f"Map generated: {html_file}")
