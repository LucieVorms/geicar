#!/usr/bin/env python3
from math import degrees, atan2

def calculate_direction_from_imu(imu_data):
    """
    Calculate the direction (bearing) of the vehicle using IMU data.

    :param imu_data: A dictionary containing 'x', 'y', 'z' orientation from the IMU sensor.
    :return: The calculated direction (bearing) in degrees (0° to 360°).
    """
    # Extract x and y components from IMU data
    x = imu_data.get('x', 0.0)
    y = imu_data.get('y', 0.0)

    # Calculate the direction (bearing) using atan2
    bearing = (degrees(atan2(y, x)) + 360) % 360  # Normalize to 0-360
    return bearing
