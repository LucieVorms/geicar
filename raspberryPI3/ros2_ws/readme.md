# This file describes the main additions for the Across the Universe project on the Raspberry

* On this directory all the gnss_xxx.csv files are itineraries that the car follows when selected by the path indicated in raspberryPI3/ros2_ws/src/gps_following/gps_following/gps_control.py with the line `self.itinerary = self.load_itinerary_from_csv('gnss_no_bike.csv')`
* raspberryPI3/ros2_ws/src/car_control :
* raspberryPI3/ros2_ws/src/gps_following : Use gnss_data
* raspberryPI3/ros2_ws/src/interfaces :
* raspberryPI3/ros2_ws/src/motors_control : Add an upper layer of abstraction to motors command to ease and strengthen the motors control. A control loop is used to regulate reference commands (speed and orientation). This is used by the car_control_node.
  * left|right rear pwm are replaced by carSpeed
  * steering pwm is replaced by front_wheel_orientation 
* raspberryPI3/ros2_ws/src/obstacle_detection : 
