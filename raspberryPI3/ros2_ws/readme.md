# This file describes the main additions for the Across the Universe project on the Raspberry

* On this directory all the gnss_xxx.csv files are itineraries that the car follows when selected by the path indicated in raspberryPI3/ros2_ws/src/gps_following/gps_following/gps_control.py with the line `self.itinerary = self.load_itinerary_from_csv('gnss_no_bike.csv')`
* raspberryPI3/ros2_ws/src/car_control : Subscribe to every topic which can affect the behavior of the car and handle the data by publishing orders on CarMotionOrder topic.
 * gps_following : to know where to go
 * obstacle_detection : to know if an emergency stop is required
 * enoughSpace : to know if the path is PRM accessible
   
* raspberryPI3/ros2_ws/src/gps_following : Uses gnss_data and calculates direction (among other things) using a pure pursuit algorithm. 
* raspberryPI3/ros2_ws/src/interfaces : Add a new topics : 
   * CarMotionOrder.msg
   * EnoughSpace.msg
   * GnssStatus.msg
   * MotorsOrders.msg
   * ObstacleInfo.msg
* raspberryPI3/ros2_ws/src/motors_control : Add an upper layer of abstraction to motors command to ease and strengthen the motors control. A control loop is used to regulate reference commands (speed and orientation). This is used by the car_control_node.
  * left|right rear pwm are replaced by carSpeed
  * steering pwm is replaced by front_wheel_orientation 
* raspberryPI3/ros2_ws/src/obstacle_detection : Detects the presence of an obstacle 50cm away. If there is an obstacle, the car stops. If an obstacle appears less than 50cm away, the car moves back up to 50cm.
