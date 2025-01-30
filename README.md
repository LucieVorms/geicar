# GeiCar Project
The GeiCar project is a project carried out by students at [INSA Toulouse](http://www.insa-toulouse.fr/fr/index.html). This project consists in developing the software of a autonomous car in order to carry out different missions. Several projects are exposed on the [official website] (https://sites.google.com/site/projetsecinsa/).

This repository is intended to provide a basis for students starting a new project on the GeiCar. The present code as well as the documentation is the result of internship carried out by [Alexis24](https://github.com/Alexix24) (Alexis Pierre Dit Lambert)

The platform is developped and maintained by :

* DI MERCURIO Sébastien
* LOMBARD Emmanuel
* MARTIN José

The projects are (or were) surpervised by:

* CHANTHERY Elodie
* DELAUTIER Sébastien
* MONTEIL Thierry
* LE BOTLAN Didier
* AURIOL Guillaume
* DI MERCURIO Sébastien

## Quick User Guide
### Turn the car on and off
* To turn on the car:
  * Toggle the red button to bring the power.
  * Press the START push button (hold it down for a short while).
  * Wait until Raspberry boot up and connect to it using its IP address (written on the board): `ssh pi@10.105.1.xx`
  * When connected start ROS subsystem using :`ros2 launch geicar_start geicar.launch.py`
  * Then, you will get a report of subsystems and be able to control the car using XBOX controler 

* To turn off the car:
	* Use the red button as a switch to turn off the power.

### Use of this repository
* First of all, [fork this repository](https://docs.github.com/en/get-started/quickstart/fork-a-repo) to get yours.
* Then, clone your fresh and new repository on your computer: `git clone https://github.com/<your id>/<your wonderful repo>`
* Have a look in "general" directory for how to connect and work with your car
* For more 'in depth' documentation on a particular subsystem, have a look in following directories:
    * raspberryPI3: Everything about raspberry setup
    * nucleoL476: Stuff about GPS-RTK and IMU
    * nucleoF103: informations on motors control, main power and ultrasonics sensors
    * jetsonNano: Directory containing info on IA, Camera and Lidar
    * simulation: if you want to setup a carla simulation environment

__warning__
You normally do not need to change firmware running in F103 and F476 boards. You main work is on the raspberry and jetson side.

# Across the Univers project based on GeiCar 
This project was carry out by :
* AVY Antoine
* VORMS Lucie
* SANCHEZ Manon
* LE Duc-Anh
* SAINT-LOUBERT Jean-Yves
* GNANGUESSIM Diskouna John

And this project was surpervised by:
* LE BOTLAN Didier
* LELEUX Philippe

## Quick User Guide
### Turn the car on autonomous mode and off autonomous mode
* To turn on the autonomous mode :
	* Turn on the car and launch the node on the Raspberry and the Jetson (see ./general/how_to_use.rd)
	* Press start and A on the XBOX controller
 	* To stop the car during autonomous mode : press B on the XBOX controller  
 * To turn off the autonomous mode :
 	* Press Y on the XBOX controller

### Use of this repository
* Changes and additions from the GeiCar repository :
    * raspberryPI3: Autonomous path following (for more 'in depth' documentation : see documentation in this directory)
    * nucleoL476: Nothing
    * nucleoF103: Nothing
    * jetsonNano: AI and camera for measuring the width of the path and LIDAR for PRM path detection and obstacle avoidance (for more 'in depth' documentation : see documentation in this directory)
    * simulation: Nothing
    * path_detection : Contains the AI models used on the jetson
    * IA/yolo_segmentation : Contains test script and model training (it was for local test, the scripts are re-used in ./jetsonNano/ros2_ws/src/usb_cam/scripts)
    * cameraFeedback : Local site hosting camera rendering with real-time distance measurement and video demonstrations of features
    * openCV_test : Contains test scripts with openCV for path detection and measurement without AI : dead end (not used)

    
