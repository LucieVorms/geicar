# This file describes the main additions for the Across the Universe project on the jetsonNano

* ./src/geicar_start_jetson : The launch file has been changed to add the path_detection (used with the Camera) node and the space_environment (used with the LIDAR) node
* ./src/interface : add a new msg : EnoughSpace.msg used with the LIDAR. It indicates whether a PMR space is found within the 60° range and at what angle.
* ./src/space_environment : This script attempts to find a 1.2m space within a 60° radius in front of the car. If it is found, there is a PMR path. The result is published on the EnoughSpace topic
* ./src/usb_cam : new script (./src/usb_cam/scripts/path_detection.py) added to do some image processing with the AI to :
  *  identify a path with a yoloV8n segmentation model
  *  mesure the distance between the 2 edges of the path 
