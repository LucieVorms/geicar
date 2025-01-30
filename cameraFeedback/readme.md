To have acces to the website you have to be connected to the same network as the car and in ros.js you can change the IP adress of the car depending of the network.

- Launch your ROS2 nodes (on pi@geicar and on the docker of the jetson)  
In cameraFeedback folder :
- Launch rosbridge_server (port 9090) : `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Open the firewall on port 8080 (default http port) (use `ufw allow 8000` && `ufw allow 9090`). One time only: check with `sudo ufw status`
- Execute `python3 -m http.server` in the website folder to start a server (on port 8000)
- From your browser 10.105.1.167/index.html(eduroam) or 172.20.10.5/index.html (Antoine's Iphone) to access to the website
