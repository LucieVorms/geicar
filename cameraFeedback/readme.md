- Launch your ROS2 nodes (on pi@geicar and on the docker of the jetson)
In cameraFeedback folder :
- Launch rosbridge_server (port 9090)
- Open the firewall on port 8080 (default http port) (use ufw allow 8000 && ufw allow 9090 ) : not needed, already done
- Execute python3 -m http.server in the website folder to start a server (on  port 8000)
- From your browser 10.105.1.168/index.html(eduroam)
