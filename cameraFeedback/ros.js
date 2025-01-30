const ROSBRIDGE_IP = "172.20.10.10"; 
const ROSBRIDGE_PORT = 9090;

let ros = new ROSLIB.Ros({
    url: `ws://${ROSBRIDGE_IP}:${ROSBRIDGE_PORT}`,
});

ros.on("connection", () => console.log("Connecté au serveur rosbridge"));
ros.on("error", (error) => console.error("Erreur de connexion :", error));
ros.on("close", () => console.log("Connexion fermée"));

const videoTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/path_detection/compressed", 
    messageType: "sensor_msgs/msg/CompressedImage", 
});

videoTopic.subscribe((message) => {
    const imageData = `data:image/jpeg;base64,${message.data}`;
    document.getElementById("cameraFeed").src = imageData;
});
