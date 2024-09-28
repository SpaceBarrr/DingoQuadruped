// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', function () {
    console.log('Connected to websocket server.');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed.');
});

// Declaring ROS Topics
// ----------------------
var camera_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/kelpie/image/compressed',
    messageType: 'sensor_msgs/CompressedImage'
});

// Subscribing to Topics
// ----------------------
camera_topic.subscribe(function (msg) {
    document.getElementById("camera_img").src = "data:image/jpg;base64," + msg.data;
});