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