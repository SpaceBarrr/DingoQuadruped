// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', function () {
    console.log('Connected to websocket server.');
    document.getElementById("ROS_status").innerText = 'Connected to ROS'
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
    document.getElementById("ROS_status").innerText = 'Cannot connect to ROS. Ensure ROS bridge has been launched.'
});

ros.on('close', function () {
    console.log('Connection to websocket server closed.');
});
