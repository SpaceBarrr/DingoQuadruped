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
var joint_states_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/kelpie/leg_control/joint_states',
    messageType: 'kelpie/joint_states'
});

var currents_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/kelpie/leg_control/currents',
    messageType: 'kelpie/joint_states'
});

var batt_voltage_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/kelpie/battery/voltage',
    messageType: 'kelpie/xyz_float32'
});

var imu_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/kelpie/imu',
    messageType: 'kelpie/imu'
});

// Other publishers:
//     name: '/kelpie/emergency_stop_status'
//     messageType: bool

//     name: '/kelpie/command_input'
//     messageType: 'kelpie/commands'

// Subscribing to Topics
// ----------------------
joint_states_topic.subscribe(function (msg) {
    document.getElementById("joint_states_fr_roll").innerText = msg.fr.roll
    document.getElementById("joint_states_fr_upper").innerText = msg.fr.upper
    document.getElementById("joint_states_fr_lower").innerText = msg.fr.lower
    console.log(msg.fr.roll + msg.fr.upper + msg.fr.lower)
});

currents_topic.subscribe(function (msg) {
    document.getElementById("currents_fr_roll").innerText = msg.fr.roll
    document.getElementById("currents_fr_upper").innerText = msg.fr.upper
    document.getElementById("currents_fr_lower").innerText = msg.fr.lower
    console.log(msg.fr.roll + msg.fr.upper + msg.fr.lower)
});

batt_voltage_topic.subscribe(function (msg) {
    document.getElementById("batt_voltage_v1").innerText = msg.x
    document.getElementById("batt_voltage_v2").innerText = msg.y
    document.getElementById("batt_voltage_overall").innerText = msg.z
    console.log(msg.x + msg.y + msg.z)
});

imu_topic.subscribe(function (msg) {
    document.getElementById("imu_acc_x").innerText = msg.acc.x
    document.getElementById("imu_acc_y").innerText = msg.acc.y
    document.getElementById("imu_acc_z").innerText = msg.acc.z
    console.log(msg.acc.x + msg.acc.y + msg.acc.z)
});