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
    document.getElementById("joint_states_fl_roll").innerText = msg.fl.roll.toFixed(2)
    document.getElementById("joint_states_fl_upper").innerText = msg.fl.upper.toFixed(2)
    document.getElementById("joint_states_fl_lower").innerText = msg.fl.lower.toFixed(2)

    document.getElementById("joint_states_fr_roll").innerText = msg.fr.roll.toFixed(2)
    document.getElementById("joint_states_fr_upper").innerText = msg.fr.upper.toFixed(2)
    document.getElementById("joint_states_fr_lower").innerText = msg.fr.lower.toFixed(2)

    document.getElementById("joint_states_rl_roll").innerText = msg.rl.roll.toFixed(2)
    document.getElementById("joint_states_rl_upper").innerText = msg.rl.upper.toFixed(2)
    document.getElementById("joint_states_rl_lower").innerText = msg.rl.lower.toFixed(2)

    document.getElementById("joint_states_rr_roll").innerText = msg.rr.roll.toFixed(2)
    document.getElementById("joint_states_rr_upper").innerText = msg.rr.upper.toFixed(2)
    document.getElementById("joint_states_rr_lower").innerText = msg.rr.lower.toFixed(2)
    // console.log(msg.fr.roll + msg.fr.upper + msg.fr.lower)
});

currents_topic.subscribe(function (msg) {
    document.getElementById("currents_fl_roll").innerText = msg.fl.roll.toFixed(2)
    document.getElementById("currents_fl_upper").innerText = msg.fl.upper.toFixed(2)
    document.getElementById("currents_fl_lower").innerText = msg.fl.lower.toFixed(2)

    document.getElementById("currents_fr_roll").innerText = msg.fr.roll.toFixed(2)
    document.getElementById("currents_fr_upper").innerText = msg.fr.upper.toFixed(2)
    document.getElementById("currents_fr_lower").innerText = msg.fr.lower.toFixed(2)

    document.getElementById("currents_rl_roll").innerText = msg.rl.roll.toFixed(2)
    document.getElementById("currents_rl_upper").innerText = msg.rl.upper.toFixed(2)
    document.getElementById("currents_rl_lower").innerText = msg.rl.lower.toFixed(2)

    document.getElementById("currents_rr_roll").innerText = msg.rr.roll.toFixed(2)
    document.getElementById("currents_rr_upper").innerText = msg.rr.upper.toFixed(2)
    document.getElementById("currents_rr_lower").innerText = msg.rr.lower.toFixed(2)
    // console.log(msg.fr.roll + msg.fr.upper + msg.fr.lower)
});

batt_voltage_topic.subscribe(function (msg) {
    document.getElementById("batt_voltage_v1").innerText = msg.x.toFixed(2)
    document.getElementById("batt_voltage_v2").innerText = msg.y.toFixed(2)
    document.getElementById("batt_voltage_overall").innerText = msg.z.toFixed(2)
    // console.log(msg.x + msg.y + msg.z)
});

imu_topic.subscribe(function (msg) {
    document.getElementById("imu_roll").innerText = msg.att.roll.toFixed(2)
    document.getElementById("imu_pitch").innerText = msg.att.pitch.toFixed(2)
    document.getElementById("imu_yaw").innerText = msg.att.yaw.toFixed(2)
    document.getElementById("imu_acc_x").innerText = msg.acc.x.toFixed(2)
    document.getElementById("imu_acc_y").innerText = msg.acc.y.toFixed(2)
    document.getElementById("imu_acc_z").innerText = msg.acc.z.toFixed(2)
    document.getElementById("imu_gyro_x").innerText = msg.gyro.x.toFixed(2)
    document.getElementById("imu_gyro_y").innerText = msg.gyro.y.toFixed(2)
    document.getElementById("imu_gyro_z").innerText = msg.gyro.z.toFixed(2)
    // console.log(msg.acc.x + msg.acc.y + msg.acc.z)
});

// Publishing to Topic for Stance Control
// ------------------
var command_input = new ROSLIB.Topic({
    ros : ros,
    name : '/kelpie/command_input',
    messageType : 'kelpie/commands'
});

var command = new ROSLIB.Message({
  gait_toggle : false,
  hop_toggle : false,
  calibration_toggle : false,
  joystick_toggle : false,
  pid_toggle : false,
  x : 0,
  y : 0,
  roll_movement : 0,
  pitch : 0,
  yaw_rate : 0,
  height_movement : 0
});

function mouse_down(button) {
  document.getElementById("status").innerText = button + " button clicked";
  switch(button) {
    case "height up":
      command.height_movement = 1
      break;
    case "height down":
      command.height_movement = -1
      break;
    case "pitch up":
      command.pitch = 0.5
      break;
    case "pitch down":
      command.pitch = -0.5
      break;
    case "roll left":
      command.roll_movement = 1
      break;
    case "roll right":
      command.roll_movement = -1
      break;
    case "yaw left":
      command.yaw_rate = 0.5
      break;
    case "yaw right":
      command.yaw_rate = -0.5
      break;
    default:
      break;
  }
}

function mouse_up(button) {
  document.getElementById("status").innerText = button + " button released";
  switch(button) {
    case "height up":
      command.height_movement = 0.0
      break;
    case "height down":
      command.height_movement = 0.0
      break;
    case "pitch up":
      command.pitch = 0.0
      break;
    case "pitch down":
      command.pitch = 0.0
      break;
    case "roll left":
      command.roll_movement = 0.0
      break;
    case "roll right":
      command.roll_movement = 0.0
      break;
    case "yaw left":
      command.yaw_rate = 0.0
      break;
    case "yaw right":
      command.yaw_rate = 0.0
      break;
    default:
      break;
  }
}

command_input.publish(command);