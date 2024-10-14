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
    messageType: 'kelpie/vec_3d_float32'
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
    let total_current = 0;

    document.getElementById("currents_fl_roll").innerText = msg.fl.roll.toFixed(2)
    total_current += msg.fl.roll;
    document.getElementById("currents_fl_upper").innerText = msg.fl.upper.toFixed(2)
    total_current += msg.fl.upper;
    document.getElementById("currents_fl_lower").innerText = msg.fl.lower.toFixed(2)
    total_current += msg.fl.lower;

    document.getElementById("currents_fr_roll").innerText = msg.fr.roll.toFixed(2)
    total_current += msg.fr.roll;
    document.getElementById("currents_fr_upper").innerText = msg.fr.upper.toFixed(2)
    total_current += msg.fr.upper;
    document.getElementById("currents_fr_lower").innerText = msg.fr.lower.toFixed(2)
    total_current += msg.fr.lower;

    document.getElementById("currents_rl_roll").innerText = msg.rl.roll.toFixed(2)
    total_current += msg.rl.roll;
    document.getElementById("currents_rl_upper").innerText = msg.rl.upper.toFixed(2)
    total_current += msg.rl.upper;
    document.getElementById("currents_rl_lower").innerText = msg.rl.lower.toFixed(2)
    total_current += msg.rl.lower;

    document.getElementById("currents_rr_roll").innerText = msg.rr.roll.toFixed(2)
    total_current += msg.rr.roll;
    document.getElementById("currents_rr_upper").innerText = msg.rr.upper.toFixed(2)
    total_current += msg.rr.upper;
    document.getElementById("currents_rr_lower").innerText = msg.rr.lower.toFixed(2)
    total_current += msg.rr.lower;

    document.getElementById("total_current").innerText = total_current.toFixed(2)
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