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
    case 'height up':
      document.getElementById("status").innerText = button + " test";
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