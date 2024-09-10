// Auto-generated. Do not edit!

// (in-package kelpie.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let att = require('./att.js');
let xyz_float32 = require('./xyz_float32.js');

//-----------------------------------------------------------

class imu {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.att = null;
      this.acc = null;
      this.gyro = null;
      this.gbias = null;
      this.grav = null;
    }
    else {
      if (initObj.hasOwnProperty('att')) {
        this.att = initObj.att
      }
      else {
        this.att = new att();
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = new xyz_float32();
      }
      if (initObj.hasOwnProperty('gyro')) {
        this.gyro = initObj.gyro
      }
      else {
        this.gyro = new xyz_float32();
      }
      if (initObj.hasOwnProperty('gbias')) {
        this.gbias = initObj.gbias
      }
      else {
        this.gbias = new xyz_float32();
      }
      if (initObj.hasOwnProperty('grav')) {
        this.grav = initObj.grav
      }
      else {
        this.grav = new xyz_float32();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type imu
    // Serialize message field [att]
    bufferOffset = att.serialize(obj.att, buffer, bufferOffset);
    // Serialize message field [acc]
    bufferOffset = xyz_float32.serialize(obj.acc, buffer, bufferOffset);
    // Serialize message field [gyro]
    bufferOffset = xyz_float32.serialize(obj.gyro, buffer, bufferOffset);
    // Serialize message field [gbias]
    bufferOffset = xyz_float32.serialize(obj.gbias, buffer, bufferOffset);
    // Serialize message field [grav]
    bufferOffset = xyz_float32.serialize(obj.grav, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type imu
    let len;
    let data = new imu(null);
    // Deserialize message field [att]
    data.att = att.deserialize(buffer, bufferOffset);
    // Deserialize message field [acc]
    data.acc = xyz_float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [gyro]
    data.gyro = xyz_float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [gbias]
    data.gbias = xyz_float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [grav]
    data.grav = xyz_float32.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kelpie/imu';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ce43921e7b399d3ef9d38833a187cc81';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    att att
    xyz_float32 acc
    xyz_float32 gyro
    xyz_float32 gbias
    xyz_float32 grav
    
    ================================================================================
    MSG: kelpie/att
    float32 roll
    float32 pitch
    float32 yaw
    
    ================================================================================
    MSG: kelpie/xyz_float32
    float32 x
    float32 y
    float32 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new imu(null);
    if (msg.att !== undefined) {
      resolved.att = att.Resolve(msg.att)
    }
    else {
      resolved.att = new att()
    }

    if (msg.acc !== undefined) {
      resolved.acc = xyz_float32.Resolve(msg.acc)
    }
    else {
      resolved.acc = new xyz_float32()
    }

    if (msg.gyro !== undefined) {
      resolved.gyro = xyz_float32.Resolve(msg.gyro)
    }
    else {
      resolved.gyro = new xyz_float32()
    }

    if (msg.gbias !== undefined) {
      resolved.gbias = xyz_float32.Resolve(msg.gbias)
    }
    else {
      resolved.gbias = new xyz_float32()
    }

    if (msg.grav !== undefined) {
      resolved.grav = xyz_float32.Resolve(msg.grav)
    }
    else {
      resolved.grav = new xyz_float32()
    }

    return resolved;
    }
};

module.exports = imu;
