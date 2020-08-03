// Auto-generated. Do not edit!

// (in-package reflex_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Motor {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_angle = null;
      this.raw_angle = null;
      this.velocity = null;
      this.load = null;
      this.voltage = null;
      this.temperature = null;
      this.error_state = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_angle')) {
        this.joint_angle = initObj.joint_angle
      }
      else {
        this.joint_angle = 0.0;
      }
      if (initObj.hasOwnProperty('raw_angle')) {
        this.raw_angle = initObj.raw_angle
      }
      else {
        this.raw_angle = 0.0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
      if (initObj.hasOwnProperty('load')) {
        this.load = initObj.load
      }
      else {
        this.load = 0.0;
      }
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0.0;
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = 0;
      }
      if (initObj.hasOwnProperty('error_state')) {
        this.error_state = initObj.error_state
      }
      else {
        this.error_state = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Motor
    // Serialize message field [joint_angle]
    bufferOffset = _serializer.float64(obj.joint_angle, buffer, bufferOffset);
    // Serialize message field [raw_angle]
    bufferOffset = _serializer.float64(obj.raw_angle, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float64(obj.velocity, buffer, bufferOffset);
    // Serialize message field [load]
    bufferOffset = _serializer.float64(obj.load, buffer, bufferOffset);
    // Serialize message field [voltage]
    bufferOffset = _serializer.float64(obj.voltage, buffer, bufferOffset);
    // Serialize message field [temperature]
    bufferOffset = _serializer.int32(obj.temperature, buffer, bufferOffset);
    // Serialize message field [error_state]
    bufferOffset = _serializer.string(obj.error_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Motor
    let len;
    let data = new Motor(null);
    // Deserialize message field [joint_angle]
    data.joint_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [raw_angle]
    data.raw_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [load]
    data.load = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [voltage]
    data.voltage = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [temperature]
    data.temperature = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [error_state]
    data.error_state = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.error_state.length;
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reflex_msgs/Motor';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '66d6779b4fae4b7b68e0863263c3993c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 joint_angle
    float64 raw_angle
    float64 velocity
    float64 load
    float64 voltage
    int32 temperature
    string error_state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Motor(null);
    if (msg.joint_angle !== undefined) {
      resolved.joint_angle = msg.joint_angle;
    }
    else {
      resolved.joint_angle = 0.0
    }

    if (msg.raw_angle !== undefined) {
      resolved.raw_angle = msg.raw_angle;
    }
    else {
      resolved.raw_angle = 0.0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    if (msg.load !== undefined) {
      resolved.load = msg.load;
    }
    else {
      resolved.load = 0.0
    }

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0.0
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = 0
    }

    if (msg.error_state !== undefined) {
      resolved.error_state = msg.error_state;
    }
    else {
      resolved.error_state = ''
    }

    return resolved;
    }
};

module.exports = Motor;
