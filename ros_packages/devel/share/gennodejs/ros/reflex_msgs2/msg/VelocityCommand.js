// Auto-generated. Do not edit!

// (in-package reflex_msgs2.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class VelocityCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.f1 = null;
      this.f2 = null;
      this.f3 = null;
      this.preshape = null;
    }
    else {
      if (initObj.hasOwnProperty('f1')) {
        this.f1 = initObj.f1
      }
      else {
        this.f1 = 0.0;
      }
      if (initObj.hasOwnProperty('f2')) {
        this.f2 = initObj.f2
      }
      else {
        this.f2 = 0.0;
      }
      if (initObj.hasOwnProperty('f3')) {
        this.f3 = initObj.f3
      }
      else {
        this.f3 = 0.0;
      }
      if (initObj.hasOwnProperty('preshape')) {
        this.preshape = initObj.preshape
      }
      else {
        this.preshape = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VelocityCommand
    // Serialize message field [f1]
    bufferOffset = _serializer.float64(obj.f1, buffer, bufferOffset);
    // Serialize message field [f2]
    bufferOffset = _serializer.float64(obj.f2, buffer, bufferOffset);
    // Serialize message field [f3]
    bufferOffset = _serializer.float64(obj.f3, buffer, bufferOffset);
    // Serialize message field [preshape]
    bufferOffset = _serializer.float64(obj.preshape, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VelocityCommand
    let len;
    let data = new VelocityCommand(null);
    // Deserialize message field [f1]
    data.f1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [f2]
    data.f2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [f3]
    data.f3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [preshape]
    data.preshape = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reflex_msgs2/VelocityCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec8e01f7c46594906539a78e3918a7c2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Velocity in radians/second of various motors
    float64 f1
    float64 f2
    float64 f3
    float64 preshape
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VelocityCommand(null);
    if (msg.f1 !== undefined) {
      resolved.f1 = msg.f1;
    }
    else {
      resolved.f1 = 0.0
    }

    if (msg.f2 !== undefined) {
      resolved.f2 = msg.f2;
    }
    else {
      resolved.f2 = 0.0
    }

    if (msg.f3 !== undefined) {
      resolved.f3 = msg.f3;
    }
    else {
      resolved.f3 = 0.0
    }

    if (msg.preshape !== undefined) {
      resolved.preshape = msg.preshape;
    }
    else {
      resolved.preshape = 0.0
    }

    return resolved;
    }
};

module.exports = VelocityCommand;
