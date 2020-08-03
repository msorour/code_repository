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

class Finger {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.proximal = null;
      this.distal_approx = null;
      this.contact = null;
      this.pressure = null;
    }
    else {
      if (initObj.hasOwnProperty('proximal')) {
        this.proximal = initObj.proximal
      }
      else {
        this.proximal = 0.0;
      }
      if (initObj.hasOwnProperty('distal_approx')) {
        this.distal_approx = initObj.distal_approx
      }
      else {
        this.distal_approx = 0.0;
      }
      if (initObj.hasOwnProperty('contact')) {
        this.contact = initObj.contact
      }
      else {
        this.contact = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('pressure')) {
        this.pressure = initObj.pressure
      }
      else {
        this.pressure = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Finger
    // Serialize message field [proximal]
    bufferOffset = _serializer.float32(obj.proximal, buffer, bufferOffset);
    // Serialize message field [distal_approx]
    bufferOffset = _serializer.float32(obj.distal_approx, buffer, bufferOffset);
    // Check that the constant length array field [contact] has the right length
    if (obj.contact.length !== 9) {
      throw new Error('Unable to serialize array field contact - length must be 9')
    }
    // Serialize message field [contact]
    bufferOffset = _arraySerializer.bool(obj.contact, buffer, bufferOffset, 9);
    // Check that the constant length array field [pressure] has the right length
    if (obj.pressure.length !== 9) {
      throw new Error('Unable to serialize array field pressure - length must be 9')
    }
    // Serialize message field [pressure]
    bufferOffset = _arraySerializer.float32(obj.pressure, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Finger
    let len;
    let data = new Finger(null);
    // Deserialize message field [proximal]
    data.proximal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distal_approx]
    data.distal_approx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [contact]
    data.contact = _arrayDeserializer.bool(buffer, bufferOffset, 9)
    // Deserialize message field [pressure]
    data.pressure = _arrayDeserializer.float32(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    return 53;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reflex_msgs/Finger';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b5232f74e901b48063f64cfc32aefbe0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # message for ReFlex Fingers
    float32 proximal		# radians, measured from all open = 0, to pi = closed
    float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link
    bool[9] contact			# binary, 0 = proximal, 8 = fingertip
    float32[9] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Finger(null);
    if (msg.proximal !== undefined) {
      resolved.proximal = msg.proximal;
    }
    else {
      resolved.proximal = 0.0
    }

    if (msg.distal_approx !== undefined) {
      resolved.distal_approx = msg.distal_approx;
    }
    else {
      resolved.distal_approx = 0.0
    }

    if (msg.contact !== undefined) {
      resolved.contact = msg.contact;
    }
    else {
      resolved.contact = new Array(9).fill(0)
    }

    if (msg.pressure !== undefined) {
      resolved.pressure = msg.pressure;
    }
    else {
      resolved.pressure = new Array(9).fill(0)
    }

    return resolved;
    }
};

module.exports = Finger;
