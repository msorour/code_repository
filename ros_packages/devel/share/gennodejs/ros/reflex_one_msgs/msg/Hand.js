// Auto-generated. Do not edit!

// (in-package reflex_one_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Finger = require('./Finger.js');
let Motor = require('./Motor.js');

//-----------------------------------------------------------

class Hand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.finger = null;
      this.motor = null;
    }
    else {
      if (initObj.hasOwnProperty('finger')) {
        this.finger = initObj.finger
      }
      else {
        this.finger = new Array(3).fill(new Finger());
      }
      if (initObj.hasOwnProperty('motor')) {
        this.motor = initObj.motor
      }
      else {
        this.motor = new Array(5).fill(new Motor());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Hand
    // Check that the constant length array field [finger] has the right length
    if (obj.finger.length !== 3) {
      throw new Error('Unable to serialize array field finger - length must be 3')
    }
    // Serialize message field [finger]
    obj.finger.forEach((val) => {
      bufferOffset = Finger.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [motor] has the right length
    if (obj.motor.length !== 5) {
      throw new Error('Unable to serialize array field motor - length must be 5')
    }
    // Serialize message field [motor]
    obj.motor.forEach((val) => {
      bufferOffset = Motor.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Hand
    let len;
    let data = new Hand(null);
    // Deserialize message field [finger]
    len = 3;
    data.finger = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.finger[i] = Finger.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [motor]
    len = 5;
    data.motor = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.motor[i] = Motor.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.motor.forEach((val) => {
      length += Motor.getMessageSize(val);
    });
    return length + 159;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reflex_one_msgs/Hand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5328a4ca4ff4a136db2555c3178a3e1b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # message for ReFlex Hand
    Finger[3] finger       # Hold out your right hand palm up, with pointer finger, middle finger and thumb extended
                           # Pointer = finger[0], Middle = finger[1], Thumb = finger[2]
    Motor[5] motor         # Finger 1, Finger 2, Finger 3, Preshape1 and Preshape2
    
    ================================================================================
    MSG: reflex_one_msgs/Finger
    # message for ReFlex Fingers
    float32 proximal		# radians, measured from all open = 0, to pi = closed
    float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link
    bool[9] contact			# binary, 0 = proximal, 8 = fingertip
    float32[9] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)
    
    ================================================================================
    MSG: reflex_one_msgs/Motor
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
    const resolved = new Hand(null);
    if (msg.finger !== undefined) {
      resolved.finger = new Array(3)
      for (let i = 0; i < resolved.finger.length; ++i) {
        if (msg.finger.length > i) {
          resolved.finger[i] = Finger.Resolve(msg.finger[i]);
        }
        else {
          resolved.finger[i] = new Finger();
        }
      }
    }
    else {
      resolved.finger = new Array(3).fill(new Finger())
    }

    if (msg.motor !== undefined) {
      resolved.motor = new Array(5)
      for (let i = 0; i < resolved.motor.length; ++i) {
        if (msg.motor.length > i) {
          resolved.motor[i] = Motor.Resolve(msg.motor[i]);
        }
        else {
          resolved.motor[i] = new Motor();
        }
      }
    }
    else {
      resolved.motor = new Array(5).fill(new Motor())
    }

    return resolved;
    }
};

module.exports = Hand;
