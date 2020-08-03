// Auto-generated. Do not edit!

// (in-package reflex_msgs2.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Finger = require('./Finger.js');
let Motor = require('./Motor.js');
let Imu = require('./Imu.js');

//-----------------------------------------------------------

class Hand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.finger = null;
      this.motor = null;
      this.palmImu = null;
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
        this.motor = new Array(4).fill(new Motor());
      }
      if (initObj.hasOwnProperty('palmImu')) {
        this.palmImu = initObj.palmImu
      }
      else {
        this.palmImu = new Imu();
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
    if (obj.motor.length !== 4) {
      throw new Error('Unable to serialize array field motor - length must be 4')
    }
    // Serialize message field [motor]
    obj.motor.forEach((val) => {
      bufferOffset = Motor.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [palmImu]
    bufferOffset = Imu.serialize(obj.palmImu, buffer, bufferOffset);
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
    len = 4;
    data.motor = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.motor[i] = Motor.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [palmImu]
    data.palmImu = Imu.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.motor.forEach((val) => {
      length += Motor.getMessageSize(val);
    });
    return length + 438;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reflex_msgs2/Hand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd9d385c8f6333f652b7d358710209f43';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # message for ReFlex Hand
    Finger[3] finger       # Hold out your right hand palm up, with pointer finger, middle finger and thumb extended
                           # Pointer = finger[0], Middle = finger[1], Thumb = finger[2]
    Motor[4] motor         # Finger 1, Finger 2, Finger 3, and Preshape
    Imu palmImu
    #ImuCalibrationData LANCE
    
    #CHANGE IMU TO ARRAY!!!
    ================================================================================
    MSG: reflex_msgs2/Finger
    # message for ReFlex Fingers
    float32 proximal		# radians, measured from all open = 0, to pi = closed
    float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link
    
    bool[14] contact			# binary, 0 = proximal, 8 = fingertip
    float32[14] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)
    Imu imu
    ================================================================================
    MSG: reflex_msgs2/Imu
    # quaternion reading from IMU (w, x, y, z)
    float32[4] quat	
    float32[3] euler_angles 
    
    # these are defined in reflex_hand.h driver
    uint8 calibration_status 
    uint16[11] calibration_data
    
    ================================================================================
    MSG: reflex_msgs2/Motor
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
      resolved.motor = new Array(4)
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
      resolved.motor = new Array(4).fill(new Motor())
    }

    if (msg.palmImu !== undefined) {
      resolved.palmImu = Imu.Resolve(msg.palmImu)
    }
    else {
      resolved.palmImu = new Imu()
    }

    return resolved;
    }
};

module.exports = Hand;
