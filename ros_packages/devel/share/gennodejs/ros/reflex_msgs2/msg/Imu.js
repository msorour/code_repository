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

class Imu {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.quat = null;
      this.euler_angles = null;
      this.calibration_status = null;
      this.calibration_data = null;
    }
    else {
      if (initObj.hasOwnProperty('quat')) {
        this.quat = initObj.quat
      }
      else {
        this.quat = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('euler_angles')) {
        this.euler_angles = initObj.euler_angles
      }
      else {
        this.euler_angles = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('calibration_status')) {
        this.calibration_status = initObj.calibration_status
      }
      else {
        this.calibration_status = 0;
      }
      if (initObj.hasOwnProperty('calibration_data')) {
        this.calibration_data = initObj.calibration_data
      }
      else {
        this.calibration_data = new Array(11).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Imu
    // Check that the constant length array field [quat] has the right length
    if (obj.quat.length !== 4) {
      throw new Error('Unable to serialize array field quat - length must be 4')
    }
    // Serialize message field [quat]
    bufferOffset = _arraySerializer.float32(obj.quat, buffer, bufferOffset, 4);
    // Check that the constant length array field [euler_angles] has the right length
    if (obj.euler_angles.length !== 3) {
      throw new Error('Unable to serialize array field euler_angles - length must be 3')
    }
    // Serialize message field [euler_angles]
    bufferOffset = _arraySerializer.float32(obj.euler_angles, buffer, bufferOffset, 3);
    // Serialize message field [calibration_status]
    bufferOffset = _serializer.uint8(obj.calibration_status, buffer, bufferOffset);
    // Check that the constant length array field [calibration_data] has the right length
    if (obj.calibration_data.length !== 11) {
      throw new Error('Unable to serialize array field calibration_data - length must be 11')
    }
    // Serialize message field [calibration_data]
    bufferOffset = _arraySerializer.uint16(obj.calibration_data, buffer, bufferOffset, 11);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Imu
    let len;
    let data = new Imu(null);
    // Deserialize message field [quat]
    data.quat = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [euler_angles]
    data.euler_angles = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [calibration_status]
    data.calibration_status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [calibration_data]
    data.calibration_data = _arrayDeserializer.uint16(buffer, bufferOffset, 11)
    return data;
  }

  static getMessageSize(object) {
    return 51;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reflex_msgs2/Imu';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dea5c53d0c934f48b1ee6e5a6eed4389';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # quaternion reading from IMU (w, x, y, z)
    float32[4] quat	
    float32[3] euler_angles 
    
    # these are defined in reflex_hand.h driver
    uint8 calibration_status 
    uint16[11] calibration_data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Imu(null);
    if (msg.quat !== undefined) {
      resolved.quat = msg.quat;
    }
    else {
      resolved.quat = new Array(4).fill(0)
    }

    if (msg.euler_angles !== undefined) {
      resolved.euler_angles = msg.euler_angles;
    }
    else {
      resolved.euler_angles = new Array(3).fill(0)
    }

    if (msg.calibration_status !== undefined) {
      resolved.calibration_status = msg.calibration_status;
    }
    else {
      resolved.calibration_status = 0
    }

    if (msg.calibration_data !== undefined) {
      resolved.calibration_data = msg.calibration_data;
    }
    else {
      resolved.calibration_data = new Array(11).fill(0)
    }

    return resolved;
    }
};

module.exports = Imu;
