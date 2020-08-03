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

class ImuCalibrationData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.acc_offset_f1 = null;
      this.acc_offset_f2 = null;
      this.acc_offset_f3 = null;
      this.acc_offset_palm = null;
      this.mag_offset_f1 = null;
      this.mag_offset_f2 = null;
      this.mag_offset_f3 = null;
      this.mag_offset_palm = null;
      this.gyr_offset_f1 = null;
      this.gyr_offset_f2 = null;
      this.gyr_offset_f3 = null;
      this.gyr_offset_palm = null;
      this.acc_radius = null;
      this.gyr_radius = null;
    }
    else {
      if (initObj.hasOwnProperty('acc_offset_f1')) {
        this.acc_offset_f1 = initObj.acc_offset_f1
      }
      else {
        this.acc_offset_f1 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('acc_offset_f2')) {
        this.acc_offset_f2 = initObj.acc_offset_f2
      }
      else {
        this.acc_offset_f2 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('acc_offset_f3')) {
        this.acc_offset_f3 = initObj.acc_offset_f3
      }
      else {
        this.acc_offset_f3 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('acc_offset_palm')) {
        this.acc_offset_palm = initObj.acc_offset_palm
      }
      else {
        this.acc_offset_palm = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('mag_offset_f1')) {
        this.mag_offset_f1 = initObj.mag_offset_f1
      }
      else {
        this.mag_offset_f1 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('mag_offset_f2')) {
        this.mag_offset_f2 = initObj.mag_offset_f2
      }
      else {
        this.mag_offset_f2 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('mag_offset_f3')) {
        this.mag_offset_f3 = initObj.mag_offset_f3
      }
      else {
        this.mag_offset_f3 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('mag_offset_palm')) {
        this.mag_offset_palm = initObj.mag_offset_palm
      }
      else {
        this.mag_offset_palm = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('gyr_offset_f1')) {
        this.gyr_offset_f1 = initObj.gyr_offset_f1
      }
      else {
        this.gyr_offset_f1 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('gyr_offset_f2')) {
        this.gyr_offset_f2 = initObj.gyr_offset_f2
      }
      else {
        this.gyr_offset_f2 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('gyr_offset_f3')) {
        this.gyr_offset_f3 = initObj.gyr_offset_f3
      }
      else {
        this.gyr_offset_f3 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('gyr_offset_palm')) {
        this.gyr_offset_palm = initObj.gyr_offset_palm
      }
      else {
        this.gyr_offset_palm = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('acc_radius')) {
        this.acc_radius = initObj.acc_radius
      }
      else {
        this.acc_radius = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('gyr_radius')) {
        this.gyr_radius = initObj.gyr_radius
      }
      else {
        this.gyr_radius = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImuCalibrationData
    // Check that the constant length array field [acc_offset_f1] has the right length
    if (obj.acc_offset_f1.length !== 3) {
      throw new Error('Unable to serialize array field acc_offset_f1 - length must be 3')
    }
    // Serialize message field [acc_offset_f1]
    bufferOffset = _arraySerializer.uint16(obj.acc_offset_f1, buffer, bufferOffset, 3);
    // Check that the constant length array field [acc_offset_f2] has the right length
    if (obj.acc_offset_f2.length !== 3) {
      throw new Error('Unable to serialize array field acc_offset_f2 - length must be 3')
    }
    // Serialize message field [acc_offset_f2]
    bufferOffset = _arraySerializer.uint16(obj.acc_offset_f2, buffer, bufferOffset, 3);
    // Check that the constant length array field [acc_offset_f3] has the right length
    if (obj.acc_offset_f3.length !== 3) {
      throw new Error('Unable to serialize array field acc_offset_f3 - length must be 3')
    }
    // Serialize message field [acc_offset_f3]
    bufferOffset = _arraySerializer.uint16(obj.acc_offset_f3, buffer, bufferOffset, 3);
    // Check that the constant length array field [acc_offset_palm] has the right length
    if (obj.acc_offset_palm.length !== 3) {
      throw new Error('Unable to serialize array field acc_offset_palm - length must be 3')
    }
    // Serialize message field [acc_offset_palm]
    bufferOffset = _arraySerializer.uint16(obj.acc_offset_palm, buffer, bufferOffset, 3);
    // Check that the constant length array field [mag_offset_f1] has the right length
    if (obj.mag_offset_f1.length !== 3) {
      throw new Error('Unable to serialize array field mag_offset_f1 - length must be 3')
    }
    // Serialize message field [mag_offset_f1]
    bufferOffset = _arraySerializer.uint16(obj.mag_offset_f1, buffer, bufferOffset, 3);
    // Check that the constant length array field [mag_offset_f2] has the right length
    if (obj.mag_offset_f2.length !== 3) {
      throw new Error('Unable to serialize array field mag_offset_f2 - length must be 3')
    }
    // Serialize message field [mag_offset_f2]
    bufferOffset = _arraySerializer.uint16(obj.mag_offset_f2, buffer, bufferOffset, 3);
    // Check that the constant length array field [mag_offset_f3] has the right length
    if (obj.mag_offset_f3.length !== 3) {
      throw new Error('Unable to serialize array field mag_offset_f3 - length must be 3')
    }
    // Serialize message field [mag_offset_f3]
    bufferOffset = _arraySerializer.uint16(obj.mag_offset_f3, buffer, bufferOffset, 3);
    // Check that the constant length array field [mag_offset_palm] has the right length
    if (obj.mag_offset_palm.length !== 3) {
      throw new Error('Unable to serialize array field mag_offset_palm - length must be 3')
    }
    // Serialize message field [mag_offset_palm]
    bufferOffset = _arraySerializer.uint16(obj.mag_offset_palm, buffer, bufferOffset, 3);
    // Check that the constant length array field [gyr_offset_f1] has the right length
    if (obj.gyr_offset_f1.length !== 3) {
      throw new Error('Unable to serialize array field gyr_offset_f1 - length must be 3')
    }
    // Serialize message field [gyr_offset_f1]
    bufferOffset = _arraySerializer.uint16(obj.gyr_offset_f1, buffer, bufferOffset, 3);
    // Check that the constant length array field [gyr_offset_f2] has the right length
    if (obj.gyr_offset_f2.length !== 3) {
      throw new Error('Unable to serialize array field gyr_offset_f2 - length must be 3')
    }
    // Serialize message field [gyr_offset_f2]
    bufferOffset = _arraySerializer.uint16(obj.gyr_offset_f2, buffer, bufferOffset, 3);
    // Check that the constant length array field [gyr_offset_f3] has the right length
    if (obj.gyr_offset_f3.length !== 3) {
      throw new Error('Unable to serialize array field gyr_offset_f3 - length must be 3')
    }
    // Serialize message field [gyr_offset_f3]
    bufferOffset = _arraySerializer.uint16(obj.gyr_offset_f3, buffer, bufferOffset, 3);
    // Check that the constant length array field [gyr_offset_palm] has the right length
    if (obj.gyr_offset_palm.length !== 3) {
      throw new Error('Unable to serialize array field gyr_offset_palm - length must be 3')
    }
    // Serialize message field [gyr_offset_palm]
    bufferOffset = _arraySerializer.uint16(obj.gyr_offset_palm, buffer, bufferOffset, 3);
    // Check that the constant length array field [acc_radius] has the right length
    if (obj.acc_radius.length !== 4) {
      throw new Error('Unable to serialize array field acc_radius - length must be 4')
    }
    // Serialize message field [acc_radius]
    bufferOffset = _arraySerializer.uint16(obj.acc_radius, buffer, bufferOffset, 4);
    // Check that the constant length array field [gyr_radius] has the right length
    if (obj.gyr_radius.length !== 4) {
      throw new Error('Unable to serialize array field gyr_radius - length must be 4')
    }
    // Serialize message field [gyr_radius]
    bufferOffset = _arraySerializer.uint16(obj.gyr_radius, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImuCalibrationData
    let len;
    let data = new ImuCalibrationData(null);
    // Deserialize message field [acc_offset_f1]
    data.acc_offset_f1 = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [acc_offset_f2]
    data.acc_offset_f2 = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [acc_offset_f3]
    data.acc_offset_f3 = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [acc_offset_palm]
    data.acc_offset_palm = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [mag_offset_f1]
    data.mag_offset_f1 = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [mag_offset_f2]
    data.mag_offset_f2 = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [mag_offset_f3]
    data.mag_offset_f3 = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [mag_offset_palm]
    data.mag_offset_palm = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [gyr_offset_f1]
    data.gyr_offset_f1 = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [gyr_offset_f2]
    data.gyr_offset_f2 = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [gyr_offset_f3]
    data.gyr_offset_f3 = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [gyr_offset_palm]
    data.gyr_offset_palm = _arrayDeserializer.uint16(buffer, bufferOffset, 3)
    // Deserialize message field [acc_radius]
    data.acc_radius = _arrayDeserializer.uint16(buffer, bufferOffset, 4)
    // Deserialize message field [gyr_radius]
    data.gyr_radius = _arrayDeserializer.uint16(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    return 88;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reflex_msgs2/ImuCalibrationData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1ef3e1b102a68813a645fa51b970838b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Acceleration offsets for each dimension in the format [x,y,z]
    uint16[3] acc_offset_f1
    uint16[3] acc_offset_f2
    uint16[3] acc_offset_f3
    uint16[3] acc_offset_palm
    
    # Magnetometer offsets for each dimension in the format [x,y,z]
    uint16[3] mag_offset_f1
    uint16[3] mag_offset_f2
    uint16[3] mag_offset_f3
    uint16[3] mag_offset_palm
    
    # Gyroscope offsets for each dimension in the format [x,y,z]
    uint16[3] gyr_offset_f1
    uint16[3] gyr_offset_f2
    uint16[3] gyr_offset_f3
    uint16[3] gyr_offset_palm
    
    # Accelerometer and Gyroscope radius in the format [f1,f2,f3,palm]
    uint16[4] acc_radius
    uint16[4] gyr_radius
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ImuCalibrationData(null);
    if (msg.acc_offset_f1 !== undefined) {
      resolved.acc_offset_f1 = msg.acc_offset_f1;
    }
    else {
      resolved.acc_offset_f1 = new Array(3).fill(0)
    }

    if (msg.acc_offset_f2 !== undefined) {
      resolved.acc_offset_f2 = msg.acc_offset_f2;
    }
    else {
      resolved.acc_offset_f2 = new Array(3).fill(0)
    }

    if (msg.acc_offset_f3 !== undefined) {
      resolved.acc_offset_f3 = msg.acc_offset_f3;
    }
    else {
      resolved.acc_offset_f3 = new Array(3).fill(0)
    }

    if (msg.acc_offset_palm !== undefined) {
      resolved.acc_offset_palm = msg.acc_offset_palm;
    }
    else {
      resolved.acc_offset_palm = new Array(3).fill(0)
    }

    if (msg.mag_offset_f1 !== undefined) {
      resolved.mag_offset_f1 = msg.mag_offset_f1;
    }
    else {
      resolved.mag_offset_f1 = new Array(3).fill(0)
    }

    if (msg.mag_offset_f2 !== undefined) {
      resolved.mag_offset_f2 = msg.mag_offset_f2;
    }
    else {
      resolved.mag_offset_f2 = new Array(3).fill(0)
    }

    if (msg.mag_offset_f3 !== undefined) {
      resolved.mag_offset_f3 = msg.mag_offset_f3;
    }
    else {
      resolved.mag_offset_f3 = new Array(3).fill(0)
    }

    if (msg.mag_offset_palm !== undefined) {
      resolved.mag_offset_palm = msg.mag_offset_palm;
    }
    else {
      resolved.mag_offset_palm = new Array(3).fill(0)
    }

    if (msg.gyr_offset_f1 !== undefined) {
      resolved.gyr_offset_f1 = msg.gyr_offset_f1;
    }
    else {
      resolved.gyr_offset_f1 = new Array(3).fill(0)
    }

    if (msg.gyr_offset_f2 !== undefined) {
      resolved.gyr_offset_f2 = msg.gyr_offset_f2;
    }
    else {
      resolved.gyr_offset_f2 = new Array(3).fill(0)
    }

    if (msg.gyr_offset_f3 !== undefined) {
      resolved.gyr_offset_f3 = msg.gyr_offset_f3;
    }
    else {
      resolved.gyr_offset_f3 = new Array(3).fill(0)
    }

    if (msg.gyr_offset_palm !== undefined) {
      resolved.gyr_offset_palm = msg.gyr_offset_palm;
    }
    else {
      resolved.gyr_offset_palm = new Array(3).fill(0)
    }

    if (msg.acc_radius !== undefined) {
      resolved.acc_radius = msg.acc_radius;
    }
    else {
      resolved.acc_radius = new Array(4).fill(0)
    }

    if (msg.gyr_radius !== undefined) {
      resolved.gyr_radius = msg.gyr_radius;
    }
    else {
      resolved.gyr_radius = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = ImuCalibrationData;
