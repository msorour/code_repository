// Auto-generated. Do not edit!

// (in-package reflex_msgs2.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class DistalRotationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.palm_imu_quat = null;
      this.joint_angle = null;
      this.proximal = null;
      this.finger_imu_quat = null;
    }
    else {
      if (initObj.hasOwnProperty('palm_imu_quat')) {
        this.palm_imu_quat = initObj.palm_imu_quat
      }
      else {
        this.palm_imu_quat = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('joint_angle')) {
        this.joint_angle = initObj.joint_angle
      }
      else {
        this.joint_angle = 0.0;
      }
      if (initObj.hasOwnProperty('proximal')) {
        this.proximal = initObj.proximal
      }
      else {
        this.proximal = 0.0;
      }
      if (initObj.hasOwnProperty('finger_imu_quat')) {
        this.finger_imu_quat = initObj.finger_imu_quat
      }
      else {
        this.finger_imu_quat = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DistalRotationRequest
    // Check that the constant length array field [palm_imu_quat] has the right length
    if (obj.palm_imu_quat.length !== 4) {
      throw new Error('Unable to serialize array field palm_imu_quat - length must be 4')
    }
    // Serialize message field [palm_imu_quat]
    bufferOffset = _arraySerializer.float32(obj.palm_imu_quat, buffer, bufferOffset, 4);
    // Serialize message field [joint_angle]
    bufferOffset = _serializer.float32(obj.joint_angle, buffer, bufferOffset);
    // Serialize message field [proximal]
    bufferOffset = _serializer.float32(obj.proximal, buffer, bufferOffset);
    // Check that the constant length array field [finger_imu_quat] has the right length
    if (obj.finger_imu_quat.length !== 4) {
      throw new Error('Unable to serialize array field finger_imu_quat - length must be 4')
    }
    // Serialize message field [finger_imu_quat]
    bufferOffset = _arraySerializer.float32(obj.finger_imu_quat, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DistalRotationRequest
    let len;
    let data = new DistalRotationRequest(null);
    // Deserialize message field [palm_imu_quat]
    data.palm_imu_quat = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [joint_angle]
    data.joint_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [proximal]
    data.proximal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [finger_imu_quat]
    data.finger_imu_quat = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a service object
    return 'reflex_msgs2/DistalRotationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '95c6d42d4818f388398c524197385198';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # message for reading IMU rotations
    float32[4] palm_imu_quat
    float32 joint_angle
    float32 proximal
    float32[4] finger_imu_quat
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DistalRotationRequest(null);
    if (msg.palm_imu_quat !== undefined) {
      resolved.palm_imu_quat = msg.palm_imu_quat;
    }
    else {
      resolved.palm_imu_quat = new Array(4).fill(0)
    }

    if (msg.joint_angle !== undefined) {
      resolved.joint_angle = msg.joint_angle;
    }
    else {
      resolved.joint_angle = 0.0
    }

    if (msg.proximal !== undefined) {
      resolved.proximal = msg.proximal;
    }
    else {
      resolved.proximal = 0.0
    }

    if (msg.finger_imu_quat !== undefined) {
      resolved.finger_imu_quat = msg.finger_imu_quat;
    }
    else {
      resolved.finger_imu_quat = new Array(4).fill(0)
    }

    return resolved;
    }
};

class DistalRotationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rotation = null;
    }
    else {
      if (initObj.hasOwnProperty('rotation')) {
        this.rotation = initObj.rotation
      }
      else {
        this.rotation = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DistalRotationResponse
    // Check that the constant length array field [rotation] has the right length
    if (obj.rotation.length !== 3) {
      throw new Error('Unable to serialize array field rotation - length must be 3')
    }
    // Serialize message field [rotation]
    bufferOffset = _arraySerializer.float32(obj.rotation, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DistalRotationResponse
    let len;
    let data = new DistalRotationResponse(null);
    // Deserialize message field [rotation]
    data.rotation = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'reflex_msgs2/DistalRotationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8d2ccfdfc1a5ba6babe40fd5c7c04dee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[3] rotation
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DistalRotationResponse(null);
    if (msg.rotation !== undefined) {
      resolved.rotation = msg.rotation;
    }
    else {
      resolved.rotation = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = {
  Request: DistalRotationRequest,
  Response: DistalRotationResponse,
  md5sum() { return '96436694e12dc909600e6760ed1174cd'; },
  datatype() { return 'reflex_msgs2/DistalRotation'; }
};
