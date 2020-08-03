// Auto-generated. Do not edit!

// (in-package reflex_one_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetSpeedRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.motor = null;
    }
    else {
      if (initObj.hasOwnProperty('motor')) {
        this.motor = initObj.motor
      }
      else {
        this.motor = new Array(5).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetSpeedRequest
    // Check that the constant length array field [motor] has the right length
    if (obj.motor.length !== 5) {
      throw new Error('Unable to serialize array field motor - length must be 5')
    }
    // Serialize message field [motor]
    bufferOffset = _arraySerializer.float64(obj.motor, buffer, bufferOffset, 5);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSpeedRequest
    let len;
    let data = new SetSpeedRequest(null);
    // Deserialize message field [motor]
    data.motor = _arrayDeserializer.float64(buffer, bufferOffset, 5)
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a service object
    return 'reflex_one_msgs/SetSpeedRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '04171ebc3d171407826cbe90dea5b330';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # message for calling hand commands
    float64[5] motor
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetSpeedRequest(null);
    if (msg.motor !== undefined) {
      resolved.motor = msg.motor;
    }
    else {
      resolved.motor = new Array(5).fill(0)
    }

    return resolved;
    }
};

class SetSpeedResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetSpeedResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSpeedResponse
    let len;
    let data = new SetSpeedResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'reflex_one_msgs/SetSpeedResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetSpeedResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetSpeedRequest,
  Response: SetSpeedResponse,
  md5sum() { return '04171ebc3d171407826cbe90dea5b330'; },
  datatype() { return 'reflex_one_msgs/SetSpeed'; }
};
