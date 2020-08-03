// Auto-generated. Do not edit!

// (in-package reflex_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FingerPressure = require('../msg/FingerPressure.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetTactileThresholdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.finger = null;
    }
    else {
      if (initObj.hasOwnProperty('finger')) {
        this.finger = initObj.finger
      }
      else {
        this.finger = new Array(3).fill(new FingerPressure());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetTactileThresholdRequest
    // Check that the constant length array field [finger] has the right length
    if (obj.finger.length !== 3) {
      throw new Error('Unable to serialize array field finger - length must be 3')
    }
    // Serialize message field [finger]
    obj.finger.forEach((val) => {
      bufferOffset = FingerPressure.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTactileThresholdRequest
    let len;
    let data = new SetTactileThresholdRequest(null);
    // Deserialize message field [finger]
    len = 3;
    data.finger = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.finger[i] = FingerPressure.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 18;
  }

  static datatype() {
    // Returns string type for a service object
    return 'reflex_msgs/SetTactileThresholdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '01cec83f9d223083364c730460331524';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # message for calling setting pressure thresholds on various fingers
    FingerPressure[3] finger
    
    ================================================================================
    MSG: reflex_msgs/FingerPressure
    # message for pressure on a single finger
    uint16[9] sensor    	# The sensors enumerate from the base of the finger to the tip
    						# There are 5 on the proximal link, 4 on the distal
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetTactileThresholdRequest(null);
    if (msg.finger !== undefined) {
      resolved.finger = new Array(3)
      for (let i = 0; i < resolved.finger.length; ++i) {
        if (msg.finger.length > i) {
          resolved.finger[i] = FingerPressure.Resolve(msg.finger[i]);
        }
        else {
          resolved.finger[i] = new FingerPressure();
        }
      }
    }
    else {
      resolved.finger = new Array(3).fill(new FingerPressure())
    }

    return resolved;
    }
};

class SetTactileThresholdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetTactileThresholdResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTactileThresholdResponse
    let len;
    let data = new SetTactileThresholdResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'reflex_msgs/SetTactileThresholdResponse';
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
    const resolved = new SetTactileThresholdResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetTactileThresholdRequest,
  Response: SetTactileThresholdResponse,
  md5sum() { return '01cec83f9d223083364c730460331524'; },
  datatype() { return 'reflex_msgs/SetTactileThreshold'; }
};
