// Auto-generated. Do not edit!

// (in-package ros_gazebo_plugin_comm.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Vector7d {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new Array(7).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Vector7d
    // Check that the constant length array field [data] has the right length
    if (obj.data.length !== 7) {
      throw new Error('Unable to serialize array field data - length must be 7')
    }
    // Serialize message field [data]
    bufferOffset = _arraySerializer.float32(obj.data, buffer, bufferOffset, 7);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Vector7d
    let len;
    let data = new Vector7d(null);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.float32(buffer, bufferOffset, 7)
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_gazebo_plugin_comm/Vector7d';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e4844fe6885794a5295df1d81cbae79f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[7] data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Vector7d(null);
    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = new Array(7).fill(0)
    }

    return resolved;
    }
};

module.exports = Vector7d;
