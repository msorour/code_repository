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

class RawServoCommands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.raw_positions = null;
    }
    else {
      if (initObj.hasOwnProperty('raw_positions')) {
        this.raw_positions = initObj.raw_positions
      }
      else {
        this.raw_positions = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RawServoCommands
    // Check that the constant length array field [raw_positions] has the right length
    if (obj.raw_positions.length !== 4) {
      throw new Error('Unable to serialize array field raw_positions - length must be 4')
    }
    // Serialize message field [raw_positions]
    bufferOffset = _arraySerializer.uint16(obj.raw_positions, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RawServoCommands
    let len;
    let data = new RawServoCommands(null);
    // Deserialize message field [raw_positions]
    data.raw_positions = _arrayDeserializer.uint16(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reflex_msgs/RawServoCommands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aea318ab5663e05395d3ab895fd67e1e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16[4] raw_positions
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RawServoCommands(null);
    if (msg.raw_positions !== undefined) {
      resolved.raw_positions = msg.raw_positions;
    }
    else {
      resolved.raw_positions = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = RawServoCommands;
