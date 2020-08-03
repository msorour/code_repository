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

class RadianServoCommands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.radian_commands = null;
    }
    else {
      if (initObj.hasOwnProperty('radian_commands')) {
        this.radian_commands = initObj.radian_commands
      }
      else {
        this.radian_commands = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RadianServoCommands
    // Check that the constant length array field [radian_commands] has the right length
    if (obj.radian_commands.length !== 4) {
      throw new Error('Unable to serialize array field radian_commands - length must be 4')
    }
    // Serialize message field [radian_commands]
    bufferOffset = _arraySerializer.float32(obj.radian_commands, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RadianServoCommands
    let len;
    let data = new RadianServoCommands(null);
    // Deserialize message field [radian_commands]
    data.radian_commands = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reflex_msgs/RadianServoCommands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ba1e88d9da1745cdc1900895d8c434b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Sets either radian position or radian/s velocity, depending on control mode
    float32[4] radian_commands
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RadianServoCommands(null);
    if (msg.radian_commands !== undefined) {
      resolved.radian_commands = msg.radian_commands;
    }
    else {
      resolved.radian_commands = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = RadianServoCommands;
