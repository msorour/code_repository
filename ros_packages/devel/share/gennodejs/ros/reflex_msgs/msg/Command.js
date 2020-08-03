// Auto-generated. Do not edit!

// (in-package reflex_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PoseCommand = require('./PoseCommand.js');
let VelocityCommand = require('./VelocityCommand.js');

//-----------------------------------------------------------

class Command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
      this.velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new PoseCommand();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new VelocityCommand();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Command
    // Serialize message field [pose]
    bufferOffset = PoseCommand.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = VelocityCommand.serialize(obj.velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Command
    let len;
    let data = new Command(null);
    // Deserialize message field [pose]
    data.pose = PoseCommand.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = VelocityCommand.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reflex_msgs/Command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bcad31578e17a6697c2483ccda6d52eb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    PoseCommand pose
    VelocityCommand velocity
    
    ================================================================================
    MSG: reflex_msgs/PoseCommand
    # Position in radians of various motors
    float64 f1
    float64 f2
    float64 f3
    float64 preshape
    
    ================================================================================
    MSG: reflex_msgs/VelocityCommand
    # Velocity in radians/second of various motors
    float64 f1
    float64 f2
    float64 f3
    float64 preshape
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Command(null);
    if (msg.pose !== undefined) {
      resolved.pose = PoseCommand.Resolve(msg.pose)
    }
    else {
      resolved.pose = new PoseCommand()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = VelocityCommand.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new VelocityCommand()
    }

    return resolved;
    }
};

module.exports = Command;
