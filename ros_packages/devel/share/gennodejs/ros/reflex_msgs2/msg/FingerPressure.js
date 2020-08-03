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

class FingerPressure {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sensor = null;
    }
    else {
      if (initObj.hasOwnProperty('sensor')) {
        this.sensor = initObj.sensor
      }
      else {
        this.sensor = new Array(14).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FingerPressure
    // Check that the constant length array field [sensor] has the right length
    if (obj.sensor.length !== 14) {
      throw new Error('Unable to serialize array field sensor - length must be 14')
    }
    // Serialize message field [sensor]
    bufferOffset = _arraySerializer.uint16(obj.sensor, buffer, bufferOffset, 14);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FingerPressure
    let len;
    let data = new FingerPressure(null);
    // Deserialize message field [sensor]
    data.sensor = _arrayDeserializer.uint16(buffer, bufferOffset, 14)
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reflex_msgs2/FingerPressure';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2fecdd6322f86c468a82244a96936129';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # message for pressure on a single finger
    uint16[14] sensor    	# The sensors enumerate from the base of the finger to the tip
    						# There are 5 on the proximal link, 4 on the distal
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FingerPressure(null);
    if (msg.sensor !== undefined) {
      resolved.sensor = msg.sensor;
    }
    else {
      resolved.sensor = new Array(14).fill(0)
    }

    return resolved;
    }
};

module.exports = FingerPressure;
