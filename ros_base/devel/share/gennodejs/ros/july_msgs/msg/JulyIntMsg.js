// Auto-generated. Do not edit!

// (in-package july_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class JulyIntMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.msg_a = null;
      this.msg_b = null;
    }
    else {
      if (initObj.hasOwnProperty('msg_a')) {
        this.msg_a = initObj.msg_a
      }
      else {
        this.msg_a = 0;
      }
      if (initObj.hasOwnProperty('msg_b')) {
        this.msg_b = initObj.msg_b
      }
      else {
        this.msg_b = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JulyIntMsg
    // Serialize message field [msg_a]
    bufferOffset = _serializer.int32(obj.msg_a, buffer, bufferOffset);
    // Serialize message field [msg_b]
    bufferOffset = _serializer.int32(obj.msg_b, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JulyIntMsg
    let len;
    let data = new JulyIntMsg(null);
    // Deserialize message field [msg_a]
    data.msg_a = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [msg_b]
    data.msg_b = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'july_msgs/JulyIntMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd09a3f7a5a94e2671033e5fccab47e31';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 msg_a
    int32 msg_b
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JulyIntMsg(null);
    if (msg.msg_a !== undefined) {
      resolved.msg_a = msg.msg_a;
    }
    else {
      resolved.msg_a = 0
    }

    if (msg.msg_b !== undefined) {
      resolved.msg_b = msg.msg_b;
    }
    else {
      resolved.msg_b = 0
    }

    return resolved;
    }
};

module.exports = JulyIntMsg;
