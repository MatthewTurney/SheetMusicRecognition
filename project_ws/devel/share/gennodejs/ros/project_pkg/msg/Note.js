// Auto-generated. Do not edit!

// (in-package project_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Note {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.key = null;
      this.duration = null;
      this.rest_before = null;
    }
    else {
      if (initObj.hasOwnProperty('key')) {
        this.key = initObj.key
      }
      else {
        this.key = 0;
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0.0;
      }
      if (initObj.hasOwnProperty('rest_before')) {
        this.rest_before = initObj.rest_before
      }
      else {
        this.rest_before = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Note
    // Serialize message field [key]
    bufferOffset = _serializer.int32(obj.key, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float32(obj.duration, buffer, bufferOffset);
    // Serialize message field [rest_before]
    bufferOffset = _serializer.float32(obj.rest_before, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Note
    let len;
    let data = new Note(null);
    // Deserialize message field [key]
    data.key = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rest_before]
    data.rest_before = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'project_pkg/Note';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ff5d283ef6af3a9b602a9758c35d7198';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 key
    float32 duration
    float32 rest_before
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Note(null);
    if (msg.key !== undefined) {
      resolved.key = msg.key;
    }
    else {
      resolved.key = 0
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0.0
    }

    if (msg.rest_before !== undefined) {
      resolved.rest_before = msg.rest_before;
    }
    else {
      resolved.rest_before = 0.0
    }

    return resolved;
    }
};

module.exports = Note;
