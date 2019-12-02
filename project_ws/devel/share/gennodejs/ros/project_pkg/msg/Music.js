// Auto-generated. Do not edit!

// (in-package project_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Note = require('./Note.js');

//-----------------------------------------------------------

class Music {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.notes = null;
    }
    else {
      if (initObj.hasOwnProperty('notes')) {
        this.notes = initObj.notes
      }
      else {
        this.notes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Music
    // Serialize message field [notes]
    // Serialize the length for message field [notes]
    bufferOffset = _serializer.uint32(obj.notes.length, buffer, bufferOffset);
    obj.notes.forEach((val) => {
      bufferOffset = Note.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Music
    let len;
    let data = new Music(null);
    // Deserialize message field [notes]
    // Deserialize array length for message field [notes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.notes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.notes[i] = Note.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.notes.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'project_pkg/Music';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '336014d282bfb921a1299cf4a485af06';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    project_pkg/Note[] notes
    
    ================================================================================
    MSG: project_pkg/Note
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
    const resolved = new Music(null);
    if (msg.notes !== undefined) {
      resolved.notes = new Array(msg.notes.length);
      for (let i = 0; i < resolved.notes.length; ++i) {
        resolved.notes[i] = Note.Resolve(msg.notes[i]);
      }
    }
    else {
      resolved.notes = []
    }

    return resolved;
    }
};

module.exports = Music;
