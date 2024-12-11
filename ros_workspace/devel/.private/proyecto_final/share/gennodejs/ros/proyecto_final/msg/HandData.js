// Auto-generated. Do not edit!

// (in-package proyecto_final.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class HandData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.is_open = null;
      this.is_peace = null;
      this.hand_detected = null;
      this.is_dino = null;
      this.is_dislike = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('is_open')) {
        this.is_open = initObj.is_open
      }
      else {
        this.is_open = false;
      }
      if (initObj.hasOwnProperty('is_peace')) {
        this.is_peace = initObj.is_peace
      }
      else {
        this.is_peace = false;
      }
      if (initObj.hasOwnProperty('hand_detected')) {
        this.hand_detected = initObj.hand_detected
      }
      else {
        this.hand_detected = false;
      }
      if (initObj.hasOwnProperty('is_dino')) {
        this.is_dino = initObj.is_dino
      }
      else {
        this.is_dino = false;
      }
      if (initObj.hasOwnProperty('is_dislike')) {
        this.is_dislike = initObj.is_dislike
      }
      else {
        this.is_dislike = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HandData
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    // Serialize message field [is_open]
    bufferOffset = _serializer.bool(obj.is_open, buffer, bufferOffset);
    // Serialize message field [is_peace]
    bufferOffset = _serializer.bool(obj.is_peace, buffer, bufferOffset);
    // Serialize message field [hand_detected]
    bufferOffset = _serializer.bool(obj.hand_detected, buffer, bufferOffset);
    // Serialize message field [is_dino]
    bufferOffset = _serializer.bool(obj.is_dino, buffer, bufferOffset);
    // Serialize message field [is_dislike]
    bufferOffset = _serializer.bool(obj.is_dislike, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HandData
    let len;
    let data = new HandData(null);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [is_open]
    data.is_open = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_peace]
    data.is_peace = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [hand_detected]
    data.hand_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_dino]
    data.is_dino = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_dislike]
    data.is_dislike = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'proyecto_final/HandData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9b0493b2ed6710620a749baf2ddc5457';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 x
    float32 y
    float32 z
    bool is_open
    bool is_peace
    bool hand_detected
    bool is_dino
    bool is_dislike
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HandData(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.is_open !== undefined) {
      resolved.is_open = msg.is_open;
    }
    else {
      resolved.is_open = false
    }

    if (msg.is_peace !== undefined) {
      resolved.is_peace = msg.is_peace;
    }
    else {
      resolved.is_peace = false
    }

    if (msg.hand_detected !== undefined) {
      resolved.hand_detected = msg.hand_detected;
    }
    else {
      resolved.hand_detected = false
    }

    if (msg.is_dino !== undefined) {
      resolved.is_dino = msg.is_dino;
    }
    else {
      resolved.is_dino = false
    }

    if (msg.is_dislike !== undefined) {
      resolved.is_dislike = msg.is_dislike;
    }
    else {
      resolved.is_dislike = false
    }

    return resolved;
    }
};

module.exports = HandData;
