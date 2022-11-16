// Auto-generated. Do not edit!

// (in-package grasp_pointcloud.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class AdjustParams {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.y = null;
      this.z = null;
      this.y_rotate_angle = null;
      this.z_rotate_angle = null;
    }
    else {
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
      if (initObj.hasOwnProperty('y_rotate_angle')) {
        this.y_rotate_angle = initObj.y_rotate_angle
      }
      else {
        this.y_rotate_angle = 0.0;
      }
      if (initObj.hasOwnProperty('z_rotate_angle')) {
        this.z_rotate_angle = initObj.z_rotate_angle
      }
      else {
        this.z_rotate_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AdjustParams
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [y_rotate_angle]
    bufferOffset = _serializer.float64(obj.y_rotate_angle, buffer, bufferOffset);
    // Serialize message field [z_rotate_angle]
    bufferOffset = _serializer.float64(obj.z_rotate_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AdjustParams
    let len;
    let data = new AdjustParams(null);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_rotate_angle]
    data.y_rotate_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_rotate_angle]
    data.z_rotate_angle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'grasp_pointcloud/AdjustParams';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '57feed5981af266697038b66e7af4ff4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 y
    float64 z
    float64 y_rotate_angle
    float64 z_rotate_angle
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AdjustParams(null);
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

    if (msg.y_rotate_angle !== undefined) {
      resolved.y_rotate_angle = msg.y_rotate_angle;
    }
    else {
      resolved.y_rotate_angle = 0.0
    }

    if (msg.z_rotate_angle !== undefined) {
      resolved.z_rotate_angle = msg.z_rotate_angle;
    }
    else {
      resolved.z_rotate_angle = 0.0
    }

    return resolved;
    }
};

module.exports = AdjustParams;
