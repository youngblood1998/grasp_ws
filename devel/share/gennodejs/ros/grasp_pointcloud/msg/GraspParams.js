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

class GraspParams {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.rotate_angle = null;
      this.tilt_angle = null;
      this.grasp_width_first = null;
      this.grasp_width_second = null;
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
      if (initObj.hasOwnProperty('rotate_angle')) {
        this.rotate_angle = initObj.rotate_angle
      }
      else {
        this.rotate_angle = 0.0;
      }
      if (initObj.hasOwnProperty('tilt_angle')) {
        this.tilt_angle = initObj.tilt_angle
      }
      else {
        this.tilt_angle = 0.0;
      }
      if (initObj.hasOwnProperty('grasp_width_first')) {
        this.grasp_width_first = initObj.grasp_width_first
      }
      else {
        this.grasp_width_first = 0.0;
      }
      if (initObj.hasOwnProperty('grasp_width_second')) {
        this.grasp_width_second = initObj.grasp_width_second
      }
      else {
        this.grasp_width_second = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GraspParams
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [rotate_angle]
    bufferOffset = _serializer.float64(obj.rotate_angle, buffer, bufferOffset);
    // Serialize message field [tilt_angle]
    bufferOffset = _serializer.float64(obj.tilt_angle, buffer, bufferOffset);
    // Serialize message field [grasp_width_first]
    bufferOffset = _serializer.float64(obj.grasp_width_first, buffer, bufferOffset);
    // Serialize message field [grasp_width_second]
    bufferOffset = _serializer.float64(obj.grasp_width_second, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GraspParams
    let len;
    let data = new GraspParams(null);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rotate_angle]
    data.rotate_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tilt_angle]
    data.tilt_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [grasp_width_first]
    data.grasp_width_first = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [grasp_width_second]
    data.grasp_width_second = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'grasp_pointcloud/GraspParams';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28bf0de5a6fc99a5d51ec496c974cccc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x
    float64 y
    float64 z
    float64 rotate_angle
    float64 tilt_angle
    float64 grasp_width_first
    float64 grasp_width_second
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GraspParams(null);
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

    if (msg.rotate_angle !== undefined) {
      resolved.rotate_angle = msg.rotate_angle;
    }
    else {
      resolved.rotate_angle = 0.0
    }

    if (msg.tilt_angle !== undefined) {
      resolved.tilt_angle = msg.tilt_angle;
    }
    else {
      resolved.tilt_angle = 0.0
    }

    if (msg.grasp_width_first !== undefined) {
      resolved.grasp_width_first = msg.grasp_width_first;
    }
    else {
      resolved.grasp_width_first = 0.0
    }

    if (msg.grasp_width_second !== undefined) {
      resolved.grasp_width_second = msg.grasp_width_second;
    }
    else {
      resolved.grasp_width_second = 0.0
    }

    return resolved;
    }
};

module.exports = GraspParams;
