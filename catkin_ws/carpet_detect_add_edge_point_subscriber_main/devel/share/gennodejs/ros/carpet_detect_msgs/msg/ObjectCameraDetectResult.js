// Auto-generated. Do not edit!

// (in-package carpet_detect_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CameraCoordinate = require('./CameraCoordinate.js');

//-----------------------------------------------------------

class ObjectCameraDetectResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.prop = null;
      this.cls_id = null;
      this.coords = null;
    }
    else {
      if (initObj.hasOwnProperty('prop')) {
        this.prop = initObj.prop
      }
      else {
        this.prop = 0.0;
      }
      if (initObj.hasOwnProperty('cls_id')) {
        this.cls_id = initObj.cls_id
      }
      else {
        this.cls_id = 0;
      }
      if (initObj.hasOwnProperty('coords')) {
        this.coords = initObj.coords
      }
      else {
        this.coords = new Array(24).fill(new CameraCoordinate());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectCameraDetectResult
    // Serialize message field [prop]
    bufferOffset = _serializer.float32(obj.prop, buffer, bufferOffset);
    // Serialize message field [cls_id]
    bufferOffset = _serializer.int32(obj.cls_id, buffer, bufferOffset);
    // Check that the constant length array field [coords] has the right length
    if (obj.coords.length !== 24) {
      throw new Error('Unable to serialize array field coords - length must be 24')
    }
    // Serialize message field [coords]
    obj.coords.forEach((val) => {
      bufferOffset = CameraCoordinate.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectCameraDetectResult
    let len;
    let data = new ObjectCameraDetectResult(null);
    // Deserialize message field [prop]
    data.prop = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cls_id]
    data.cls_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [coords]
    len = 24;
    data.coords = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.coords[i] = CameraCoordinate.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'carpet_detect_msgs/ObjectCameraDetectResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d362a9701db027a22375f917c033511';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 prop
    int32 cls_id
    CameraCoordinate[24] coords
    ================================================================================
    MSG: carpet_detect_msgs/CameraCoordinate
    float32 x
    float32 y
    float32 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjectCameraDetectResult(null);
    if (msg.prop !== undefined) {
      resolved.prop = msg.prop;
    }
    else {
      resolved.prop = 0.0
    }

    if (msg.cls_id !== undefined) {
      resolved.cls_id = msg.cls_id;
    }
    else {
      resolved.cls_id = 0
    }

    if (msg.coords !== undefined) {
      resolved.coords = new Array(24)
      for (let i = 0; i < resolved.coords.length; ++i) {
        if (msg.coords.length > i) {
          resolved.coords[i] = CameraCoordinate.Resolve(msg.coords[i]);
        }
        else {
          resolved.coords[i] = new CameraCoordinate();
        }
      }
    }
    else {
      resolved.coords = new Array(24).fill(new CameraCoordinate())
    }

    return resolved;
    }
};

module.exports = ObjectCameraDetectResult;
