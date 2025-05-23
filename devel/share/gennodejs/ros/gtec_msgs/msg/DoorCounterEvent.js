// Auto-generated. Do not edit!

// (in-package gtec_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DoorCounterEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lth = null;
      this.htl = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lth')) {
        this.lth = initObj.lth
      }
      else {
        this.lth = 0;
      }
      if (initObj.hasOwnProperty('htl')) {
        this.htl = initObj.htl
      }
      else {
        this.htl = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DoorCounterEvent
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lth]
    bufferOffset = _serializer.int16(obj.lth, buffer, bufferOffset);
    // Serialize message field [htl]
    bufferOffset = _serializer.int16(obj.htl, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DoorCounterEvent
    let len;
    let data = new DoorCounterEvent(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lth]
    data.lth = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [htl]
    data.htl = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gtec_msgs/DoorCounterEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8e9928bb0cb3ba644932e73b3a112f9c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int16 lth
    int16 htl
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DoorCounterEvent(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lth !== undefined) {
      resolved.lth = msg.lth;
    }
    else {
      resolved.lth = 0
    }

    if (msg.htl !== undefined) {
      resolved.htl = msg.htl;
    }
    else {
      resolved.htl = 0
    }

    return resolved;
    }
};

module.exports = DoorCounterEvent;
