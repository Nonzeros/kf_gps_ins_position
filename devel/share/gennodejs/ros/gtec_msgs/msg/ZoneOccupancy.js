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

class ZoneOccupancy {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.zoneId = null;
      this.targetIds = null;
      this.count = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('zoneId')) {
        this.zoneId = initObj.zoneId
      }
      else {
        this.zoneId = '';
      }
      if (initObj.hasOwnProperty('targetIds')) {
        this.targetIds = initObj.targetIds
      }
      else {
        this.targetIds = [];
      }
      if (initObj.hasOwnProperty('count')) {
        this.count = initObj.count
      }
      else {
        this.count = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ZoneOccupancy
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [zoneId]
    bufferOffset = _serializer.string(obj.zoneId, buffer, bufferOffset);
    // Serialize message field [targetIds]
    bufferOffset = _arraySerializer.int16(obj.targetIds, buffer, bufferOffset, null);
    // Serialize message field [count]
    bufferOffset = _serializer.int16(obj.count, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ZoneOccupancy
    let len;
    let data = new ZoneOccupancy(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [zoneId]
    data.zoneId = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [targetIds]
    data.targetIds = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [count]
    data.count = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.zoneId);
    length += 2 * object.targetIds.length;
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gtec_msgs/ZoneOccupancy';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ca4a300ff4c235be9feab5048bc4c41f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string zoneId
    int16[] targetIds
    int16 count
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
    const resolved = new ZoneOccupancy(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.zoneId !== undefined) {
      resolved.zoneId = msg.zoneId;
    }
    else {
      resolved.zoneId = ''
    }

    if (msg.targetIds !== undefined) {
      resolved.targetIds = msg.targetIds;
    }
    else {
      resolved.targetIds = []
    }

    if (msg.count !== undefined) {
      resolved.count = msg.count;
    }
    else {
      resolved.count = 0
    }

    return resolved;
    }
};

module.exports = ZoneOccupancy;
