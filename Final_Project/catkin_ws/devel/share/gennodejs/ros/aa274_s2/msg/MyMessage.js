// Auto-generated. Do not edit!

// (in-package aa274_s2.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MyMessage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.text = null;
      this.flag = null;
      this.symbol = null;
      this.ratio = null;
      this.clock = null;
      this.period = null;
      this.para = null;
    }
    else {
      if (initObj.hasOwnProperty('text')) {
        this.text = initObj.text
      }
      else {
        this.text = '';
      }
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = false;
      }
      if (initObj.hasOwnProperty('symbol')) {
        this.symbol = initObj.symbol
      }
      else {
        this.symbol = 0;
      }
      if (initObj.hasOwnProperty('ratio')) {
        this.ratio = initObj.ratio
      }
      else {
        this.ratio = 0.0;
      }
      if (initObj.hasOwnProperty('clock')) {
        this.clock = initObj.clock
      }
      else {
        this.clock = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('period')) {
        this.period = initObj.period
      }
      else {
        this.period = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('para')) {
        this.para = initObj.para
      }
      else {
        this.para = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MyMessage
    // Serialize message field [text]
    bufferOffset = _serializer.string(obj.text, buffer, bufferOffset);
    // Serialize message field [flag]
    bufferOffset = _serializer.bool(obj.flag, buffer, bufferOffset);
    // Serialize message field [symbol]
    bufferOffset = _serializer.char(obj.symbol, buffer, bufferOffset);
    // Serialize message field [ratio]
    bufferOffset = _serializer.float64(obj.ratio, buffer, bufferOffset);
    // Serialize message field [clock]
    bufferOffset = _serializer.time(obj.clock, buffer, bufferOffset);
    // Serialize message field [period]
    bufferOffset = _serializer.duration(obj.period, buffer, bufferOffset);
    // Serialize message field [para]
    bufferOffset = _serializer.int64(obj.para, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MyMessage
    let len;
    let data = new MyMessage(null);
    // Deserialize message field [text]
    data.text = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [flag]
    data.flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [symbol]
    data.symbol = _deserializer.char(buffer, bufferOffset);
    // Deserialize message field [ratio]
    data.ratio = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [clock]
    data.clock = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [period]
    data.period = _deserializer.duration(buffer, bufferOffset);
    // Deserialize message field [para]
    data.para = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.text.length;
    return length + 38;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aa274_s2/MyMessage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '239c79299df89155b13fc3feae022d2f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string text
    bool flag
    char symbol
    float64 ratio
    time clock
    duration period
    int64 para
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MyMessage(null);
    if (msg.text !== undefined) {
      resolved.text = msg.text;
    }
    else {
      resolved.text = ''
    }

    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = false
    }

    if (msg.symbol !== undefined) {
      resolved.symbol = msg.symbol;
    }
    else {
      resolved.symbol = 0
    }

    if (msg.ratio !== undefined) {
      resolved.ratio = msg.ratio;
    }
    else {
      resolved.ratio = 0.0
    }

    if (msg.clock !== undefined) {
      resolved.clock = msg.clock;
    }
    else {
      resolved.clock = {secs: 0, nsecs: 0}
    }

    if (msg.period !== undefined) {
      resolved.period = msg.period;
    }
    else {
      resolved.period = {secs: 0, nsecs: 0}
    }

    if (msg.para !== undefined) {
      resolved.para = msg.para;
    }
    else {
      resolved.para = 0
    }

    return resolved;
    }
};

module.exports = MyMessage;
