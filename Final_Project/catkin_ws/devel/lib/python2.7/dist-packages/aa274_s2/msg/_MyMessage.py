# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from aa274_s2/MyMessage.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class MyMessage(genpy.Message):
  _md5sum = "239c79299df89155b13fc3feae022d2f"
  _type = "aa274_s2/MyMessage"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string text
bool flag
char symbol
float64 ratio
time clock
duration period
int64 para
"""
  __slots__ = ['text','flag','symbol','ratio','clock','period','para']
  _slot_types = ['string','bool','char','float64','time','duration','int64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       text,flag,symbol,ratio,clock,period,para

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MyMessage, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.text is None:
        self.text = ''
      if self.flag is None:
        self.flag = False
      if self.symbol is None:
        self.symbol = 0
      if self.ratio is None:
        self.ratio = 0.
      if self.clock is None:
        self.clock = genpy.Time()
      if self.period is None:
        self.period = genpy.Duration()
      if self.para is None:
        self.para = 0
    else:
      self.text = ''
      self.flag = False
      self.symbol = 0
      self.ratio = 0.
      self.clock = genpy.Time()
      self.period = genpy.Duration()
      self.para = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.text
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_2Bd2I2iq().pack(_x.flag, _x.symbol, _x.ratio, _x.clock.secs, _x.clock.nsecs, _x.period.secs, _x.period.nsecs, _x.para))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.clock is None:
        self.clock = genpy.Time()
      if self.period is None:
        self.period = genpy.Duration()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.text = str[start:end].decode('utf-8')
      else:
        self.text = str[start:end]
      _x = self
      start = end
      end += 34
      (_x.flag, _x.symbol, _x.ratio, _x.clock.secs, _x.clock.nsecs, _x.period.secs, _x.period.nsecs, _x.para,) = _get_struct_2Bd2I2iq().unpack(str[start:end])
      self.flag = bool(self.flag)
      self.clock.canon()
      self.period.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.text
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_2Bd2I2iq().pack(_x.flag, _x.symbol, _x.ratio, _x.clock.secs, _x.clock.nsecs, _x.period.secs, _x.period.nsecs, _x.para))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.clock is None:
        self.clock = genpy.Time()
      if self.period is None:
        self.period = genpy.Duration()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.text = str[start:end].decode('utf-8')
      else:
        self.text = str[start:end]
      _x = self
      start = end
      end += 34
      (_x.flag, _x.symbol, _x.ratio, _x.clock.secs, _x.clock.nsecs, _x.period.secs, _x.period.nsecs, _x.para,) = _get_struct_2Bd2I2iq().unpack(str[start:end])
      self.flag = bool(self.flag)
      self.clock.canon()
      self.period.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2Bd2I2iq = None
def _get_struct_2Bd2I2iq():
    global _struct_2Bd2I2iq
    if _struct_2Bd2I2iq is None:
        _struct_2Bd2I2iq = struct.Struct("<2Bd2I2iq")
    return _struct_2Bd2I2iq