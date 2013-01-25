"""autogenerated by genpy from oryx_msgs/BlobList.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import oryx_msgs.msg

class BlobList(genpy.Message):
  _md5sum = "76c461d0b54f222fd763ee65e508f7e8"
  _type = "oryx_msgs/BlobList"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """Blob[] blobs
int32 blobCount
int8 color

================================================================================
MSG: oryx_msgs/Blob
int32 x
int32 y
int32 size
int32 radius

"""
  __slots__ = ['blobs','blobCount','color']
  _slot_types = ['oryx_msgs/Blob[]','int32','int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       blobs,blobCount,color

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(BlobList, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.blobs is None:
        self.blobs = []
      if self.blobCount is None:
        self.blobCount = 0
      if self.color is None:
        self.color = 0
    else:
      self.blobs = []
      self.blobCount = 0
      self.color = 0

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
      length = len(self.blobs)
      buff.write(_struct_I.pack(length))
      for val1 in self.blobs:
        _x = val1
        buff.write(_struct_4i.pack(_x.x, _x.y, _x.size, _x.radius))
      _x = self
      buff.write(_struct_ib.pack(_x.blobCount, _x.color))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.blobs is None:
        self.blobs = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.blobs = []
      for i in range(0, length):
        val1 = oryx_msgs.msg.Blob()
        _x = val1
        start = end
        end += 16
        (_x.x, _x.y, _x.size, _x.radius,) = _struct_4i.unpack(str[start:end])
        self.blobs.append(val1)
      _x = self
      start = end
      end += 5
      (_x.blobCount, _x.color,) = _struct_ib.unpack(str[start:end])
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
      length = len(self.blobs)
      buff.write(_struct_I.pack(length))
      for val1 in self.blobs:
        _x = val1
        buff.write(_struct_4i.pack(_x.x, _x.y, _x.size, _x.radius))
      _x = self
      buff.write(_struct_ib.pack(_x.blobCount, _x.color))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.blobs is None:
        self.blobs = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.blobs = []
      for i in range(0, length):
        val1 = oryx_msgs.msg.Blob()
        _x = val1
        start = end
        end += 16
        (_x.x, _x.y, _x.size, _x.radius,) = _struct_4i.unpack(str[start:end])
        self.blobs.append(val1)
      _x = self
      start = end
      end += 5
      (_x.blobCount, _x.color,) = _struct_ib.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_ib = struct.Struct("<ib")
_struct_4i = struct.Struct("<4i")
