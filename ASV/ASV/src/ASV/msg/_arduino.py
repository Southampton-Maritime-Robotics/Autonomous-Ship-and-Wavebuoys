"""autogenerated by genpy from ASV/arduino.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class arduino(genpy.Message):
  _md5sum = "c206d8bc0371a0b0ba4915d805666d5f"
  _type = "ASV/arduino"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 battery_voltage
float32 prop_demand
float32 voltage_demand

"""
  __slots__ = ['battery_voltage','prop_demand','voltage_demand']
  _slot_types = ['float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       battery_voltage,prop_demand,voltage_demand

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(arduino, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.battery_voltage is None:
        self.battery_voltage = 0.
      if self.prop_demand is None:
        self.prop_demand = 0.
      if self.voltage_demand is None:
        self.voltage_demand = 0.
    else:
      self.battery_voltage = 0.
      self.prop_demand = 0.
      self.voltage_demand = 0.

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
      _x = self
      buff.write(_struct_3f.pack(_x.battery_voltage, _x.prop_demand, _x.voltage_demand))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.battery_voltage, _x.prop_demand, _x.voltage_demand,) = _struct_3f.unpack(str[start:end])
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
      _x = self
      buff.write(_struct_3f.pack(_x.battery_voltage, _x.prop_demand, _x.voltage_demand))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.battery_voltage, _x.prop_demand, _x.voltage_demand,) = _struct_3f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3f = struct.Struct("<3f")