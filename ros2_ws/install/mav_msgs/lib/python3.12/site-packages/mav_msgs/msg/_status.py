# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mav_msgs:msg/Status.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Status(type):
    """Metaclass of message 'Status'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'RC_COMMAND_ATTITUDE': 'attitude_thrust',
        'RC_COMMAND_ATTITUDE_HEIGHT': 'attitude_height',
        'RC_COMMAND_POSITION': 'position',
        'MOTOR_STATUS_RUNNING': 'running',
        'MOTOR_STATUS_STOPPED': 'stopped',
        'MOTOR_STATUS_STARTING': 'starting',
        'MOTOR_STATUS_STOPPING': 'stopping',
        'GPS_STATUS_LOCK': 'lock',
        'GPS_STATUS_NO_LOCK': 'no_lock',
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mav_msgs.msg.Status')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'RC_COMMAND_ATTITUDE': cls.__constants['RC_COMMAND_ATTITUDE'],
            'RC_COMMAND_ATTITUDE_HEIGHT': cls.__constants['RC_COMMAND_ATTITUDE_HEIGHT'],
            'RC_COMMAND_POSITION': cls.__constants['RC_COMMAND_POSITION'],
            'MOTOR_STATUS_RUNNING': cls.__constants['MOTOR_STATUS_RUNNING'],
            'MOTOR_STATUS_STOPPED': cls.__constants['MOTOR_STATUS_STOPPED'],
            'MOTOR_STATUS_STARTING': cls.__constants['MOTOR_STATUS_STARTING'],
            'MOTOR_STATUS_STOPPING': cls.__constants['MOTOR_STATUS_STOPPING'],
            'GPS_STATUS_LOCK': cls.__constants['GPS_STATUS_LOCK'],
            'GPS_STATUS_NO_LOCK': cls.__constants['GPS_STATUS_NO_LOCK'],
        }

    @property
    def RC_COMMAND_ATTITUDE(self):
        """Message constant 'RC_COMMAND_ATTITUDE'."""
        return Metaclass_Status.__constants['RC_COMMAND_ATTITUDE']

    @property
    def RC_COMMAND_ATTITUDE_HEIGHT(self):
        """Message constant 'RC_COMMAND_ATTITUDE_HEIGHT'."""
        return Metaclass_Status.__constants['RC_COMMAND_ATTITUDE_HEIGHT']

    @property
    def RC_COMMAND_POSITION(self):
        """Message constant 'RC_COMMAND_POSITION'."""
        return Metaclass_Status.__constants['RC_COMMAND_POSITION']

    @property
    def MOTOR_STATUS_RUNNING(self):
        """Message constant 'MOTOR_STATUS_RUNNING'."""
        return Metaclass_Status.__constants['MOTOR_STATUS_RUNNING']

    @property
    def MOTOR_STATUS_STOPPED(self):
        """Message constant 'MOTOR_STATUS_STOPPED'."""
        return Metaclass_Status.__constants['MOTOR_STATUS_STOPPED']

    @property
    def MOTOR_STATUS_STARTING(self):
        """Message constant 'MOTOR_STATUS_STARTING'."""
        return Metaclass_Status.__constants['MOTOR_STATUS_STARTING']

    @property
    def MOTOR_STATUS_STOPPING(self):
        """Message constant 'MOTOR_STATUS_STOPPING'."""
        return Metaclass_Status.__constants['MOTOR_STATUS_STOPPING']

    @property
    def GPS_STATUS_LOCK(self):
        """Message constant 'GPS_STATUS_LOCK'."""
        return Metaclass_Status.__constants['GPS_STATUS_LOCK']

    @property
    def GPS_STATUS_NO_LOCK(self):
        """Message constant 'GPS_STATUS_NO_LOCK'."""
        return Metaclass_Status.__constants['GPS_STATUS_NO_LOCK']


class Status(metaclass=Metaclass_Status):
    """
    Message class 'Status'.

    Constants:
      RC_COMMAND_ATTITUDE
      RC_COMMAND_ATTITUDE_HEIGHT
      RC_COMMAND_POSITION
      MOTOR_STATUS_RUNNING
      MOTOR_STATUS_STOPPED
      MOTOR_STATUS_STARTING
      MOTOR_STATUS_STOPPING
      GPS_STATUS_LOCK
      GPS_STATUS_NO_LOCK
    """

    __slots__ = [
        '_header',
        '_vehicle_name',
        '_vehicle_type',
        '_battery_voltage',
        '_rc_command_mode',
        '_command_interface_enabled',
        '_flight_time',
        '_system_uptime',
        '_cpu_load',
        '_motor_status',
        '_in_air',
        '_gps_status',
        '_gps_num_satellites',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'vehicle_name': 'string',
        'vehicle_type': 'string',
        'battery_voltage': 'float',
        'rc_command_mode': 'string',
        'command_interface_enabled': 'boolean',
        'flight_time': 'float',
        'system_uptime': 'float',
        'cpu_load': 'float',
        'motor_status': 'string',
        'in_air': 'boolean',
        'gps_status': 'string',
        'gps_num_satellites': 'int32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.vehicle_name = kwargs.get('vehicle_name', str())
        self.vehicle_type = kwargs.get('vehicle_type', str())
        self.battery_voltage = kwargs.get('battery_voltage', float())
        self.rc_command_mode = kwargs.get('rc_command_mode', str())
        self.command_interface_enabled = kwargs.get('command_interface_enabled', bool())
        self.flight_time = kwargs.get('flight_time', float())
        self.system_uptime = kwargs.get('system_uptime', float())
        self.cpu_load = kwargs.get('cpu_load', float())
        self.motor_status = kwargs.get('motor_status', str())
        self.in_air = kwargs.get('in_air', bool())
        self.gps_status = kwargs.get('gps_status', str())
        self.gps_num_satellites = kwargs.get('gps_num_satellites', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.vehicle_name != other.vehicle_name:
            return False
        if self.vehicle_type != other.vehicle_type:
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.rc_command_mode != other.rc_command_mode:
            return False
        if self.command_interface_enabled != other.command_interface_enabled:
            return False
        if self.flight_time != other.flight_time:
            return False
        if self.system_uptime != other.system_uptime:
            return False
        if self.cpu_load != other.cpu_load:
            return False
        if self.motor_status != other.motor_status:
            return False
        if self.in_air != other.in_air:
            return False
        if self.gps_status != other.gps_status:
            return False
        if self.gps_num_satellites != other.gps_num_satellites:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def vehicle_name(self):
        """Message field 'vehicle_name'."""
        return self._vehicle_name

    @vehicle_name.setter
    def vehicle_name(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'vehicle_name' field must be of type 'str'"
        self._vehicle_name = value

    @builtins.property
    def vehicle_type(self):
        """Message field 'vehicle_type'."""
        return self._vehicle_type

    @vehicle_type.setter
    def vehicle_type(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'vehicle_type' field must be of type 'str'"
        self._vehicle_type = value

    @builtins.property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_voltage = value

    @builtins.property
    def rc_command_mode(self):
        """Message field 'rc_command_mode'."""
        return self._rc_command_mode

    @rc_command_mode.setter
    def rc_command_mode(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'rc_command_mode' field must be of type 'str'"
        self._rc_command_mode = value

    @builtins.property
    def command_interface_enabled(self):
        """Message field 'command_interface_enabled'."""
        return self._command_interface_enabled

    @command_interface_enabled.setter
    def command_interface_enabled(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'command_interface_enabled' field must be of type 'bool'"
        self._command_interface_enabled = value

    @builtins.property
    def flight_time(self):
        """Message field 'flight_time'."""
        return self._flight_time

    @flight_time.setter
    def flight_time(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'flight_time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'flight_time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._flight_time = value

    @builtins.property
    def system_uptime(self):
        """Message field 'system_uptime'."""
        return self._system_uptime

    @system_uptime.setter
    def system_uptime(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'system_uptime' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'system_uptime' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._system_uptime = value

    @builtins.property
    def cpu_load(self):
        """Message field 'cpu_load'."""
        return self._cpu_load

    @cpu_load.setter
    def cpu_load(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'cpu_load' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cpu_load' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cpu_load = value

    @builtins.property
    def motor_status(self):
        """Message field 'motor_status'."""
        return self._motor_status

    @motor_status.setter
    def motor_status(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'motor_status' field must be of type 'str'"
        self._motor_status = value

    @builtins.property
    def in_air(self):
        """Message field 'in_air'."""
        return self._in_air

    @in_air.setter
    def in_air(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'in_air' field must be of type 'bool'"
        self._in_air = value

    @builtins.property
    def gps_status(self):
        """Message field 'gps_status'."""
        return self._gps_status

    @gps_status.setter
    def gps_status(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'gps_status' field must be of type 'str'"
        self._gps_status = value

    @builtins.property
    def gps_num_satellites(self):
        """Message field 'gps_num_satellites'."""
        return self._gps_num_satellites

    @gps_num_satellites.setter
    def gps_num_satellites(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'gps_num_satellites' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'gps_num_satellites' field must be an integer in [-2147483648, 2147483647]"
        self._gps_num_satellites = value
