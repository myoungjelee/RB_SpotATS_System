# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ats_msgs:msg/AtsState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AtsState(type):
    """Metaclass of message 'AtsState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ats_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ats_msgs.msg.AtsState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ats_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ats_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ats_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ats_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ats_state

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AtsState(metaclass=Metaclass_AtsState):
    """Message class 'AtsState'."""

    __slots__ = [
        '_stamp',
        '_system_state',
        '_pose_frame',
        '_pose_x',
        '_pose_y',
        '_pose_yaw',
        '_vel_vx',
        '_vel_vy',
        '_vel_wz',
        '_battery_soc',
        '_battery_voltage',
        '_primary_id',
        '_lost_sec',
        '_vision_json',
        '_plan_json',
        '_current_index',
        '_queue_status',
        '_history_json',
        '_roe_ok',
        '_safe_backstop',
        '_max_speed',
        '_last_violation',
        '_events_json',
    ]

    _fields_and_field_types = {
        'stamp': 'builtin_interfaces/Time',
        'system_state': 'string',
        'pose_frame': 'string',
        'pose_x': 'double',
        'pose_y': 'double',
        'pose_yaw': 'double',
        'vel_vx': 'double',
        'vel_vy': 'double',
        'vel_wz': 'double',
        'battery_soc': 'double',
        'battery_voltage': 'double',
        'primary_id': 'int32',
        'lost_sec': 'double',
        'vision_json': 'string',
        'plan_json': 'string',
        'current_index': 'int32',
        'queue_status': 'string',
        'history_json': 'string',
        'roe_ok': 'boolean',
        'safe_backstop': 'boolean',
        'max_speed': 'double',
        'last_violation': 'string',
        'events_json': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())
        self.system_state = kwargs.get('system_state', str())
        self.pose_frame = kwargs.get('pose_frame', str())
        self.pose_x = kwargs.get('pose_x', float())
        self.pose_y = kwargs.get('pose_y', float())
        self.pose_yaw = kwargs.get('pose_yaw', float())
        self.vel_vx = kwargs.get('vel_vx', float())
        self.vel_vy = kwargs.get('vel_vy', float())
        self.vel_wz = kwargs.get('vel_wz', float())
        self.battery_soc = kwargs.get('battery_soc', float())
        self.battery_voltage = kwargs.get('battery_voltage', float())
        self.primary_id = kwargs.get('primary_id', int())
        self.lost_sec = kwargs.get('lost_sec', float())
        self.vision_json = kwargs.get('vision_json', str())
        self.plan_json = kwargs.get('plan_json', str())
        self.current_index = kwargs.get('current_index', int())
        self.queue_status = kwargs.get('queue_status', str())
        self.history_json = kwargs.get('history_json', str())
        self.roe_ok = kwargs.get('roe_ok', bool())
        self.safe_backstop = kwargs.get('safe_backstop', bool())
        self.max_speed = kwargs.get('max_speed', float())
        self.last_violation = kwargs.get('last_violation', str())
        self.events_json = kwargs.get('events_json', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.stamp != other.stamp:
            return False
        if self.system_state != other.system_state:
            return False
        if self.pose_frame != other.pose_frame:
            return False
        if self.pose_x != other.pose_x:
            return False
        if self.pose_y != other.pose_y:
            return False
        if self.pose_yaw != other.pose_yaw:
            return False
        if self.vel_vx != other.vel_vx:
            return False
        if self.vel_vy != other.vel_vy:
            return False
        if self.vel_wz != other.vel_wz:
            return False
        if self.battery_soc != other.battery_soc:
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.primary_id != other.primary_id:
            return False
        if self.lost_sec != other.lost_sec:
            return False
        if self.vision_json != other.vision_json:
            return False
        if self.plan_json != other.plan_json:
            return False
        if self.current_index != other.current_index:
            return False
        if self.queue_status != other.queue_status:
            return False
        if self.history_json != other.history_json:
            return False
        if self.roe_ok != other.roe_ok:
            return False
        if self.safe_backstop != other.safe_backstop:
            return False
        if self.max_speed != other.max_speed:
            return False
        if self.last_violation != other.last_violation:
            return False
        if self.events_json != other.events_json:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value

    @builtins.property
    def system_state(self):
        """Message field 'system_state'."""
        return self._system_state

    @system_state.setter
    def system_state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'system_state' field must be of type 'str'"
        self._system_state = value

    @builtins.property
    def pose_frame(self):
        """Message field 'pose_frame'."""
        return self._pose_frame

    @pose_frame.setter
    def pose_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'pose_frame' field must be of type 'str'"
        self._pose_frame = value

    @builtins.property
    def pose_x(self):
        """Message field 'pose_x'."""
        return self._pose_x

    @pose_x.setter
    def pose_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pose_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pose_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pose_x = value

    @builtins.property
    def pose_y(self):
        """Message field 'pose_y'."""
        return self._pose_y

    @pose_y.setter
    def pose_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pose_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pose_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pose_y = value

    @builtins.property
    def pose_yaw(self):
        """Message field 'pose_yaw'."""
        return self._pose_yaw

    @pose_yaw.setter
    def pose_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pose_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pose_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pose_yaw = value

    @builtins.property
    def vel_vx(self):
        """Message field 'vel_vx'."""
        return self._vel_vx

    @vel_vx.setter
    def vel_vx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vel_vx' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vel_vx' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vel_vx = value

    @builtins.property
    def vel_vy(self):
        """Message field 'vel_vy'."""
        return self._vel_vy

    @vel_vy.setter
    def vel_vy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vel_vy' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vel_vy' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vel_vy = value

    @builtins.property
    def vel_wz(self):
        """Message field 'vel_wz'."""
        return self._vel_wz

    @vel_wz.setter
    def vel_wz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vel_wz' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vel_wz' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vel_wz = value

    @builtins.property
    def battery_soc(self):
        """Message field 'battery_soc'."""
        return self._battery_soc

    @battery_soc.setter
    def battery_soc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_soc' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'battery_soc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._battery_soc = value

    @builtins.property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'battery_voltage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._battery_voltage = value

    @builtins.property
    def primary_id(self):
        """Message field 'primary_id'."""
        return self._primary_id

    @primary_id.setter
    def primary_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'primary_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'primary_id' field must be an integer in [-2147483648, 2147483647]"
        self._primary_id = value

    @builtins.property
    def lost_sec(self):
        """Message field 'lost_sec'."""
        return self._lost_sec

    @lost_sec.setter
    def lost_sec(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lost_sec' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'lost_sec' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._lost_sec = value

    @builtins.property
    def vision_json(self):
        """Message field 'vision_json'."""
        return self._vision_json

    @vision_json.setter
    def vision_json(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'vision_json' field must be of type 'str'"
        self._vision_json = value

    @builtins.property
    def plan_json(self):
        """Message field 'plan_json'."""
        return self._plan_json

    @plan_json.setter
    def plan_json(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'plan_json' field must be of type 'str'"
        self._plan_json = value

    @builtins.property
    def current_index(self):
        """Message field 'current_index'."""
        return self._current_index

    @current_index.setter
    def current_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_index' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'current_index' field must be an integer in [-2147483648, 2147483647]"
        self._current_index = value

    @builtins.property
    def queue_status(self):
        """Message field 'queue_status'."""
        return self._queue_status

    @queue_status.setter
    def queue_status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'queue_status' field must be of type 'str'"
        self._queue_status = value

    @builtins.property
    def history_json(self):
        """Message field 'history_json'."""
        return self._history_json

    @history_json.setter
    def history_json(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'history_json' field must be of type 'str'"
        self._history_json = value

    @builtins.property
    def roe_ok(self):
        """Message field 'roe_ok'."""
        return self._roe_ok

    @roe_ok.setter
    def roe_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'roe_ok' field must be of type 'bool'"
        self._roe_ok = value

    @builtins.property
    def safe_backstop(self):
        """Message field 'safe_backstop'."""
        return self._safe_backstop

    @safe_backstop.setter
    def safe_backstop(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'safe_backstop' field must be of type 'bool'"
        self._safe_backstop = value

    @builtins.property
    def max_speed(self):
        """Message field 'max_speed'."""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_speed = value

    @builtins.property
    def last_violation(self):
        """Message field 'last_violation'."""
        return self._last_violation

    @last_violation.setter
    def last_violation(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'last_violation' field must be of type 'str'"
        self._last_violation = value

    @builtins.property
    def events_json(self):
        """Message field 'events_json'."""
        return self._events_json

    @events_json.setter
    def events_json(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'events_json' field must be of type 'str'"
        self._events_json = value
