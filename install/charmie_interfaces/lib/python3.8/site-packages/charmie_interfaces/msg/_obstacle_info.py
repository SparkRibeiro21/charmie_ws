# generated from rosidl_generator_py/resource/_idl.py.em
# with input from charmie_interfaces:msg/ObstacleInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ObstacleInfo(type):
    """Metaclass of message 'ObstacleInfo'."""

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
            module = import_type_support('charmie_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'charmie_interfaces.msg.ObstacleInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__obstacle_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__obstacle_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__obstacle_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__obstacle_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__obstacle_info

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ObstacleInfo(metaclass=Metaclass_ObstacleInfo):
    """Message class 'ObstacleInfo'."""

    __slots__ = [
        '_alfa',
        '_dist',
        '_length_cm',
        '_length_degrees',
    ]

    _fields_and_field_types = {
        'alfa': 'float',
        'dist': 'float',
        'length_cm': 'float',
        'length_degrees': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.alfa = kwargs.get('alfa', float())
        self.dist = kwargs.get('dist', float())
        self.length_cm = kwargs.get('length_cm', float())
        self.length_degrees = kwargs.get('length_degrees', float())

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
        if self.alfa != other.alfa:
            return False
        if self.dist != other.dist:
            return False
        if self.length_cm != other.length_cm:
            return False
        if self.length_degrees != other.length_degrees:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def alfa(self):
        """Message field 'alfa'."""
        return self._alfa

    @alfa.setter
    def alfa(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'alfa' field must be of type 'float'"
        self._alfa = value

    @property
    def dist(self):
        """Message field 'dist'."""
        return self._dist

    @dist.setter
    def dist(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dist' field must be of type 'float'"
        self._dist = value

    @property
    def length_cm(self):
        """Message field 'length_cm'."""
        return self._length_cm

    @length_cm.setter
    def length_cm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'length_cm' field must be of type 'float'"
        self._length_cm = value

    @property
    def length_degrees(self):
        """Message field 'length_degrees'."""
        return self._length_degrees

    @length_degrees.setter
    def length_degrees(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'length_degrees' field must be of type 'float'"
        self._length_degrees = value
