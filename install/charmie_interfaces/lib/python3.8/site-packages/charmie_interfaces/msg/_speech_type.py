# generated from rosidl_generator_py/resource/_idl.py.em
# with input from charmie_interfaces:msg/SpeechType.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SpeechType(type):
    """Metaclass of message 'SpeechType'."""

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
                'charmie_interfaces.msg.SpeechType')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__speech_type
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__speech_type
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__speech_type
            cls._TYPE_SUPPORT = module.type_support_msg__msg__speech_type
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__speech_type

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SpeechType(metaclass=Metaclass_SpeechType):
    """Message class 'SpeechType'."""

    __slots__ = [
        '_yes_or_no',
        '_receptionist',
        '_gpsr',
        '_restaurant',
    ]

    _fields_and_field_types = {
        'yes_or_no': 'boolean',
        'receptionist': 'boolean',
        'gpsr': 'boolean',
        'restaurant': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.yes_or_no = kwargs.get('yes_or_no', bool())
        self.receptionist = kwargs.get('receptionist', bool())
        self.gpsr = kwargs.get('gpsr', bool())
        self.restaurant = kwargs.get('restaurant', bool())

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
        if self.yes_or_no != other.yes_or_no:
            return False
        if self.receptionist != other.receptionist:
            return False
        if self.gpsr != other.gpsr:
            return False
        if self.restaurant != other.restaurant:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def yes_or_no(self):
        """Message field 'yes_or_no'."""
        return self._yes_or_no

    @yes_or_no.setter
    def yes_or_no(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'yes_or_no' field must be of type 'bool'"
        self._yes_or_no = value

    @property
    def receptionist(self):
        """Message field 'receptionist'."""
        return self._receptionist

    @receptionist.setter
    def receptionist(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'receptionist' field must be of type 'bool'"
        self._receptionist = value

    @property
    def gpsr(self):
        """Message field 'gpsr'."""
        return self._gpsr

    @gpsr.setter
    def gpsr(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'gpsr' field must be of type 'bool'"
        self._gpsr = value

    @property
    def restaurant(self):
        """Message field 'restaurant'."""
        return self._restaurant

    @restaurant.setter
    def restaurant(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'restaurant' field must be of type 'bool'"
        self._restaurant = value
