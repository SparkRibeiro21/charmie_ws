# generated from rosidl_generator_py/resource/_idl.py.em
# with input from charmie_interfaces:msg/Keypoints.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Keypoints(type):
    """Metaclass of message 'Keypoints'."""

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
                'charmie_interfaces.msg.Keypoints')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__keypoints
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__keypoints
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__keypoints
            cls._TYPE_SUPPORT = module.type_support_msg__msg__keypoints
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__keypoints

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Keypoints(metaclass=Metaclass_Keypoints):
    """Message class 'Keypoints'."""

    __slots__ = [
        '_index_person',
        '_average_distance',
        '_standard_deviation',
        '_key_p0_x',
        '_key_p0_y',
        '_key_p1_x',
        '_key_p1_y',
        '_key_p2_x',
        '_key_p2_y',
        '_key_p3_x',
        '_key_p3_y',
        '_key_p4_x',
        '_key_p4_y',
        '_key_p5_x',
        '_key_p5_y',
        '_key_p6_x',
        '_key_p6_y',
        '_key_p7_x',
        '_key_p7_y',
        '_key_p8_x',
        '_key_p8_y',
        '_key_p9_x',
        '_key_p9_y',
        '_key_p10_x',
        '_key_p10_y',
        '_key_p11_x',
        '_key_p11_y',
        '_key_p12_x',
        '_key_p12_y',
        '_key_p13_x',
        '_key_p13_y',
        '_key_p14_x',
        '_key_p14_y',
        '_key_p15_x',
        '_key_p15_y',
        '_key_p16_x',
        '_key_p16_y',
    ]

    _fields_and_field_types = {
        'index_person': 'int32',
        'average_distance': 'float',
        'standard_deviation': 'float',
        'key_p0_x': 'int32',
        'key_p0_y': 'int32',
        'key_p1_x': 'int32',
        'key_p1_y': 'int32',
        'key_p2_x': 'int32',
        'key_p2_y': 'int32',
        'key_p3_x': 'int32',
        'key_p3_y': 'int32',
        'key_p4_x': 'int32',
        'key_p4_y': 'int32',
        'key_p5_x': 'int32',
        'key_p5_y': 'int32',
        'key_p6_x': 'int32',
        'key_p6_y': 'int32',
        'key_p7_x': 'int32',
        'key_p7_y': 'int32',
        'key_p8_x': 'int32',
        'key_p8_y': 'int32',
        'key_p9_x': 'int32',
        'key_p9_y': 'int32',
        'key_p10_x': 'int32',
        'key_p10_y': 'int32',
        'key_p11_x': 'int32',
        'key_p11_y': 'int32',
        'key_p12_x': 'int32',
        'key_p12_y': 'int32',
        'key_p13_x': 'int32',
        'key_p13_y': 'int32',
        'key_p14_x': 'int32',
        'key_p14_y': 'int32',
        'key_p15_x': 'int32',
        'key_p15_y': 'int32',
        'key_p16_x': 'int32',
        'key_p16_y': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.index_person = kwargs.get('index_person', int())
        self.average_distance = kwargs.get('average_distance', float())
        self.standard_deviation = kwargs.get('standard_deviation', float())
        self.key_p0_x = kwargs.get('key_p0_x', int())
        self.key_p0_y = kwargs.get('key_p0_y', int())
        self.key_p1_x = kwargs.get('key_p1_x', int())
        self.key_p1_y = kwargs.get('key_p1_y', int())
        self.key_p2_x = kwargs.get('key_p2_x', int())
        self.key_p2_y = kwargs.get('key_p2_y', int())
        self.key_p3_x = kwargs.get('key_p3_x', int())
        self.key_p3_y = kwargs.get('key_p3_y', int())
        self.key_p4_x = kwargs.get('key_p4_x', int())
        self.key_p4_y = kwargs.get('key_p4_y', int())
        self.key_p5_x = kwargs.get('key_p5_x', int())
        self.key_p5_y = kwargs.get('key_p5_y', int())
        self.key_p6_x = kwargs.get('key_p6_x', int())
        self.key_p6_y = kwargs.get('key_p6_y', int())
        self.key_p7_x = kwargs.get('key_p7_x', int())
        self.key_p7_y = kwargs.get('key_p7_y', int())
        self.key_p8_x = kwargs.get('key_p8_x', int())
        self.key_p8_y = kwargs.get('key_p8_y', int())
        self.key_p9_x = kwargs.get('key_p9_x', int())
        self.key_p9_y = kwargs.get('key_p9_y', int())
        self.key_p10_x = kwargs.get('key_p10_x', int())
        self.key_p10_y = kwargs.get('key_p10_y', int())
        self.key_p11_x = kwargs.get('key_p11_x', int())
        self.key_p11_y = kwargs.get('key_p11_y', int())
        self.key_p12_x = kwargs.get('key_p12_x', int())
        self.key_p12_y = kwargs.get('key_p12_y', int())
        self.key_p13_x = kwargs.get('key_p13_x', int())
        self.key_p13_y = kwargs.get('key_p13_y', int())
        self.key_p14_x = kwargs.get('key_p14_x', int())
        self.key_p14_y = kwargs.get('key_p14_y', int())
        self.key_p15_x = kwargs.get('key_p15_x', int())
        self.key_p15_y = kwargs.get('key_p15_y', int())
        self.key_p16_x = kwargs.get('key_p16_x', int())
        self.key_p16_y = kwargs.get('key_p16_y', int())

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
        if self.index_person != other.index_person:
            return False
        if self.average_distance != other.average_distance:
            return False
        if self.standard_deviation != other.standard_deviation:
            return False
        if self.key_p0_x != other.key_p0_x:
            return False
        if self.key_p0_y != other.key_p0_y:
            return False
        if self.key_p1_x != other.key_p1_x:
            return False
        if self.key_p1_y != other.key_p1_y:
            return False
        if self.key_p2_x != other.key_p2_x:
            return False
        if self.key_p2_y != other.key_p2_y:
            return False
        if self.key_p3_x != other.key_p3_x:
            return False
        if self.key_p3_y != other.key_p3_y:
            return False
        if self.key_p4_x != other.key_p4_x:
            return False
        if self.key_p4_y != other.key_p4_y:
            return False
        if self.key_p5_x != other.key_p5_x:
            return False
        if self.key_p5_y != other.key_p5_y:
            return False
        if self.key_p6_x != other.key_p6_x:
            return False
        if self.key_p6_y != other.key_p6_y:
            return False
        if self.key_p7_x != other.key_p7_x:
            return False
        if self.key_p7_y != other.key_p7_y:
            return False
        if self.key_p8_x != other.key_p8_x:
            return False
        if self.key_p8_y != other.key_p8_y:
            return False
        if self.key_p9_x != other.key_p9_x:
            return False
        if self.key_p9_y != other.key_p9_y:
            return False
        if self.key_p10_x != other.key_p10_x:
            return False
        if self.key_p10_y != other.key_p10_y:
            return False
        if self.key_p11_x != other.key_p11_x:
            return False
        if self.key_p11_y != other.key_p11_y:
            return False
        if self.key_p12_x != other.key_p12_x:
            return False
        if self.key_p12_y != other.key_p12_y:
            return False
        if self.key_p13_x != other.key_p13_x:
            return False
        if self.key_p13_y != other.key_p13_y:
            return False
        if self.key_p14_x != other.key_p14_x:
            return False
        if self.key_p14_y != other.key_p14_y:
            return False
        if self.key_p15_x != other.key_p15_x:
            return False
        if self.key_p15_y != other.key_p15_y:
            return False
        if self.key_p16_x != other.key_p16_x:
            return False
        if self.key_p16_y != other.key_p16_y:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def index_person(self):
        """Message field 'index_person'."""
        return self._index_person

    @index_person.setter
    def index_person(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'index_person' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'index_person' field must be an integer in [-2147483648, 2147483647]"
        self._index_person = value

    @property
    def average_distance(self):
        """Message field 'average_distance'."""
        return self._average_distance

    @average_distance.setter
    def average_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'average_distance' field must be of type 'float'"
        self._average_distance = value

    @property
    def standard_deviation(self):
        """Message field 'standard_deviation'."""
        return self._standard_deviation

    @standard_deviation.setter
    def standard_deviation(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'standard_deviation' field must be of type 'float'"
        self._standard_deviation = value

    @property
    def key_p0_x(self):
        """Message field 'key_p0_x'."""
        return self._key_p0_x

    @key_p0_x.setter
    def key_p0_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p0_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p0_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p0_x = value

    @property
    def key_p0_y(self):
        """Message field 'key_p0_y'."""
        return self._key_p0_y

    @key_p0_y.setter
    def key_p0_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p0_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p0_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p0_y = value

    @property
    def key_p1_x(self):
        """Message field 'key_p1_x'."""
        return self._key_p1_x

    @key_p1_x.setter
    def key_p1_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p1_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p1_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p1_x = value

    @property
    def key_p1_y(self):
        """Message field 'key_p1_y'."""
        return self._key_p1_y

    @key_p1_y.setter
    def key_p1_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p1_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p1_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p1_y = value

    @property
    def key_p2_x(self):
        """Message field 'key_p2_x'."""
        return self._key_p2_x

    @key_p2_x.setter
    def key_p2_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p2_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p2_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p2_x = value

    @property
    def key_p2_y(self):
        """Message field 'key_p2_y'."""
        return self._key_p2_y

    @key_p2_y.setter
    def key_p2_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p2_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p2_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p2_y = value

    @property
    def key_p3_x(self):
        """Message field 'key_p3_x'."""
        return self._key_p3_x

    @key_p3_x.setter
    def key_p3_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p3_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p3_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p3_x = value

    @property
    def key_p3_y(self):
        """Message field 'key_p3_y'."""
        return self._key_p3_y

    @key_p3_y.setter
    def key_p3_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p3_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p3_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p3_y = value

    @property
    def key_p4_x(self):
        """Message field 'key_p4_x'."""
        return self._key_p4_x

    @key_p4_x.setter
    def key_p4_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p4_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p4_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p4_x = value

    @property
    def key_p4_y(self):
        """Message field 'key_p4_y'."""
        return self._key_p4_y

    @key_p4_y.setter
    def key_p4_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p4_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p4_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p4_y = value

    @property
    def key_p5_x(self):
        """Message field 'key_p5_x'."""
        return self._key_p5_x

    @key_p5_x.setter
    def key_p5_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p5_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p5_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p5_x = value

    @property
    def key_p5_y(self):
        """Message field 'key_p5_y'."""
        return self._key_p5_y

    @key_p5_y.setter
    def key_p5_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p5_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p5_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p5_y = value

    @property
    def key_p6_x(self):
        """Message field 'key_p6_x'."""
        return self._key_p6_x

    @key_p6_x.setter
    def key_p6_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p6_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p6_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p6_x = value

    @property
    def key_p6_y(self):
        """Message field 'key_p6_y'."""
        return self._key_p6_y

    @key_p6_y.setter
    def key_p6_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p6_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p6_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p6_y = value

    @property
    def key_p7_x(self):
        """Message field 'key_p7_x'."""
        return self._key_p7_x

    @key_p7_x.setter
    def key_p7_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p7_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p7_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p7_x = value

    @property
    def key_p7_y(self):
        """Message field 'key_p7_y'."""
        return self._key_p7_y

    @key_p7_y.setter
    def key_p7_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p7_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p7_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p7_y = value

    @property
    def key_p8_x(self):
        """Message field 'key_p8_x'."""
        return self._key_p8_x

    @key_p8_x.setter
    def key_p8_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p8_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p8_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p8_x = value

    @property
    def key_p8_y(self):
        """Message field 'key_p8_y'."""
        return self._key_p8_y

    @key_p8_y.setter
    def key_p8_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p8_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p8_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p8_y = value

    @property
    def key_p9_x(self):
        """Message field 'key_p9_x'."""
        return self._key_p9_x

    @key_p9_x.setter
    def key_p9_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p9_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p9_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p9_x = value

    @property
    def key_p9_y(self):
        """Message field 'key_p9_y'."""
        return self._key_p9_y

    @key_p9_y.setter
    def key_p9_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p9_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p9_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p9_y = value

    @property
    def key_p10_x(self):
        """Message field 'key_p10_x'."""
        return self._key_p10_x

    @key_p10_x.setter
    def key_p10_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p10_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p10_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p10_x = value

    @property
    def key_p10_y(self):
        """Message field 'key_p10_y'."""
        return self._key_p10_y

    @key_p10_y.setter
    def key_p10_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p10_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p10_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p10_y = value

    @property
    def key_p11_x(self):
        """Message field 'key_p11_x'."""
        return self._key_p11_x

    @key_p11_x.setter
    def key_p11_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p11_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p11_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p11_x = value

    @property
    def key_p11_y(self):
        """Message field 'key_p11_y'."""
        return self._key_p11_y

    @key_p11_y.setter
    def key_p11_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p11_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p11_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p11_y = value

    @property
    def key_p12_x(self):
        """Message field 'key_p12_x'."""
        return self._key_p12_x

    @key_p12_x.setter
    def key_p12_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p12_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p12_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p12_x = value

    @property
    def key_p12_y(self):
        """Message field 'key_p12_y'."""
        return self._key_p12_y

    @key_p12_y.setter
    def key_p12_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p12_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p12_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p12_y = value

    @property
    def key_p13_x(self):
        """Message field 'key_p13_x'."""
        return self._key_p13_x

    @key_p13_x.setter
    def key_p13_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p13_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p13_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p13_x = value

    @property
    def key_p13_y(self):
        """Message field 'key_p13_y'."""
        return self._key_p13_y

    @key_p13_y.setter
    def key_p13_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p13_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p13_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p13_y = value

    @property
    def key_p14_x(self):
        """Message field 'key_p14_x'."""
        return self._key_p14_x

    @key_p14_x.setter
    def key_p14_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p14_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p14_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p14_x = value

    @property
    def key_p14_y(self):
        """Message field 'key_p14_y'."""
        return self._key_p14_y

    @key_p14_y.setter
    def key_p14_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p14_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p14_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p14_y = value

    @property
    def key_p15_x(self):
        """Message field 'key_p15_x'."""
        return self._key_p15_x

    @key_p15_x.setter
    def key_p15_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p15_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p15_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p15_x = value

    @property
    def key_p15_y(self):
        """Message field 'key_p15_y'."""
        return self._key_p15_y

    @key_p15_y.setter
    def key_p15_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p15_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p15_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p15_y = value

    @property
    def key_p16_x(self):
        """Message field 'key_p16_x'."""
        return self._key_p16_x

    @key_p16_x.setter
    def key_p16_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p16_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p16_x' field must be an integer in [-2147483648, 2147483647]"
        self._key_p16_x = value

    @property
    def key_p16_y(self):
        """Message field 'key_p16_y'."""
        return self._key_p16_y

    @key_p16_y.setter
    def key_p16_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_p16_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_p16_y' field must be an integer in [-2147483648, 2147483647]"
        self._key_p16_y = value
