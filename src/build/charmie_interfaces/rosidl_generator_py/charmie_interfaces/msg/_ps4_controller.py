# generated from rosidl_generator_py/resource/_idl.py.em
# with input from charmie_interfaces:msg/PS4Controller.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PS4Controller(type):
    """Metaclass of message 'PS4Controller'."""

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
                'charmie_interfaces.msg.PS4Controller')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ps4_controller
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ps4_controller
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ps4_controller
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ps4_controller
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ps4_controller

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PS4Controller(metaclass=Metaclass_PS4Controller):
    """Message class 'PS4Controller'."""

    __slots__ = [
        '_triangle',
        '_circle',
        '_cross',
        '_square',
        '_arrow_up',
        '_arrow_right',
        '_arrow_down',
        '_arrow_left',
        '_l1',
        '_r1',
        '_l3',
        '_r3',
        '_share',
        '_options',
        '_ps',
        '_l3_ang',
        '_l3_dist',
        '_l3_xx',
        '_l3_yy',
        '_r3_ang',
        '_r3_dist',
        '_r3_xx',
        '_r3_yy',
        '_l2',
        '_r2',
    ]

    _fields_and_field_types = {
        'triangle': 'uint8',
        'circle': 'uint8',
        'cross': 'uint8',
        'square': 'uint8',
        'arrow_up': 'uint8',
        'arrow_right': 'uint8',
        'arrow_down': 'uint8',
        'arrow_left': 'uint8',
        'l1': 'uint8',
        'r1': 'uint8',
        'l3': 'uint8',
        'r3': 'uint8',
        'share': 'uint8',
        'options': 'uint8',
        'ps': 'uint8',
        'l3_ang': 'float',
        'l3_dist': 'float',
        'l3_xx': 'float',
        'l3_yy': 'float',
        'r3_ang': 'float',
        'r3_dist': 'float',
        'r3_xx': 'float',
        'r3_yy': 'float',
        'l2': 'float',
        'r2': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.triangle = kwargs.get('triangle', int())
        self.circle = kwargs.get('circle', int())
        self.cross = kwargs.get('cross', int())
        self.square = kwargs.get('square', int())
        self.arrow_up = kwargs.get('arrow_up', int())
        self.arrow_right = kwargs.get('arrow_right', int())
        self.arrow_down = kwargs.get('arrow_down', int())
        self.arrow_left = kwargs.get('arrow_left', int())
        self.l1 = kwargs.get('l1', int())
        self.r1 = kwargs.get('r1', int())
        self.l3 = kwargs.get('l3', int())
        self.r3 = kwargs.get('r3', int())
        self.share = kwargs.get('share', int())
        self.options = kwargs.get('options', int())
        self.ps = kwargs.get('ps', int())
        self.l3_ang = kwargs.get('l3_ang', float())
        self.l3_dist = kwargs.get('l3_dist', float())
        self.l3_xx = kwargs.get('l3_xx', float())
        self.l3_yy = kwargs.get('l3_yy', float())
        self.r3_ang = kwargs.get('r3_ang', float())
        self.r3_dist = kwargs.get('r3_dist', float())
        self.r3_xx = kwargs.get('r3_xx', float())
        self.r3_yy = kwargs.get('r3_yy', float())
        self.l2 = kwargs.get('l2', float())
        self.r2 = kwargs.get('r2', float())

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
        if self.triangle != other.triangle:
            return False
        if self.circle != other.circle:
            return False
        if self.cross != other.cross:
            return False
        if self.square != other.square:
            return False
        if self.arrow_up != other.arrow_up:
            return False
        if self.arrow_right != other.arrow_right:
            return False
        if self.arrow_down != other.arrow_down:
            return False
        if self.arrow_left != other.arrow_left:
            return False
        if self.l1 != other.l1:
            return False
        if self.r1 != other.r1:
            return False
        if self.l3 != other.l3:
            return False
        if self.r3 != other.r3:
            return False
        if self.share != other.share:
            return False
        if self.options != other.options:
            return False
        if self.ps != other.ps:
            return False
        if self.l3_ang != other.l3_ang:
            return False
        if self.l3_dist != other.l3_dist:
            return False
        if self.l3_xx != other.l3_xx:
            return False
        if self.l3_yy != other.l3_yy:
            return False
        if self.r3_ang != other.r3_ang:
            return False
        if self.r3_dist != other.r3_dist:
            return False
        if self.r3_xx != other.r3_xx:
            return False
        if self.r3_yy != other.r3_yy:
            return False
        if self.l2 != other.l2:
            return False
        if self.r2 != other.r2:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def triangle(self):
        """Message field 'triangle'."""
        return self._triangle

    @triangle.setter
    def triangle(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'triangle' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'triangle' field must be an unsigned integer in [0, 255]"
        self._triangle = value

    @property
    def circle(self):
        """Message field 'circle'."""
        return self._circle

    @circle.setter
    def circle(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'circle' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'circle' field must be an unsigned integer in [0, 255]"
        self._circle = value

    @property
    def cross(self):
        """Message field 'cross'."""
        return self._cross

    @cross.setter
    def cross(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cross' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'cross' field must be an unsigned integer in [0, 255]"
        self._cross = value

    @property
    def square(self):
        """Message field 'square'."""
        return self._square

    @square.setter
    def square(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'square' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'square' field must be an unsigned integer in [0, 255]"
        self._square = value

    @property
    def arrow_up(self):
        """Message field 'arrow_up'."""
        return self._arrow_up

    @arrow_up.setter
    def arrow_up(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'arrow_up' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'arrow_up' field must be an unsigned integer in [0, 255]"
        self._arrow_up = value

    @property
    def arrow_right(self):
        """Message field 'arrow_right'."""
        return self._arrow_right

    @arrow_right.setter
    def arrow_right(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'arrow_right' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'arrow_right' field must be an unsigned integer in [0, 255]"
        self._arrow_right = value

    @property
    def arrow_down(self):
        """Message field 'arrow_down'."""
        return self._arrow_down

    @arrow_down.setter
    def arrow_down(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'arrow_down' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'arrow_down' field must be an unsigned integer in [0, 255]"
        self._arrow_down = value

    @property
    def arrow_left(self):
        """Message field 'arrow_left'."""
        return self._arrow_left

    @arrow_left.setter
    def arrow_left(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'arrow_left' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'arrow_left' field must be an unsigned integer in [0, 255]"
        self._arrow_left = value

    @property
    def l1(self):
        """Message field 'l1'."""
        return self._l1

    @l1.setter
    def l1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'l1' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'l1' field must be an unsigned integer in [0, 255]"
        self._l1 = value

    @property
    def r1(self):
        """Message field 'r1'."""
        return self._r1

    @r1.setter
    def r1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'r1' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'r1' field must be an unsigned integer in [0, 255]"
        self._r1 = value

    @property
    def l3(self):
        """Message field 'l3'."""
        return self._l3

    @l3.setter
    def l3(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'l3' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'l3' field must be an unsigned integer in [0, 255]"
        self._l3 = value

    @property
    def r3(self):
        """Message field 'r3'."""
        return self._r3

    @r3.setter
    def r3(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'r3' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'r3' field must be an unsigned integer in [0, 255]"
        self._r3 = value

    @property
    def share(self):
        """Message field 'share'."""
        return self._share

    @share.setter
    def share(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'share' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'share' field must be an unsigned integer in [0, 255]"
        self._share = value

    @property
    def options(self):
        """Message field 'options'."""
        return self._options

    @options.setter
    def options(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'options' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'options' field must be an unsigned integer in [0, 255]"
        self._options = value

    @property
    def ps(self):
        """Message field 'ps'."""
        return self._ps

    @ps.setter
    def ps(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ps' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'ps' field must be an unsigned integer in [0, 255]"
        self._ps = value

    @property
    def l3_ang(self):
        """Message field 'l3_ang'."""
        return self._l3_ang

    @l3_ang.setter
    def l3_ang(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l3_ang' field must be of type 'float'"
        self._l3_ang = value

    @property
    def l3_dist(self):
        """Message field 'l3_dist'."""
        return self._l3_dist

    @l3_dist.setter
    def l3_dist(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l3_dist' field must be of type 'float'"
        self._l3_dist = value

    @property
    def l3_xx(self):
        """Message field 'l3_xx'."""
        return self._l3_xx

    @l3_xx.setter
    def l3_xx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l3_xx' field must be of type 'float'"
        self._l3_xx = value

    @property
    def l3_yy(self):
        """Message field 'l3_yy'."""
        return self._l3_yy

    @l3_yy.setter
    def l3_yy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l3_yy' field must be of type 'float'"
        self._l3_yy = value

    @property
    def r3_ang(self):
        """Message field 'r3_ang'."""
        return self._r3_ang

    @r3_ang.setter
    def r3_ang(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r3_ang' field must be of type 'float'"
        self._r3_ang = value

    @property
    def r3_dist(self):
        """Message field 'r3_dist'."""
        return self._r3_dist

    @r3_dist.setter
    def r3_dist(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r3_dist' field must be of type 'float'"
        self._r3_dist = value

    @property
    def r3_xx(self):
        """Message field 'r3_xx'."""
        return self._r3_xx

    @r3_xx.setter
    def r3_xx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r3_xx' field must be of type 'float'"
        self._r3_xx = value

    @property
    def r3_yy(self):
        """Message field 'r3_yy'."""
        return self._r3_yy

    @r3_yy.setter
    def r3_yy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r3_yy' field must be of type 'float'"
        self._r3_yy = value

    @property
    def l2(self):
        """Message field 'l2'."""
        return self._l2

    @l2.setter
    def l2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l2' field must be of type 'float'"
        self._l2 = value

    @property
    def r2(self):
        """Message field 'r2'."""
        return self._r2

    @r2.setter
    def r2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r2' field must be of type 'float'"
        self._r2 = value
