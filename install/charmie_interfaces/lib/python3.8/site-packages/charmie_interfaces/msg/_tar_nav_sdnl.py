# generated from rosidl_generator_py/resource/_idl.py.em
# with input from charmie_interfaces:msg/TarNavSDNL.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TarNavSDNL(type):
    """Metaclass of message 'TarNavSDNL'."""

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
                'charmie_interfaces.msg.TarNavSDNL')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tar_nav_sdnl
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tar_nav_sdnl
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tar_nav_sdnl
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tar_nav_sdnl
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tar_nav_sdnl

            from geometry_msgs.msg import Pose2D
            if Pose2D.__class__._TYPE_SUPPORT is None:
                Pose2D.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TarNavSDNL(metaclass=Metaclass_TarNavSDNL):
    """Message class 'TarNavSDNL'."""

    __slots__ = [
        '_move_target_coordinates',
        '_rotate_target_coordinates',
        '_flag_not_obs',
    ]

    _fields_and_field_types = {
        'move_target_coordinates': 'geometry_msgs/Pose2D',
        'rotate_target_coordinates': 'geometry_msgs/Pose2D',
        'flag_not_obs': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose2D'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose2D'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Pose2D
        self.move_target_coordinates = kwargs.get('move_target_coordinates', Pose2D())
        from geometry_msgs.msg import Pose2D
        self.rotate_target_coordinates = kwargs.get('rotate_target_coordinates', Pose2D())
        self.flag_not_obs = kwargs.get('flag_not_obs', bool())

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
        if self.move_target_coordinates != other.move_target_coordinates:
            return False
        if self.rotate_target_coordinates != other.rotate_target_coordinates:
            return False
        if self.flag_not_obs != other.flag_not_obs:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def move_target_coordinates(self):
        """Message field 'move_target_coordinates'."""
        return self._move_target_coordinates

    @move_target_coordinates.setter
    def move_target_coordinates(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose2D
            assert \
                isinstance(value, Pose2D), \
                "The 'move_target_coordinates' field must be a sub message of type 'Pose2D'"
        self._move_target_coordinates = value

    @property
    def rotate_target_coordinates(self):
        """Message field 'rotate_target_coordinates'."""
        return self._rotate_target_coordinates

    @rotate_target_coordinates.setter
    def rotate_target_coordinates(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose2D
            assert \
                isinstance(value, Pose2D), \
                "The 'rotate_target_coordinates' field must be a sub message of type 'Pose2D'"
        self._rotate_target_coordinates = value

    @property
    def flag_not_obs(self):
        """Message field 'flag_not_obs'."""
        return self._flag_not_obs

    @flag_not_obs.setter
    def flag_not_obs(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'flag_not_obs' field must be of type 'bool'"
        self._flag_not_obs = value
