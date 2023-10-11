# generated from rosidl_generator_py/resource/_idl.py.em
# with input from charmie_interfaces:msg/DetectedPerson.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DetectedPerson(type):
    """Metaclass of message 'DetectedPerson'."""

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
                'charmie_interfaces.msg.DetectedPerson')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__detected_person
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__detected_person
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__detected_person
            cls._TYPE_SUPPORT = module.type_support_msg__msg__detected_person
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__detected_person

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DetectedPerson(metaclass=Metaclass_DetectedPerson):
    """Message class 'DetectedPerson'."""

    __slots__ = [
        '_index_person',
        '_conf_person',
        '_x_rel',
        '_y_rel',
        '_box_top_left_x',
        '_box_top_left_y',
        '_box_width',
        '_box_height',
        '_kp_nose_x',
        '_kp_nose_y',
        '_kp_nose_conf',
        '_kp_eye_left_x',
        '_kp_eye_left_y',
        '_kp_eye_left_conf',
        '_kp_eye_right_x',
        '_kp_eye_right_y',
        '_kp_eye_right_conf',
        '_kp_ear_left_x',
        '_kp_ear_left_y',
        '_kp_ear_left_conf',
        '_kp_ear_right_x',
        '_kp_ear_right_y',
        '_kp_ear_right_conf',
        '_kp_shoulder_left_x',
        '_kp_shoulder_left_y',
        '_kp_shoulder_left_conf',
        '_kp_shoulder_right_x',
        '_kp_shoulder_right_y',
        '_kp_shoulder_right_conf',
        '_kp_elbow_left_x',
        '_kp_elbow_left_y',
        '_kp_elbow_left_conf',
        '_kp_elbow_right_x',
        '_kp_elbow_right_y',
        '_kp_elbow_right_conf',
        '_kp_wrist_left_x',
        '_kp_wrist_left_y',
        '_kp_wrist_left_conf',
        '_kp_wrist_right_x',
        '_kp_wrist_right_y',
        '_kp_wrist_right_conf',
        '_kp_hip_left_x',
        '_kp_hip_left_y',
        '_kp_hip_left_conf',
        '_kp_hip_right_x',
        '_kp_hip_right_y',
        '_kp_hip_right_conf',
        '_kp_knee_left_x',
        '_kp_knee_left_y',
        '_kp_knee_left_conf',
        '_kp_knee_right_x',
        '_kp_knee_right_y',
        '_kp_knee_right_conf',
        '_kp_ankle_left_x',
        '_kp_ankle_left_y',
        '_kp_ankle_left_conf',
        '_kp_ankle_right_x',
        '_kp_ankle_right_y',
        '_kp_ankle_right_conf',
    ]

    _fields_and_field_types = {
        'index_person': 'int32',
        'conf_person': 'float',
        'x_rel': 'float',
        'y_rel': 'float',
        'box_top_left_x': 'int32',
        'box_top_left_y': 'int32',
        'box_width': 'int32',
        'box_height': 'int32',
        'kp_nose_x': 'int32',
        'kp_nose_y': 'int32',
        'kp_nose_conf': 'float',
        'kp_eye_left_x': 'int32',
        'kp_eye_left_y': 'int32',
        'kp_eye_left_conf': 'float',
        'kp_eye_right_x': 'int32',
        'kp_eye_right_y': 'int32',
        'kp_eye_right_conf': 'float',
        'kp_ear_left_x': 'int32',
        'kp_ear_left_y': 'int32',
        'kp_ear_left_conf': 'float',
        'kp_ear_right_x': 'int32',
        'kp_ear_right_y': 'int32',
        'kp_ear_right_conf': 'float',
        'kp_shoulder_left_x': 'int32',
        'kp_shoulder_left_y': 'int32',
        'kp_shoulder_left_conf': 'float',
        'kp_shoulder_right_x': 'int32',
        'kp_shoulder_right_y': 'int32',
        'kp_shoulder_right_conf': 'float',
        'kp_elbow_left_x': 'int32',
        'kp_elbow_left_y': 'int32',
        'kp_elbow_left_conf': 'float',
        'kp_elbow_right_x': 'int32',
        'kp_elbow_right_y': 'int32',
        'kp_elbow_right_conf': 'float',
        'kp_wrist_left_x': 'int32',
        'kp_wrist_left_y': 'int32',
        'kp_wrist_left_conf': 'float',
        'kp_wrist_right_x': 'int32',
        'kp_wrist_right_y': 'int32',
        'kp_wrist_right_conf': 'float',
        'kp_hip_left_x': 'int32',
        'kp_hip_left_y': 'int32',
        'kp_hip_left_conf': 'float',
        'kp_hip_right_x': 'int32',
        'kp_hip_right_y': 'int32',
        'kp_hip_right_conf': 'float',
        'kp_knee_left_x': 'int32',
        'kp_knee_left_y': 'int32',
        'kp_knee_left_conf': 'float',
        'kp_knee_right_x': 'int32',
        'kp_knee_right_y': 'int32',
        'kp_knee_right_conf': 'float',
        'kp_ankle_left_x': 'int32',
        'kp_ankle_left_y': 'int32',
        'kp_ankle_left_conf': 'float',
        'kp_ankle_right_x': 'int32',
        'kp_ankle_right_y': 'int32',
        'kp_ankle_right_conf': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.index_person = kwargs.get('index_person', int())
        self.conf_person = kwargs.get('conf_person', float())
        self.x_rel = kwargs.get('x_rel', float())
        self.y_rel = kwargs.get('y_rel', float())
        self.box_top_left_x = kwargs.get('box_top_left_x', int())
        self.box_top_left_y = kwargs.get('box_top_left_y', int())
        self.box_width = kwargs.get('box_width', int())
        self.box_height = kwargs.get('box_height', int())
        self.kp_nose_x = kwargs.get('kp_nose_x', int())
        self.kp_nose_y = kwargs.get('kp_nose_y', int())
        self.kp_nose_conf = kwargs.get('kp_nose_conf', float())
        self.kp_eye_left_x = kwargs.get('kp_eye_left_x', int())
        self.kp_eye_left_y = kwargs.get('kp_eye_left_y', int())
        self.kp_eye_left_conf = kwargs.get('kp_eye_left_conf', float())
        self.kp_eye_right_x = kwargs.get('kp_eye_right_x', int())
        self.kp_eye_right_y = kwargs.get('kp_eye_right_y', int())
        self.kp_eye_right_conf = kwargs.get('kp_eye_right_conf', float())
        self.kp_ear_left_x = kwargs.get('kp_ear_left_x', int())
        self.kp_ear_left_y = kwargs.get('kp_ear_left_y', int())
        self.kp_ear_left_conf = kwargs.get('kp_ear_left_conf', float())
        self.kp_ear_right_x = kwargs.get('kp_ear_right_x', int())
        self.kp_ear_right_y = kwargs.get('kp_ear_right_y', int())
        self.kp_ear_right_conf = kwargs.get('kp_ear_right_conf', float())
        self.kp_shoulder_left_x = kwargs.get('kp_shoulder_left_x', int())
        self.kp_shoulder_left_y = kwargs.get('kp_shoulder_left_y', int())
        self.kp_shoulder_left_conf = kwargs.get('kp_shoulder_left_conf', float())
        self.kp_shoulder_right_x = kwargs.get('kp_shoulder_right_x', int())
        self.kp_shoulder_right_y = kwargs.get('kp_shoulder_right_y', int())
        self.kp_shoulder_right_conf = kwargs.get('kp_shoulder_right_conf', float())
        self.kp_elbow_left_x = kwargs.get('kp_elbow_left_x', int())
        self.kp_elbow_left_y = kwargs.get('kp_elbow_left_y', int())
        self.kp_elbow_left_conf = kwargs.get('kp_elbow_left_conf', float())
        self.kp_elbow_right_x = kwargs.get('kp_elbow_right_x', int())
        self.kp_elbow_right_y = kwargs.get('kp_elbow_right_y', int())
        self.kp_elbow_right_conf = kwargs.get('kp_elbow_right_conf', float())
        self.kp_wrist_left_x = kwargs.get('kp_wrist_left_x', int())
        self.kp_wrist_left_y = kwargs.get('kp_wrist_left_y', int())
        self.kp_wrist_left_conf = kwargs.get('kp_wrist_left_conf', float())
        self.kp_wrist_right_x = kwargs.get('kp_wrist_right_x', int())
        self.kp_wrist_right_y = kwargs.get('kp_wrist_right_y', int())
        self.kp_wrist_right_conf = kwargs.get('kp_wrist_right_conf', float())
        self.kp_hip_left_x = kwargs.get('kp_hip_left_x', int())
        self.kp_hip_left_y = kwargs.get('kp_hip_left_y', int())
        self.kp_hip_left_conf = kwargs.get('kp_hip_left_conf', float())
        self.kp_hip_right_x = kwargs.get('kp_hip_right_x', int())
        self.kp_hip_right_y = kwargs.get('kp_hip_right_y', int())
        self.kp_hip_right_conf = kwargs.get('kp_hip_right_conf', float())
        self.kp_knee_left_x = kwargs.get('kp_knee_left_x', int())
        self.kp_knee_left_y = kwargs.get('kp_knee_left_y', int())
        self.kp_knee_left_conf = kwargs.get('kp_knee_left_conf', float())
        self.kp_knee_right_x = kwargs.get('kp_knee_right_x', int())
        self.kp_knee_right_y = kwargs.get('kp_knee_right_y', int())
        self.kp_knee_right_conf = kwargs.get('kp_knee_right_conf', float())
        self.kp_ankle_left_x = kwargs.get('kp_ankle_left_x', int())
        self.kp_ankle_left_y = kwargs.get('kp_ankle_left_y', int())
        self.kp_ankle_left_conf = kwargs.get('kp_ankle_left_conf', float())
        self.kp_ankle_right_x = kwargs.get('kp_ankle_right_x', int())
        self.kp_ankle_right_y = kwargs.get('kp_ankle_right_y', int())
        self.kp_ankle_right_conf = kwargs.get('kp_ankle_right_conf', float())

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
        if self.conf_person != other.conf_person:
            return False
        if self.x_rel != other.x_rel:
            return False
        if self.y_rel != other.y_rel:
            return False
        if self.box_top_left_x != other.box_top_left_x:
            return False
        if self.box_top_left_y != other.box_top_left_y:
            return False
        if self.box_width != other.box_width:
            return False
        if self.box_height != other.box_height:
            return False
        if self.kp_nose_x != other.kp_nose_x:
            return False
        if self.kp_nose_y != other.kp_nose_y:
            return False
        if self.kp_nose_conf != other.kp_nose_conf:
            return False
        if self.kp_eye_left_x != other.kp_eye_left_x:
            return False
        if self.kp_eye_left_y != other.kp_eye_left_y:
            return False
        if self.kp_eye_left_conf != other.kp_eye_left_conf:
            return False
        if self.kp_eye_right_x != other.kp_eye_right_x:
            return False
        if self.kp_eye_right_y != other.kp_eye_right_y:
            return False
        if self.kp_eye_right_conf != other.kp_eye_right_conf:
            return False
        if self.kp_ear_left_x != other.kp_ear_left_x:
            return False
        if self.kp_ear_left_y != other.kp_ear_left_y:
            return False
        if self.kp_ear_left_conf != other.kp_ear_left_conf:
            return False
        if self.kp_ear_right_x != other.kp_ear_right_x:
            return False
        if self.kp_ear_right_y != other.kp_ear_right_y:
            return False
        if self.kp_ear_right_conf != other.kp_ear_right_conf:
            return False
        if self.kp_shoulder_left_x != other.kp_shoulder_left_x:
            return False
        if self.kp_shoulder_left_y != other.kp_shoulder_left_y:
            return False
        if self.kp_shoulder_left_conf != other.kp_shoulder_left_conf:
            return False
        if self.kp_shoulder_right_x != other.kp_shoulder_right_x:
            return False
        if self.kp_shoulder_right_y != other.kp_shoulder_right_y:
            return False
        if self.kp_shoulder_right_conf != other.kp_shoulder_right_conf:
            return False
        if self.kp_elbow_left_x != other.kp_elbow_left_x:
            return False
        if self.kp_elbow_left_y != other.kp_elbow_left_y:
            return False
        if self.kp_elbow_left_conf != other.kp_elbow_left_conf:
            return False
        if self.kp_elbow_right_x != other.kp_elbow_right_x:
            return False
        if self.kp_elbow_right_y != other.kp_elbow_right_y:
            return False
        if self.kp_elbow_right_conf != other.kp_elbow_right_conf:
            return False
        if self.kp_wrist_left_x != other.kp_wrist_left_x:
            return False
        if self.kp_wrist_left_y != other.kp_wrist_left_y:
            return False
        if self.kp_wrist_left_conf != other.kp_wrist_left_conf:
            return False
        if self.kp_wrist_right_x != other.kp_wrist_right_x:
            return False
        if self.kp_wrist_right_y != other.kp_wrist_right_y:
            return False
        if self.kp_wrist_right_conf != other.kp_wrist_right_conf:
            return False
        if self.kp_hip_left_x != other.kp_hip_left_x:
            return False
        if self.kp_hip_left_y != other.kp_hip_left_y:
            return False
        if self.kp_hip_left_conf != other.kp_hip_left_conf:
            return False
        if self.kp_hip_right_x != other.kp_hip_right_x:
            return False
        if self.kp_hip_right_y != other.kp_hip_right_y:
            return False
        if self.kp_hip_right_conf != other.kp_hip_right_conf:
            return False
        if self.kp_knee_left_x != other.kp_knee_left_x:
            return False
        if self.kp_knee_left_y != other.kp_knee_left_y:
            return False
        if self.kp_knee_left_conf != other.kp_knee_left_conf:
            return False
        if self.kp_knee_right_x != other.kp_knee_right_x:
            return False
        if self.kp_knee_right_y != other.kp_knee_right_y:
            return False
        if self.kp_knee_right_conf != other.kp_knee_right_conf:
            return False
        if self.kp_ankle_left_x != other.kp_ankle_left_x:
            return False
        if self.kp_ankle_left_y != other.kp_ankle_left_y:
            return False
        if self.kp_ankle_left_conf != other.kp_ankle_left_conf:
            return False
        if self.kp_ankle_right_x != other.kp_ankle_right_x:
            return False
        if self.kp_ankle_right_y != other.kp_ankle_right_y:
            return False
        if self.kp_ankle_right_conf != other.kp_ankle_right_conf:
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
    def conf_person(self):
        """Message field 'conf_person'."""
        return self._conf_person

    @conf_person.setter
    def conf_person(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'conf_person' field must be of type 'float'"
        self._conf_person = value

    @property
    def x_rel(self):
        """Message field 'x_rel'."""
        return self._x_rel

    @x_rel.setter
    def x_rel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_rel' field must be of type 'float'"
        self._x_rel = value

    @property
    def y_rel(self):
        """Message field 'y_rel'."""
        return self._y_rel

    @y_rel.setter
    def y_rel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_rel' field must be of type 'float'"
        self._y_rel = value

    @property
    def box_top_left_x(self):
        """Message field 'box_top_left_x'."""
        return self._box_top_left_x

    @box_top_left_x.setter
    def box_top_left_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'box_top_left_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'box_top_left_x' field must be an integer in [-2147483648, 2147483647]"
        self._box_top_left_x = value

    @property
    def box_top_left_y(self):
        """Message field 'box_top_left_y'."""
        return self._box_top_left_y

    @box_top_left_y.setter
    def box_top_left_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'box_top_left_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'box_top_left_y' field must be an integer in [-2147483648, 2147483647]"
        self._box_top_left_y = value

    @property
    def box_width(self):
        """Message field 'box_width'."""
        return self._box_width

    @box_width.setter
    def box_width(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'box_width' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'box_width' field must be an integer in [-2147483648, 2147483647]"
        self._box_width = value

    @property
    def box_height(self):
        """Message field 'box_height'."""
        return self._box_height

    @box_height.setter
    def box_height(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'box_height' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'box_height' field must be an integer in [-2147483648, 2147483647]"
        self._box_height = value

    @property
    def kp_nose_x(self):
        """Message field 'kp_nose_x'."""
        return self._kp_nose_x

    @kp_nose_x.setter
    def kp_nose_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_nose_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_nose_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_nose_x = value

    @property
    def kp_nose_y(self):
        """Message field 'kp_nose_y'."""
        return self._kp_nose_y

    @kp_nose_y.setter
    def kp_nose_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_nose_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_nose_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_nose_y = value

    @property
    def kp_nose_conf(self):
        """Message field 'kp_nose_conf'."""
        return self._kp_nose_conf

    @kp_nose_conf.setter
    def kp_nose_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_nose_conf' field must be of type 'float'"
        self._kp_nose_conf = value

    @property
    def kp_eye_left_x(self):
        """Message field 'kp_eye_left_x'."""
        return self._kp_eye_left_x

    @kp_eye_left_x.setter
    def kp_eye_left_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_eye_left_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_eye_left_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_eye_left_x = value

    @property
    def kp_eye_left_y(self):
        """Message field 'kp_eye_left_y'."""
        return self._kp_eye_left_y

    @kp_eye_left_y.setter
    def kp_eye_left_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_eye_left_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_eye_left_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_eye_left_y = value

    @property
    def kp_eye_left_conf(self):
        """Message field 'kp_eye_left_conf'."""
        return self._kp_eye_left_conf

    @kp_eye_left_conf.setter
    def kp_eye_left_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_eye_left_conf' field must be of type 'float'"
        self._kp_eye_left_conf = value

    @property
    def kp_eye_right_x(self):
        """Message field 'kp_eye_right_x'."""
        return self._kp_eye_right_x

    @kp_eye_right_x.setter
    def kp_eye_right_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_eye_right_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_eye_right_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_eye_right_x = value

    @property
    def kp_eye_right_y(self):
        """Message field 'kp_eye_right_y'."""
        return self._kp_eye_right_y

    @kp_eye_right_y.setter
    def kp_eye_right_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_eye_right_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_eye_right_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_eye_right_y = value

    @property
    def kp_eye_right_conf(self):
        """Message field 'kp_eye_right_conf'."""
        return self._kp_eye_right_conf

    @kp_eye_right_conf.setter
    def kp_eye_right_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_eye_right_conf' field must be of type 'float'"
        self._kp_eye_right_conf = value

    @property
    def kp_ear_left_x(self):
        """Message field 'kp_ear_left_x'."""
        return self._kp_ear_left_x

    @kp_ear_left_x.setter
    def kp_ear_left_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_ear_left_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_ear_left_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_ear_left_x = value

    @property
    def kp_ear_left_y(self):
        """Message field 'kp_ear_left_y'."""
        return self._kp_ear_left_y

    @kp_ear_left_y.setter
    def kp_ear_left_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_ear_left_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_ear_left_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_ear_left_y = value

    @property
    def kp_ear_left_conf(self):
        """Message field 'kp_ear_left_conf'."""
        return self._kp_ear_left_conf

    @kp_ear_left_conf.setter
    def kp_ear_left_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_ear_left_conf' field must be of type 'float'"
        self._kp_ear_left_conf = value

    @property
    def kp_ear_right_x(self):
        """Message field 'kp_ear_right_x'."""
        return self._kp_ear_right_x

    @kp_ear_right_x.setter
    def kp_ear_right_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_ear_right_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_ear_right_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_ear_right_x = value

    @property
    def kp_ear_right_y(self):
        """Message field 'kp_ear_right_y'."""
        return self._kp_ear_right_y

    @kp_ear_right_y.setter
    def kp_ear_right_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_ear_right_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_ear_right_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_ear_right_y = value

    @property
    def kp_ear_right_conf(self):
        """Message field 'kp_ear_right_conf'."""
        return self._kp_ear_right_conf

    @kp_ear_right_conf.setter
    def kp_ear_right_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_ear_right_conf' field must be of type 'float'"
        self._kp_ear_right_conf = value

    @property
    def kp_shoulder_left_x(self):
        """Message field 'kp_shoulder_left_x'."""
        return self._kp_shoulder_left_x

    @kp_shoulder_left_x.setter
    def kp_shoulder_left_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_shoulder_left_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_shoulder_left_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_shoulder_left_x = value

    @property
    def kp_shoulder_left_y(self):
        """Message field 'kp_shoulder_left_y'."""
        return self._kp_shoulder_left_y

    @kp_shoulder_left_y.setter
    def kp_shoulder_left_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_shoulder_left_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_shoulder_left_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_shoulder_left_y = value

    @property
    def kp_shoulder_left_conf(self):
        """Message field 'kp_shoulder_left_conf'."""
        return self._kp_shoulder_left_conf

    @kp_shoulder_left_conf.setter
    def kp_shoulder_left_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_shoulder_left_conf' field must be of type 'float'"
        self._kp_shoulder_left_conf = value

    @property
    def kp_shoulder_right_x(self):
        """Message field 'kp_shoulder_right_x'."""
        return self._kp_shoulder_right_x

    @kp_shoulder_right_x.setter
    def kp_shoulder_right_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_shoulder_right_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_shoulder_right_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_shoulder_right_x = value

    @property
    def kp_shoulder_right_y(self):
        """Message field 'kp_shoulder_right_y'."""
        return self._kp_shoulder_right_y

    @kp_shoulder_right_y.setter
    def kp_shoulder_right_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_shoulder_right_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_shoulder_right_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_shoulder_right_y = value

    @property
    def kp_shoulder_right_conf(self):
        """Message field 'kp_shoulder_right_conf'."""
        return self._kp_shoulder_right_conf

    @kp_shoulder_right_conf.setter
    def kp_shoulder_right_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_shoulder_right_conf' field must be of type 'float'"
        self._kp_shoulder_right_conf = value

    @property
    def kp_elbow_left_x(self):
        """Message field 'kp_elbow_left_x'."""
        return self._kp_elbow_left_x

    @kp_elbow_left_x.setter
    def kp_elbow_left_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_elbow_left_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_elbow_left_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_elbow_left_x = value

    @property
    def kp_elbow_left_y(self):
        """Message field 'kp_elbow_left_y'."""
        return self._kp_elbow_left_y

    @kp_elbow_left_y.setter
    def kp_elbow_left_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_elbow_left_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_elbow_left_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_elbow_left_y = value

    @property
    def kp_elbow_left_conf(self):
        """Message field 'kp_elbow_left_conf'."""
        return self._kp_elbow_left_conf

    @kp_elbow_left_conf.setter
    def kp_elbow_left_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_elbow_left_conf' field must be of type 'float'"
        self._kp_elbow_left_conf = value

    @property
    def kp_elbow_right_x(self):
        """Message field 'kp_elbow_right_x'."""
        return self._kp_elbow_right_x

    @kp_elbow_right_x.setter
    def kp_elbow_right_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_elbow_right_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_elbow_right_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_elbow_right_x = value

    @property
    def kp_elbow_right_y(self):
        """Message field 'kp_elbow_right_y'."""
        return self._kp_elbow_right_y

    @kp_elbow_right_y.setter
    def kp_elbow_right_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_elbow_right_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_elbow_right_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_elbow_right_y = value

    @property
    def kp_elbow_right_conf(self):
        """Message field 'kp_elbow_right_conf'."""
        return self._kp_elbow_right_conf

    @kp_elbow_right_conf.setter
    def kp_elbow_right_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_elbow_right_conf' field must be of type 'float'"
        self._kp_elbow_right_conf = value

    @property
    def kp_wrist_left_x(self):
        """Message field 'kp_wrist_left_x'."""
        return self._kp_wrist_left_x

    @kp_wrist_left_x.setter
    def kp_wrist_left_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_wrist_left_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_wrist_left_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_wrist_left_x = value

    @property
    def kp_wrist_left_y(self):
        """Message field 'kp_wrist_left_y'."""
        return self._kp_wrist_left_y

    @kp_wrist_left_y.setter
    def kp_wrist_left_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_wrist_left_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_wrist_left_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_wrist_left_y = value

    @property
    def kp_wrist_left_conf(self):
        """Message field 'kp_wrist_left_conf'."""
        return self._kp_wrist_left_conf

    @kp_wrist_left_conf.setter
    def kp_wrist_left_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_wrist_left_conf' field must be of type 'float'"
        self._kp_wrist_left_conf = value

    @property
    def kp_wrist_right_x(self):
        """Message field 'kp_wrist_right_x'."""
        return self._kp_wrist_right_x

    @kp_wrist_right_x.setter
    def kp_wrist_right_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_wrist_right_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_wrist_right_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_wrist_right_x = value

    @property
    def kp_wrist_right_y(self):
        """Message field 'kp_wrist_right_y'."""
        return self._kp_wrist_right_y

    @kp_wrist_right_y.setter
    def kp_wrist_right_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_wrist_right_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_wrist_right_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_wrist_right_y = value

    @property
    def kp_wrist_right_conf(self):
        """Message field 'kp_wrist_right_conf'."""
        return self._kp_wrist_right_conf

    @kp_wrist_right_conf.setter
    def kp_wrist_right_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_wrist_right_conf' field must be of type 'float'"
        self._kp_wrist_right_conf = value

    @property
    def kp_hip_left_x(self):
        """Message field 'kp_hip_left_x'."""
        return self._kp_hip_left_x

    @kp_hip_left_x.setter
    def kp_hip_left_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_hip_left_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_hip_left_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_hip_left_x = value

    @property
    def kp_hip_left_y(self):
        """Message field 'kp_hip_left_y'."""
        return self._kp_hip_left_y

    @kp_hip_left_y.setter
    def kp_hip_left_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_hip_left_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_hip_left_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_hip_left_y = value

    @property
    def kp_hip_left_conf(self):
        """Message field 'kp_hip_left_conf'."""
        return self._kp_hip_left_conf

    @kp_hip_left_conf.setter
    def kp_hip_left_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_hip_left_conf' field must be of type 'float'"
        self._kp_hip_left_conf = value

    @property
    def kp_hip_right_x(self):
        """Message field 'kp_hip_right_x'."""
        return self._kp_hip_right_x

    @kp_hip_right_x.setter
    def kp_hip_right_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_hip_right_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_hip_right_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_hip_right_x = value

    @property
    def kp_hip_right_y(self):
        """Message field 'kp_hip_right_y'."""
        return self._kp_hip_right_y

    @kp_hip_right_y.setter
    def kp_hip_right_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_hip_right_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_hip_right_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_hip_right_y = value

    @property
    def kp_hip_right_conf(self):
        """Message field 'kp_hip_right_conf'."""
        return self._kp_hip_right_conf

    @kp_hip_right_conf.setter
    def kp_hip_right_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_hip_right_conf' field must be of type 'float'"
        self._kp_hip_right_conf = value

    @property
    def kp_knee_left_x(self):
        """Message field 'kp_knee_left_x'."""
        return self._kp_knee_left_x

    @kp_knee_left_x.setter
    def kp_knee_left_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_knee_left_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_knee_left_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_knee_left_x = value

    @property
    def kp_knee_left_y(self):
        """Message field 'kp_knee_left_y'."""
        return self._kp_knee_left_y

    @kp_knee_left_y.setter
    def kp_knee_left_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_knee_left_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_knee_left_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_knee_left_y = value

    @property
    def kp_knee_left_conf(self):
        """Message field 'kp_knee_left_conf'."""
        return self._kp_knee_left_conf

    @kp_knee_left_conf.setter
    def kp_knee_left_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_knee_left_conf' field must be of type 'float'"
        self._kp_knee_left_conf = value

    @property
    def kp_knee_right_x(self):
        """Message field 'kp_knee_right_x'."""
        return self._kp_knee_right_x

    @kp_knee_right_x.setter
    def kp_knee_right_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_knee_right_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_knee_right_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_knee_right_x = value

    @property
    def kp_knee_right_y(self):
        """Message field 'kp_knee_right_y'."""
        return self._kp_knee_right_y

    @kp_knee_right_y.setter
    def kp_knee_right_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_knee_right_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_knee_right_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_knee_right_y = value

    @property
    def kp_knee_right_conf(self):
        """Message field 'kp_knee_right_conf'."""
        return self._kp_knee_right_conf

    @kp_knee_right_conf.setter
    def kp_knee_right_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_knee_right_conf' field must be of type 'float'"
        self._kp_knee_right_conf = value

    @property
    def kp_ankle_left_x(self):
        """Message field 'kp_ankle_left_x'."""
        return self._kp_ankle_left_x

    @kp_ankle_left_x.setter
    def kp_ankle_left_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_ankle_left_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_ankle_left_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_ankle_left_x = value

    @property
    def kp_ankle_left_y(self):
        """Message field 'kp_ankle_left_y'."""
        return self._kp_ankle_left_y

    @kp_ankle_left_y.setter
    def kp_ankle_left_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_ankle_left_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_ankle_left_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_ankle_left_y = value

    @property
    def kp_ankle_left_conf(self):
        """Message field 'kp_ankle_left_conf'."""
        return self._kp_ankle_left_conf

    @kp_ankle_left_conf.setter
    def kp_ankle_left_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_ankle_left_conf' field must be of type 'float'"
        self._kp_ankle_left_conf = value

    @property
    def kp_ankle_right_x(self):
        """Message field 'kp_ankle_right_x'."""
        return self._kp_ankle_right_x

    @kp_ankle_right_x.setter
    def kp_ankle_right_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_ankle_right_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_ankle_right_x' field must be an integer in [-2147483648, 2147483647]"
        self._kp_ankle_right_x = value

    @property
    def kp_ankle_right_y(self):
        """Message field 'kp_ankle_right_y'."""
        return self._kp_ankle_right_y

    @kp_ankle_right_y.setter
    def kp_ankle_right_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kp_ankle_right_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kp_ankle_right_y' field must be an integer in [-2147483648, 2147483647]"
        self._kp_ankle_right_y = value

    @property
    def kp_ankle_right_conf(self):
        """Message field 'kp_ankle_right_conf'."""
        return self._kp_ankle_right_conf

    @kp_ankle_right_conf.setter
    def kp_ankle_right_conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp_ankle_right_conf' field must be of type 'float'"
        self._kp_ankle_right_conf = value
