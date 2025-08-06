# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2cli_test_interfaces:action/ShortVariedMultiNested.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ShortVariedMultiNested_Goal(type):
    """Metaclass of message 'ShortVariedMultiNested_Goal'."""

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
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__short_varied_multi_nested__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__short_varied_multi_nested__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__short_varied_multi_nested__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__short_varied_multi_nested__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__short_varied_multi_nested__goal

            from ros2cli_test_interfaces.msg import ShortVariedNested
            if ShortVariedNested.__class__._TYPE_SUPPORT is None:
                ShortVariedNested.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ShortVariedMultiNested_Goal(metaclass=Metaclass_ShortVariedMultiNested_Goal):
    """Message class 'ShortVariedMultiNested_Goal'."""

    __slots__ = [
        '_short_varied_nested',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'short_varied_nested': 'ros2cli_test_interfaces/ShortVariedNested',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['ros2cli_test_interfaces', 'msg'], 'ShortVariedNested'),  # noqa: E501
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
        from ros2cli_test_interfaces.msg import ShortVariedNested
        self.short_varied_nested = kwargs.get('short_varied_nested', ShortVariedNested())

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
        if self.short_varied_nested != other.short_varied_nested:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def short_varied_nested(self):
        """Message field 'short_varied_nested'."""
        return self._short_varied_nested

    @short_varied_nested.setter
    def short_varied_nested(self, value):
        if self._check_fields:
            from ros2cli_test_interfaces.msg import ShortVariedNested
            assert \
                isinstance(value, ShortVariedNested), \
                "The 'short_varied_nested' field must be a sub message of type 'ShortVariedNested'"
        self._short_varied_nested = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ShortVariedMultiNested_Result(type):
    """Metaclass of message 'ShortVariedMultiNested_Result'."""

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
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__short_varied_multi_nested__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__short_varied_multi_nested__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__short_varied_multi_nested__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__short_varied_multi_nested__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__short_varied_multi_nested__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ShortVariedMultiNested_Result(metaclass=Metaclass_ShortVariedMultiNested_Result):
    """Message class 'ShortVariedMultiNested_Result'."""

    __slots__ = [
        '_bool_value',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'bool_value': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
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
        self.bool_value = kwargs.get('bool_value', bool())

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
        if self.bool_value != other.bool_value:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def bool_value(self):
        """Message field 'bool_value'."""
        return self._bool_value

    @bool_value.setter
    def bool_value(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'bool_value' field must be of type 'bool'"
        self._bool_value = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ShortVariedMultiNested_Feedback(type):
    """Metaclass of message 'ShortVariedMultiNested_Feedback'."""

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
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__short_varied_multi_nested__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__short_varied_multi_nested__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__short_varied_multi_nested__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__short_varied_multi_nested__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__short_varied_multi_nested__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ShortVariedMultiNested_Feedback(metaclass=Metaclass_ShortVariedMultiNested_Feedback):
    """Message class 'ShortVariedMultiNested_Feedback'."""

    __slots__ = [
        '_bool_values',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'bool_values': 'boolean[3]',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('boolean'), 3),  # noqa: E501
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
        self.bool_values = kwargs.get(
            'bool_values',
            [bool() for x in range(3)]
        )

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
        if self.bool_values != other.bool_values:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def bool_values(self):
        """Message field 'bool_values'."""
        return self._bool_values

    @bool_values.setter
    def bool_values(self, value):
        if self._check_fields:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'bool_values' field must be a set or sequence with length 3 and each value of type 'bool'"
        self._bool_values = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ShortVariedMultiNested_SendGoal_Request(type):
    """Metaclass of message 'ShortVariedMultiNested_SendGoal_Request'."""

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
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__short_varied_multi_nested__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__short_varied_multi_nested__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__short_varied_multi_nested__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__short_varied_multi_nested__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__short_varied_multi_nested__send_goal__request

            from ros2cli_test_interfaces.action import ShortVariedMultiNested
            if ShortVariedMultiNested.Goal.__class__._TYPE_SUPPORT is None:
                ShortVariedMultiNested.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ShortVariedMultiNested_SendGoal_Request(metaclass=Metaclass_ShortVariedMultiNested_SendGoal_Request):
    """Message class 'ShortVariedMultiNested_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'ros2cli_test_interfaces/ShortVariedMultiNested_Goal',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ros2cli_test_interfaces', 'action'], 'ShortVariedMultiNested_Goal'),  # noqa: E501
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
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_Goal
        self.goal = kwargs.get('goal', ShortVariedMultiNested_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if self._check_fields:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if self._check_fields:
            from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_Goal
            assert \
                isinstance(value, ShortVariedMultiNested_Goal), \
                "The 'goal' field must be a sub message of type 'ShortVariedMultiNested_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ShortVariedMultiNested_SendGoal_Response(type):
    """Metaclass of message 'ShortVariedMultiNested_SendGoal_Response'."""

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
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__short_varied_multi_nested__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__short_varied_multi_nested__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__short_varied_multi_nested__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__short_varied_multi_nested__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__short_varied_multi_nested__send_goal__response

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


class ShortVariedMultiNested_SendGoal_Response(metaclass=Metaclass_ShortVariedMultiNested_SendGoal_Response):
    """Message class 'ShortVariedMultiNested_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
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
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if self._check_fields:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ShortVariedMultiNested_SendGoal_Event(type):
    """Metaclass of message 'ShortVariedMultiNested_SendGoal_Event'."""

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
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_SendGoal_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__short_varied_multi_nested__send_goal__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__short_varied_multi_nested__send_goal__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__short_varied_multi_nested__send_goal__event
            cls._TYPE_SUPPORT = module.type_support_msg__action__short_varied_multi_nested__send_goal__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__short_varied_multi_nested__send_goal__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ShortVariedMultiNested_SendGoal_Event(metaclass=Metaclass_ShortVariedMultiNested_SendGoal_Event):
    """Message class 'ShortVariedMultiNested_SendGoal_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<ros2cli_test_interfaces/ShortVariedMultiNested_SendGoal_Request, 1>',
        'response': 'sequence<ros2cli_test_interfaces/ShortVariedMultiNested_SendGoal_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['ros2cli_test_interfaces', 'action'], 'ShortVariedMultiNested_SendGoal_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['ros2cli_test_interfaces', 'action'], 'ShortVariedMultiNested_SendGoal_Response'), 1),  # noqa: E501
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
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

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
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from ros2cli_test_interfaces.action import ShortVariedMultiNested_SendGoal_Request
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, ShortVariedMultiNested_SendGoal_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'ShortVariedMultiNested_SendGoal_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from ros2cli_test_interfaces.action import ShortVariedMultiNested_SendGoal_Response
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, ShortVariedMultiNested_SendGoal_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'ShortVariedMultiNested_SendGoal_Response'"
        self._response = value


class Metaclass_ShortVariedMultiNested_SendGoal(type):
    """Metaclass of service 'ShortVariedMultiNested_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__short_varied_multi_nested__send_goal

            from ros2cli_test_interfaces.action import _short_varied_multi_nested
            if _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_SendGoal_Request._TYPE_SUPPORT is None:
                _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_SendGoal_Request.__import_type_support__()
            if _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_SendGoal_Response._TYPE_SUPPORT is None:
                _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_SendGoal_Response.__import_type_support__()
            if _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_SendGoal_Event._TYPE_SUPPORT is None:
                _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_SendGoal_Event.__import_type_support__()


class ShortVariedMultiNested_SendGoal(metaclass=Metaclass_ShortVariedMultiNested_SendGoal):
    from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_SendGoal_Request as Request
    from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_SendGoal_Response as Response
    from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_SendGoal_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ShortVariedMultiNested_GetResult_Request(type):
    """Metaclass of message 'ShortVariedMultiNested_GetResult_Request'."""

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
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__short_varied_multi_nested__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__short_varied_multi_nested__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__short_varied_multi_nested__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__short_varied_multi_nested__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__short_varied_multi_nested__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ShortVariedMultiNested_GetResult_Request(metaclass=Metaclass_ShortVariedMultiNested_GetResult_Request):
    """Message class 'ShortVariedMultiNested_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
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
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if self._check_fields:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ShortVariedMultiNested_GetResult_Response(type):
    """Metaclass of message 'ShortVariedMultiNested_GetResult_Response'."""

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
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__short_varied_multi_nested__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__short_varied_multi_nested__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__short_varied_multi_nested__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__short_varied_multi_nested__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__short_varied_multi_nested__get_result__response

            from ros2cli_test_interfaces.action import ShortVariedMultiNested
            if ShortVariedMultiNested.Result.__class__._TYPE_SUPPORT is None:
                ShortVariedMultiNested.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ShortVariedMultiNested_GetResult_Response(metaclass=Metaclass_ShortVariedMultiNested_GetResult_Response):
    """Message class 'ShortVariedMultiNested_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'ros2cli_test_interfaces/ShortVariedMultiNested_Result',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ros2cli_test_interfaces', 'action'], 'ShortVariedMultiNested_Result'),  # noqa: E501
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
        self.status = kwargs.get('status', int())
        from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_Result
        self.result = kwargs.get('result', ShortVariedMultiNested_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if self._check_fields:
            from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_Result
            assert \
                isinstance(value, ShortVariedMultiNested_Result), \
                "The 'result' field must be a sub message of type 'ShortVariedMultiNested_Result'"
        self._result = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ShortVariedMultiNested_GetResult_Event(type):
    """Metaclass of message 'ShortVariedMultiNested_GetResult_Event'."""

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
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_GetResult_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__short_varied_multi_nested__get_result__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__short_varied_multi_nested__get_result__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__short_varied_multi_nested__get_result__event
            cls._TYPE_SUPPORT = module.type_support_msg__action__short_varied_multi_nested__get_result__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__short_varied_multi_nested__get_result__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ShortVariedMultiNested_GetResult_Event(metaclass=Metaclass_ShortVariedMultiNested_GetResult_Event):
    """Message class 'ShortVariedMultiNested_GetResult_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<ros2cli_test_interfaces/ShortVariedMultiNested_GetResult_Request, 1>',
        'response': 'sequence<ros2cli_test_interfaces/ShortVariedMultiNested_GetResult_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['ros2cli_test_interfaces', 'action'], 'ShortVariedMultiNested_GetResult_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['ros2cli_test_interfaces', 'action'], 'ShortVariedMultiNested_GetResult_Response'), 1),  # noqa: E501
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
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

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
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from ros2cli_test_interfaces.action import ShortVariedMultiNested_GetResult_Request
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, ShortVariedMultiNested_GetResult_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'ShortVariedMultiNested_GetResult_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from ros2cli_test_interfaces.action import ShortVariedMultiNested_GetResult_Response
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, ShortVariedMultiNested_GetResult_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'ShortVariedMultiNested_GetResult_Response'"
        self._response = value


class Metaclass_ShortVariedMultiNested_GetResult(type):
    """Metaclass of service 'ShortVariedMultiNested_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__short_varied_multi_nested__get_result

            from ros2cli_test_interfaces.action import _short_varied_multi_nested
            if _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_GetResult_Request._TYPE_SUPPORT is None:
                _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_GetResult_Request.__import_type_support__()
            if _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_GetResult_Response._TYPE_SUPPORT is None:
                _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_GetResult_Response.__import_type_support__()
            if _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_GetResult_Event._TYPE_SUPPORT is None:
                _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_GetResult_Event.__import_type_support__()


class ShortVariedMultiNested_GetResult(metaclass=Metaclass_ShortVariedMultiNested_GetResult):
    from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_GetResult_Request as Request
    from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_GetResult_Response as Response
    from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_GetResult_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ShortVariedMultiNested_FeedbackMessage(type):
    """Metaclass of message 'ShortVariedMultiNested_FeedbackMessage'."""

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
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__short_varied_multi_nested__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__short_varied_multi_nested__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__short_varied_multi_nested__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__short_varied_multi_nested__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__short_varied_multi_nested__feedback_message

            from ros2cli_test_interfaces.action import ShortVariedMultiNested
            if ShortVariedMultiNested.Feedback.__class__._TYPE_SUPPORT is None:
                ShortVariedMultiNested.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ShortVariedMultiNested_FeedbackMessage(metaclass=Metaclass_ShortVariedMultiNested_FeedbackMessage):
    """Message class 'ShortVariedMultiNested_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'ros2cli_test_interfaces/ShortVariedMultiNested_Feedback',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ros2cli_test_interfaces', 'action'], 'ShortVariedMultiNested_Feedback'),  # noqa: E501
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
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_Feedback
        self.feedback = kwargs.get('feedback', ShortVariedMultiNested_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if self._check_fields:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if self._check_fields:
            from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_Feedback
            assert \
                isinstance(value, ShortVariedMultiNested_Feedback), \
                "The 'feedback' field must be a sub message of type 'ShortVariedMultiNested_Feedback'"
        self._feedback = value


class Metaclass_ShortVariedMultiNested(type):
    """Metaclass of action 'ShortVariedMultiNested'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ros2cli_test_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2cli_test_interfaces.action.ShortVariedMultiNested')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__short_varied_multi_nested

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from ros2cli_test_interfaces.action import _short_varied_multi_nested
            if _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_SendGoal._TYPE_SUPPORT is None:
                _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_SendGoal.__import_type_support__()
            if _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_GetResult._TYPE_SUPPORT is None:
                _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_GetResult.__import_type_support__()
            if _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_FeedbackMessage._TYPE_SUPPORT is None:
                _short_varied_multi_nested.Metaclass_ShortVariedMultiNested_FeedbackMessage.__import_type_support__()


class ShortVariedMultiNested(metaclass=Metaclass_ShortVariedMultiNested):

    # The goal message defined in the action definition.
    from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_Goal as Goal
    # The result message defined in the action definition.
    from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_Result as Result
    # The feedback message defined in the action definition.
    from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from ros2cli_test_interfaces.action._short_varied_multi_nested import ShortVariedMultiNested_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
