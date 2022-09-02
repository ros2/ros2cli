# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ros2topic.verb.pub import set_message_fields_expanded

import builtins
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
import rosidl_parser.definition


class MockMessageStamped:

    __slots__ = [
        '_header',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),
    )

    def __init__(self):
        self.header = Header()

    @builtins.property
    def header(self):
        return self._header

    @header.setter
    def header(self, value):
        self._header = value


class MockMessageWithStampFields:

    __slots__ = [
        '_timestamp1',
        '_timestamp2',
    ]

    _fields_and_field_types = {
        'timestamp1': 'builtin_interfaces/Time',
        'timestamp2': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),
    )

    def __init__(self):
        self.timestamp1 = Time()
        self.timestamp2 = Time()

    @builtins.property
    def timestamp1(self):
        return self._timestamp1

    @timestamp1.setter
    def timestamp1(self, value):
        self._timestamp1 = value

    @builtins.property
    def timestamp2(self):
        return self._timestamp2

    @timestamp2.setter
    def timestamp2(self, value):
        self._timestamp2 = value


def test_set_message_fields_expanded_header_auto():
    msg = MockMessageStamped()
    values = {'header': 'auto'}
    assert msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0
    assert msg.header.frame_id == ''
    timestamp_fields = set_message_fields_expanded(
        msg, values, expand_header_auto=True, expand_time_now=True)
    assert timestamp_fields is not None
    for field_setter in timestamp_fields:
        stamp = Time(sec=1, nanosec=2)
        field_setter(stamp)
    assert msg.header.stamp.sec == 1 and msg.header.stamp.nanosec == 2
    assert msg.header.frame_id == ''


def test_set_message_fields_expanded_stamp_now():
    msg = MockMessageStamped()
    values = {'header': {'stamp': 'now'}}
    assert msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0
    assert msg.header.frame_id == ''
    timestamp_fields = set_message_fields_expanded(
        msg, values, expand_header_auto=True, expand_time_now=True)
    assert timestamp_fields is not None
    for field_setter in timestamp_fields:
        stamp = Time(sec=1, nanosec=2)
        field_setter(stamp)
    assert msg.header.stamp.sec == 1 and msg.header.stamp.nanosec == 2
    assert msg.header.frame_id == ''


def test_set_message_fields_expanded_stamp_now_with_frame_id():
    msg = MockMessageStamped()
    values = {'header': {'stamp': 'now', 'frame_id': 'hello'}}
    assert msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0
    assert msg.header.frame_id == ''
    timestamp_fields = set_message_fields_expanded(
        msg, values, expand_header_auto=True, expand_time_now=True)
    assert timestamp_fields is not None
    for field_setter in timestamp_fields:
        stamp = Time(sec=1, nanosec=2)
        field_setter(stamp)
    assert msg.header.stamp.sec == 1 and msg.header.stamp.nanosec == 2
    assert msg.header.frame_id == 'hello'


def test_set_message_fields_expanded_stamp_now_with_timestamp_fields():
    msg = MockMessageWithStampFields()
    values = {'timestamp1': 'now', 'timestamp2': 'now'}
    assert msg.timestamp1.sec == 0 and msg.timestamp1.nanosec == 0
    assert msg.timestamp2.sec == 0 and msg.timestamp2.nanosec == 0
    timestamp_fields = set_message_fields_expanded(
        msg, values, expand_header_auto=True, expand_time_now=True)
    assert timestamp_fields is not None
    for field_setter in timestamp_fields:
        stamp = Time(sec=1, nanosec=2)
        field_setter(stamp)
    assert msg.timestamp1.sec == 1 and msg.timestamp1.nanosec == 2
    assert msg.timestamp2.sec == 1 and msg.timestamp2.nanosec == 2
