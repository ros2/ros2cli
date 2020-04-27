# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import functools
from xmlrpc.client import Marshaller
from xmlrpc.client import Unmarshaller

import rclpy.duration
import rclpy.qos
import rclpy.topic_endpoint_info

from .generic import dump_any_enum
from .generic import dump_any_with_slots
from .generic import end_any_enum
from .generic import end_any_with_slots
from .generic import fullname


def end_duration(unmarshaller, data):
    unmarshaller.append(
        rclpy.duration.Duration(nanoseconds=int(data))
    )
    unmarshaller._value = 0


def dump_duration(marshaller, value, write):
    write(f'<value><{fullname(type(value))}>')
    write(str(value.nanoseconds))
    write(f'</{fullname(type(value))}></value>')


Unmarshaller.dispatch[fullname(rclpy.duration.Duration)] = end_duration
Marshaller.dispatch[rclpy.duration.Duration] = dump_duration


Unmarshaller.dispatch[fullname(rclpy.qos.QoSProfile)] = functools.partial(
    end_any_with_slots, type_=rclpy.qos.QoSProfile
)
Marshaller.dispatch[rclpy.qos.QoSProfile] = functools.partial(
    dump_any_with_slots, transform=lambda slot: slot.lstrip('_')
)

policy_types = (
    rclpy.qos.HistoryPolicy,
    rclpy.qos.ReliabilityPolicy,
    rclpy.qos.DurabilityPolicy,
    rclpy.qos.LivelinessPolicy
)

for enum_type in policy_types:
    Unmarshaller.dispatch[fullname(enum_type)] = \
        functools.partial(end_any_enum, enum_=enum_type)

    Marshaller.dispatch[enum_type] = dump_any_enum


Unmarshaller.dispatch[fullname(rclpy.topic_endpoint_info.TopicEndpointInfo)] = \
    functools.partial(end_any_with_slots, type_=rclpy.topic_endpoint_info.TopicEndpointInfo)

Marshaller.dispatch[rclpy.topic_endpoint_info.TopicEndpointInfo] = \
    functools.partial(dump_any_with_slots, transform=lambda slot: slot.lstrip('_'))

Unmarshaller.dispatch[fullname(rclpy.topic_endpoint_info.TopicEndpointTypeEnum)] = \
    functools.partial(end_any_enum, enum_=rclpy.topic_endpoint_info.TopicEndpointTypeEnum)

Marshaller.dispatch[rclpy.topic_endpoint_info.TopicEndpointTypeEnum] = dump_any_enum
