# Copyright 2019 Amazon.com, Inc. or its affiliates. All rights reserved.
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

import pytest
import rclpy

from ros2topic.api import duration_ns
from ros2topic.api import qos_profile_from_short_keys

liveliness_options = [
    ('automatic', rclpy.qos.LivelinessPolicy.AUTOMATIC),
    ('system_default', rclpy.qos.LivelinessPolicy.SYSTEM_DEFAULT),
    ('manual_by_topic', rclpy.qos.LivelinessPolicy.MANUAL_BY_TOPIC),
    ('unknown', rclpy.qos.LivelinessPolicy.UNKNOWN)
]


@pytest.mark.parametrize('liveliness,expected_liveliness', liveliness_options)
def test_profile_conversion(liveliness, expected_liveliness):
    profile = qos_profile_from_short_keys(
        'sensor_data', reliability='reliable', durability='transient_local',
        depth=10, history='keep_last', liveliness=liveliness,
        liveliness_lease_duration=duration_ns(1e9))
    assert profile.durability == rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
    assert profile.reliability == rclpy.qos.QoSReliabilityPolicy.RELIABLE
    assert profile.depth == 10
    assert profile.history == rclpy.qos.QoSHistoryPolicy.KEEP_LAST
    assert profile.liveliness == expected_liveliness
    assert profile.liveliness_lease_duration == rclpy.duration.Duration(nanoseconds=1e9)
