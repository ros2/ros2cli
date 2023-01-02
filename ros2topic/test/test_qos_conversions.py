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

import rclpy
from rclpy.duration import Duration

from ros2topic.api import qos_profile_from_short_keys


def test_profile_conversion():
    profile = qos_profile_from_short_keys(
        'sensor_data', reliability='reliable', durability='transient_local',
        depth=10, history='keep_last', liveliness='manual_by_topic',
        liveliness_lease_duration_s=10.3)
    assert profile.durability == rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
    assert profile.reliability == rclpy.qos.QoSReliabilityPolicy.RELIABLE
    assert profile.depth == 10
    assert profile.history == rclpy.qos.QoSHistoryPolicy.KEEP_LAST
    assert profile.liveliness == rclpy.qos.QoSLivelinessPolicy.MANUAL_BY_TOPIC
    assert profile.liveliness == rclpy.qos.QoSLivelinessPolicy.MANUAL_BY_TOPIC
    assert profile.liveliness_lease_duration == Duration(seconds=10.3)
