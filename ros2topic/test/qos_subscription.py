# Copyright 2019 Open Source Robotics Foundation, Inc.
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
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String


class Counter:
    """Simple class for aggregating topic counts."""

    def __init__(self):
        """Create a Counter."""
        self.count = {
            'default': 0,
            'transient': 0,
            'volatile': 0,
            'reliable': 0,
            'best_effort': 0
        }

    def onMsg(self, param, msg):
        """Message subscription callback."""
        self.count[param] = self.count[param] + 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('qos_subscription')

    counter = Counter()

    sub_default = node.create_subscription(
            String,
            'topic',
            lambda msg: counter.onMsg('default', msg),
            QoSProfile(depth=10))

    sub_transient = node.create_subscription(
            String,
            'topic',
            lambda msg: counter.onMsg('transient', msg),
            QoSProfile(
                depth=10,
                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL))

    sub_volatile = node.create_subscription(
            String,
            'topic',
            lambda msg: counter.onMsg('volatile', msg),
            QoSProfile(
                depth=10,
                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE))

    sub_reliable = node.create_subscription(
            String,
            'topic',
            lambda msg: counter.onMsg('reliable', msg),
            QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE))

    sub_best_effort = node.create_subscription(
            String,
            'topic',
            lambda msg: counter.onMsg('best_effort', msg),
            QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT))

    # Asserts to suppress unused variable warnings.
    assert sub_default
    assert sub_transient
    assert sub_volatile
    assert sub_reliable
    assert sub_best_effort

    timer = node.create_timer(0.5, lambda: print(counter.count))

    rclpy.spin(node)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
