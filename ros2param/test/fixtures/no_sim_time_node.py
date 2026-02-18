# Copyright 2026 Tim Wendt
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
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import ExternalShutdownException


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = rclpy.create_node('no_sim_time_node')

            # Ensure use_sim_time does not exist on this node
            try:
                node.undeclare_parameter('use_sim_time')
            except ParameterNotDeclaredException:
                pass

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('no_sim_time_node stopped cleanly')


if __name__ == '__main__':
    main()
