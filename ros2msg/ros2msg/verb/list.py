# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import warnings

from ros2msg.api import get_all_message_types
from ros2msg.verb import VerbExtension


class ListVerb(VerbExtension):
    """Output a list of available message types."""

    def main(self, *, args):
        warnings.warn(
            "'ros2 msg' is deprecated and will be removed in a future ROS release. "
            "Instead use: 'ros2 interface list -m'"
        )
        message_types = get_all_message_types()
        for package_name in sorted(message_types.keys()):
            for message_name in sorted(message_types[package_name]):
                print('{package_name}/msg/{message_name}'.format_map(locals()))
