# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import argparse

from ros2topic.verb.pub import PubVerb


def test_timeout_alias():
    parser = argparse.ArgumentParser()
    PubVerb().add_arguments(parser, 'ros2 topic pub')

    args = parser.parse_args(['/chatter', 'std_msgs/msg/String', '--timeout', '2.5'])

    assert args.max_wait_time_secs == 2.5
