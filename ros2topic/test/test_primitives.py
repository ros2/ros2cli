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

from ros2topic.verb.echo import msg_to_csv
from ros2topic.verb.echo import msg_to_yaml
from ros2topic.verb.echo import register_yaml_representer
from test_communication import message_fixtures


def test_primitives():
    register_yaml_representer()
    # Smoke-test the formatters on a bunch of messages
    msgs = []
    msgs.extend(message_fixtures.get_msg_bounded_array_nested())
    msgs.extend(message_fixtures.get_msg_bounded_array_primitives())
    msgs.extend(message_fixtures.get_msg_builtins())
    msgs.extend(message_fixtures.get_msg_dynamic_array_nested())
    msgs.extend(message_fixtures.get_msg_dynamic_array_primitives())
    msgs.extend(message_fixtures.get_msg_empty())
    msgs.extend(message_fixtures.get_msg_fields_with_same_type())
    msgs.extend(message_fixtures.get_msg_nested())
    msgs.extend(message_fixtures.get_msg_primitives())
    msgs.extend(message_fixtures.get_msg_static_array_nested())
    msgs.extend(message_fixtures.get_msg_static_array_primitives())
    for m in msgs:
        msg_to_yaml(m)
        msg_to_csv(m)
