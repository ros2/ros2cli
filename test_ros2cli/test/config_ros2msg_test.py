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


class Config():
    pass


config = Config()

config.verb = 'msg'

config.options = [
    'list',
    'package',
    'packages',
    'show',
    'show_old_style',
]

config.arguments_by_option = {
    'list': ['list'],
    'package': ['package', 'std_msgs'],
    'packages': ['packages'],
    'show': ['show', 'std_msgs/msg/String'],
    'show_old_style': ['show', 'std_msgs/String'],
}

config.actions_by_option = {
    'list': [],
    'package': [],
    'packages': [],
    'show': [],
    'show_old_style': [],
}

some_messages_from_std_msgs = [
    'std_msgs/msg/Bool',
    'std_msgs/msg/Float32',
    'std_msgs/msg/Float64',
]

config.msgs_by_option = {
    'list': some_messages_from_std_msgs,
    'package': some_messages_from_std_msgs,
    'packages': ['std_msgs'],
    'show': ['string data'],
    'show_old_style': ['string data'],
}
