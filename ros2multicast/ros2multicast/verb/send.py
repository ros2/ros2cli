# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from ros2multicast.api import send
from ros2multicast.verb import VerbExtension


class SendVerb(VerbExtension):
    """Send a single UDP multicast packet."""

    def main(self, *, args):
        print('Sending one UDP multicast datagram...')
        send(b'Hello World!')
