# Copyright 2024 Open Source Robotics Foundation, Inc.
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

"""Implementation of 'ros2 latency' command for latency measurement."""

from ros2cli.command import CommandExtension


class LatencyCommand(CommandExtension):
    """Measure end-to-end latency between publishers and subscribers."""

    def add_arguments(self, parser, cli_name):
        """Add command-line arguments."""
        from ros2stats.verb.latency import LatencyVerb
        verb = LatencyVerb()
        verb.add_arguments(parser, cli_name)

    def main(self, *, args):
        """Execute the latency command."""
        from ros2stats.verb.latency import LatencyVerb
        verb = LatencyVerb()
        return verb.main(args=args)
