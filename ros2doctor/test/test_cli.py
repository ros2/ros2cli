# Copyright 2020 Open Source Robotics Foundation, Inc.
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

from argparse import Namespace
import unittest
import unittest.mock as mock

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing.actions
import launch_testing.markers

import pytest

from ros2doctor.verb.hello import HelloVerb
from ros2doctor.verb.hello import SummaryTable


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                launch_testing.actions.ReadyToTest()
            ]
        )
    ])


def _generate_expected_summary_table():
    """Generate expected summary table for one emit period on a single host."""
    expected_summary = SummaryTable()
    # 1 pub/send per default emit period
    expected_summary.increment_pub()
    expected_summary.increment_send()
    return expected_summary


class TestROS2DoctorCLI(unittest.TestCase):

    def test_hello_single_host(self):
        """Run HelloVerb for one emit period on a single host."""
        args = Namespace()
        args.topic = '/canyouhearme'
        args.emit_period = 0.1
        args.print_period = 1.0
        args.ttl = None
        args.once = True
        with mock.patch('socket.gethostname', return_value='!nv@lid-n*de-n4me'):
            summary = SummaryTable()
            hello_verb = HelloVerb()
            hello_verb.main(args=args, summary_table=summary)
            expected_summary = _generate_expected_summary_table()
            self.assertEqual(summary._pub, expected_summary._pub)
            self.assertEqual(summary._sub, expected_summary._sub)
            self.assertEqual(summary._send, expected_summary._send)
            self.assertEqual(summary._receive, expected_summary._receive)
