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

from argparse import ArgumentParser
import socket
import unittest
import unittest.mock as mock

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing.actions
import launch_testing.markers

import pytest

from ros2doctor.verb.hello import HelloMulticastUDPReceiver
from ros2doctor.verb.hello import HelloMulticastUDPSender
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

    def test_hello_single_host_custom_group_and_port(self):
        """Run HelloVerb with a custom multicast group and port."""
        group = '225.0.0.2'
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('', 0))
            port = sock.getsockname()[1]
        parser = ArgumentParser()
        hello_verb = HelloVerb()
        hello_verb.add_arguments(parser, 'ros2 doctor hello')
        args = parser.parse_args([
            '--group', group, '--port', str(port), '--once'])
        with mock.patch('socket.gethostname', return_value='!nv@lid-n*de-n4me'), \
                mock.patch(
                    'ros2doctor.verb.hello.HelloMulticastUDPSender',
                    wraps=HelloMulticastUDPSender) as sender, \
                mock.patch(
                    'ros2doctor.verb.hello.HelloMulticastUDPReceiver',
                    wraps=HelloMulticastUDPReceiver) as receiver, \
                mock.patch.object(
                    SummaryTable, 'format_print_summary',
                    wraps=SummaryTable.format_print_summary,
                    autospec=True) as format_print_summary:
            summary = SummaryTable()
            hello_verb.main(args=args, summary_table=summary)
            expected_summary = _generate_expected_summary_table()
            self.assertEqual(summary._pub, expected_summary._pub)
            self.assertEqual(summary._sub, expected_summary._sub)
            self.assertEqual(summary._send, expected_summary._send)
            self.assertEqual(summary._receive, expected_summary._receive)
            sender.assert_called_once_with(
                summary, group=args.group, port=args.port, ttl=args.ttl)
            receiver.assert_called_once_with(
                summary, group=args.group, port=args.port)
            format_print_summary.assert_called_once_with(
                summary, args.topic, args.print_period,
                group=args.group, port=args.port)
