# Copyright 2025 Open Source Robotics Foundation, Inc.
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

import array
from pathlib import Path
import sys
import time
import unittest
import xmlrpc.client

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import ResetEnvironment
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
from launch_testing_ros.actions import EnableRmwIsolation

import pytest

from rcl_interfaces.msg import ParameterType
import rclpy
from rclpy.parameter import get_parameter_value
from rclpy.utilities import get_available_rmw_implementations
from ros2cli.helpers import get_rmw_additional_env
from ros2cli.node.strategy import NodeStrategy
from ros2doctor.api.parameter import ParameterReport


TEST_NODE = 'test_node'
TEST_NAMESPACE = '/foo'

TEST_TIMEOUT = 20.0


def test_get_parameter_value():
    """Test get_parameter_value with various inputs."""
    test_cases = [
        ('true', ParameterType.PARAMETER_BOOL, 'bool_value', True),
        ('false', ParameterType.PARAMETER_BOOL, 'bool_value', False),
        ('1', ParameterType.PARAMETER_INTEGER, 'integer_value', 1),
        ('0', ParameterType.PARAMETER_INTEGER, 'integer_value', 0),
        ('-1', ParameterType.PARAMETER_INTEGER, 'integer_value', -1),
        ('1.0', ParameterType.PARAMETER_DOUBLE, 'double_value', 1.0),
        ('0.0', ParameterType.PARAMETER_DOUBLE, 'double_value', 0.0),
        ('-1.0', ParameterType.PARAMETER_DOUBLE, 'double_value', -1.0),
        ('1.1234', ParameterType.PARAMETER_DOUBLE, 'double_value', 1.1234),
        ('foo', ParameterType.PARAMETER_STRING, 'string_value', 'foo'),
        (
            '[false, true]',
            ParameterType.PARAMETER_BOOL_ARRAY,
            'bool_array_value',
            [False, True],
        ),
        (
            '[-1, 0, 1]',
            ParameterType.PARAMETER_INTEGER_ARRAY,
            'integer_array_value',
            array.array('q', (-1, 0, 1)),
        ),
        (
            '[-1.0, 0.0, 1.0, 1.1234]',
            ParameterType.PARAMETER_DOUBLE_ARRAY,
            'double_array_value',
            array.array('d', (-1.0, 0.0, 1.0, 1.1234)),
        ),
        (
            '["foo", "bar", "buzz"]',
            ParameterType.PARAMETER_STRING_ARRAY,
            'string_array_value',
            ['foo', 'bar', 'buzz'],
        ),
        (
            '["foo", "bar", "buzz"',
            ParameterType.PARAMETER_STRING,
            'string_value',
            '["foo", "bar", "buzz"',
        ),
        ('off', ParameterType.PARAMETER_BOOL, 'bool_value', False),
        ('!!str off', ParameterType.PARAMETER_STRING, 'string_value', 'off'),
        (
            '[true,0.1,1]',
            ParameterType.PARAMETER_STRING,
            'string_value',
            '[true,0.1,1]',
        ),
    ]
    for (
        string_value, expected_type, value_attribute, expected_value
    ) in test_cases:
        value = get_parameter_value(string_value=string_value)
        assert value.type == expected_type
        assert getattr(value, value_attribute) == expected_value


@pytest.mark.rostest
@launch_testing.parametrize(
    'rmw_implementation', get_available_rmw_implementations()
)
def generate_test_description(rmw_implementation):
    path_to_fixtures = Path(__file__).parent / 'fixtures'
    additional_env = get_rmw_additional_env(rmw_implementation)
    additional_env['PYTHONUNBUFFERED'] = '1'
    set_env_actions = [
        SetEnvironmentVariable(k, v) for k, v in additional_env.items()
    ]

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=['ros2', 'daemon', 'stop'],
                name='daemon-stop',
                on_exit=[
                    *set_env_actions,
                    EnableRmwIsolation(),
                    RegisterEventHandler(
                        OnShutdown(
                            on_shutdown=[
                                ExecuteProcess(
                                    cmd=['ros2', 'daemon', 'stop'],
                                    name='daemon-stop-isolated',
                                    additional_env=dict(additional_env),
                                ),
                                ResetEnvironment(),
                            ]
                        )
                    ),
                    ExecuteProcess(
                        cmd=['ros2', 'daemon', 'start'],
                        name='daemon-start',
                        on_exit=[
                            Node(
                                executable=sys.executable,
                                arguments=[str(
                                    path_to_fixtures /
                                    'parameter_fixtures_node.py'
                                )],
                                name=TEST_NODE,
                                namespace=TEST_NAMESPACE,
                            ),
                            launch_testing.actions.ReadyToTest(),
                        ],
                    ),
                ],
            )
        ]
    )


class TestParameterAPI(unittest.TestCase):
    """Test ros2doctor ParameterReport with running nodes."""

    @classmethod
    def setUpClass(
        cls, launch_service, proc_info, proc_output, rmw_implementation
    ):
        # Skip zenoh because it requires a router for node discovery
        if rmw_implementation == 'rmw_zenoh_cpp':
            raise unittest.SkipTest(
                'rmw_zenoh_cpp requires router for discovery'
            )
        cls.rmw_implementation = rmw_implementation

    def setUp(self):
        """Wait for test nodes to be discovered."""
        start_time = time.time()
        timed_out = True
        with NodeStrategy(None) as node:
            while (time.time() - start_time) < TEST_TIMEOUT:
                try:
                    services = node.get_service_names_and_types_by_node(
                        TEST_NODE, TEST_NAMESPACE
                    )
                except rclpy.node.NodeNameNonExistentError:
                    continue
                except ConnectionRefusedError:
                    continue
                except xmlrpc.client.Fault as e:
                    if 'NodeNameNonExistentError' in e.faultString:
                        continue
                    raise

                service_names = [
                    name_type_tuple[0] for name_type_tuple in services
                ]
                list_params = f'{TEST_NAMESPACE}/{TEST_NODE}/list_parameters'
                if (
                    len(service_names) > 0
                    and list_params in service_names
                ):
                    timed_out = False
                    break
        if timed_out:
            self.fail(
                f'CLI daemon failed to find test node after '
                f'{TEST_TIMEOUT} seconds'
            )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_parameter_report(self):
        """Test ParameterReport lists parameters from running nodes."""
        report = ParameterReport().report()
        self.assertEqual(report.name, 'PARAMETER LIST')

        # Parse items to find parameters for our test node
        node_params = {}
        current_node = None
        for key, value in report.items:
            if key == 'node':
                current_node = value
                node_params[current_node] = []
            elif key == 'parameter' and current_node:
                node_params[current_node].append(value)

        target_node = f'{TEST_NAMESPACE}/{TEST_NODE}'
        self.assertIn(target_node, node_params)

        params = node_params[target_node]

        param_dict = {}
        for p in params:
            if ': ' in p:
                name, val = p.split(': ', 1)
                param_dict[name] = val
            else:
                param_dict[p] = None

        expected_values = {
            'bool_param': 'True',
            'int_param': '42',
            'double_param': '3.14',
            'str_param': 'hello',
            'nested.param': 'nested_value',
            'use_sim_time': 'False',
        }

        for name, val in expected_values.items():
            self.assertIn(name, param_dict)
            self.assertEqual(param_dict[name], val)

        array_params = [
            'bool_array_param',
            'int_array_param',
            'double_array_param',
            'str_array_param',
        ]
        for name in array_params:
            self.assertIn(name, param_dict)
            self.assertIsNotNone(param_dict[name])

        items_dict = {item[0]: item[1] for item in report.items}
        self.assertIn('total nodes checked', items_dict)
        self.assertGreaterEqual(items_dict['total nodes checked'], 1)
