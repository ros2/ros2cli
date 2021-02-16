# Copyright 2020 Sony Corporation.
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

import pytest

from ros2cli.node.direct import add_arguments as add_direct_node_arguments
from ros2cli.node.direct import DEFAULT_TIMEOUT
from ros2cli.node.direct import DirectNode

TEST_NODE_NAME = 'test_node'


@pytest.fixture(scope='function')
def test_arguments_parser():
    parser = argparse.ArgumentParser()
    add_direct_node_arguments(parser)
    return parser


def test_default_arguments(test_arguments_parser):
    args = test_arguments_parser.parse_args()
    assert not args.use_sim_time
    assert DEFAULT_TIMEOUT == args.spin_time


def test_use_sim_time_arguments(test_arguments_parser):
    args = test_arguments_parser.parse_args(['--use-sim-time'])
    assert args.use_sim_time
    assert DEFAULT_TIMEOUT == args.spin_time


def test_spin_time_arguments(test_arguments_parser):
    args = test_arguments_parser.parse_args(['--spin-time', '10.0'])
    assert not args.use_sim_time
    assert 10.0 == args.spin_time


def test_use_sim_time_parameter():
    with DirectNode(args=[], node_name=TEST_NODE_NAME) as direct_node:
        assert not direct_node.node.get_parameter('use_sim_time').value

    args = argparse.Namespace(use_sim_time=False)
    with DirectNode(args, node_name=TEST_NODE_NAME) as direct_node:
        assert not direct_node.node.get_parameter('use_sim_time').value

    # TODO(fujitatomoya): enable this test once /clock callback thread is insulated.
    #                     see https://github.com/ros2/rclcpp/issues/1542
    # args = argparse.Namespace(use_sim_time=True)
    # with DirectNode(args, node_name=TEST_NODE_NAME) as direct_node:
    #     assert direct_node.node.get_parameter('use_sim_time').value
