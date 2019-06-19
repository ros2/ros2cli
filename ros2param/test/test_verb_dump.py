# Copyright 2019 Canonical Ltd
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

import os
import tempfile

# import pytest
import unittest

import rclpy

from ros2cli import cli

TEST_NODE = 'test_node'
TEST_NAMESPACE = ''


class TestVerbDump(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node(
            TEST_NODE, namespace=TEST_NAMESPACE, context=cls.context,
            allow_undeclared_parameters=True)

        cls.node.declare_parameter('bool_param', True)
        cls.node.declare_parameter('int_param', 42)
        cls.node.declare_parameter('double_param', 1.23)
        cls.node.declare_parameter('str_param', 'Hello World')
        cls.node.declare_parameter('foo/str_param', 'foo')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_verb_dump_invalid_node(self):
        assert cli.main(
                argv=['param', 'dump', 'invalid_node']) == 'Node not found'

    def test_verb_dump_invalid_path(self):
        assert cli.main(
                argv=['param', 'dump', 'test_node', '--file-path', 'invalid_path']) == 'Invalid output directory'

    def test_verb_dump(self):
        assert cli.main(
                    argv=['param', 'dump', 'test_node', '--file-path', '.']) == 0
