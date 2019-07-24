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
import threading

import unittest

import rclpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import PARAMETER_SEPARATOR_STRING
from ros2cli import cli

TEST_NODE = 'test_node'
TEST_NAMESPACE = ''

EXPECTED_PARAMETER_FILE = '''\
test_node:
  ros__parameters:
    bool_param: true
    double_param: 1.23
    foo:
      bar:
        str_param: foobar
      str_param: foo
    int_param: 42
    str_param: Hello World
'''


class TestVerbDump(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node(
            TEST_NODE, namespace=TEST_NAMESPACE, context=cls.context)

        cls.executor = MultiThreadedExecutor(context=cls.context, num_threads=2)
        cls.executor.add_node(cls.node)

        cls.node.declare_parameter('bool_param', True)
        cls.node.declare_parameter('int_param', 42)
        cls.node.declare_parameter('double_param', 1.23)
        cls.node.declare_parameter('str_param', 'Hello World')
        cls.node.declare_parameter('foo' + PARAMETER_SEPARATOR_STRING +
                                   'str_param', 'foo')
        cls.node.declare_parameter('foo' + PARAMETER_SEPARATOR_STRING +
                                   'bar' + PARAMETER_SEPARATOR_STRING +
                                   'str_param', 'foobar')

        # We need both the test node and 'dump'
        # node to be able to spin
        cls.exec_thread = threading.Thread(target=cls.executor.spin)
        cls.exec_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)
        cls.exec_thread.join()

    def test_verb_dump_invalid_node(self):
        assert cli.main(
                argv=['param', 'dump', 'invalid_node']) == 'Node not found'

    def test_verb_dump_invalid_path(self):
        assert cli.main(
                argv=['param', 'dump', 'test_node', '--output-dir', 'invalid_path']) \
                    == 'Invalid output directory'

    def test_verb_dump(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            assert cli.main(
                    argv=['param', 'dump', 'test_node', '--output-dir', tmpdir]) is None

            # Compare generated parameter file against expected
            generated_param_file = os.path.join(tmpdir, self.node.get_name() + ".yaml")
            assert (open(generated_param_file, 'r').read() ==
                    EXPECTED_PARAMETER_FILE)
