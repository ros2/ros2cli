# Copyright 2026 Tomoya Fujita, Fumiya Ohnishi
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

from io import StringIO
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock
from unittest.mock import patch

from rcl_interfaces.msg import LoggerLevel

from ros2log.verb.get import GetVerb


def _make_context_manager(value):
    context_manager = MagicMock()
    context_manager.__enter__.return_value = value
    context_manager.__exit__.return_value = False
    return context_manager


class TestGetVerb(unittest.TestCase):
    """Test the get verb."""

    def test_requires_node_name_or_all(self):
        verb = GetVerb()
        args = SimpleNamespace(all=False, node_name=None)

        self.assertEqual(
            'Either a node name or --all must be specified',
            verb.main(args=args),
        )

    def test_rejects_node_name_with_all(self):
        verb = GetVerb()
        args = SimpleNamespace(all=True, node_name='/talker')

        self.assertEqual(
            'Node name cannot be used with --all',
            verb.main(args=args),
        )

    @patch('ros2log.verb.get.call_get_logger_levels')
    @patch('ros2log.verb.get.get_target_node_names')
    @patch('ros2log.verb.get.DirectNode')
    @patch('ros2log.verb.get.NodeStrategy')
    def test_single_node_output(
        self,
        mock_node_strategy,
        mock_direct_node,
        mock_get_target_node_names,
        mock_call_get_logger_levels,
    ):
        mock_node_strategy.return_value = _make_context_manager(object())
        mock_direct_node.return_value = _make_context_manager(object())
        mock_get_target_node_names.return_value = (['/talker'], None)
        mock_call_get_logger_levels.return_value = {
            '/talker': [LoggerLevel(name='talker', level=20)],
        }

        verb = GetVerb()
        args = SimpleNamespace(all=False, node_name='/talker')

        captured = StringIO()
        sys.stdout = captured
        try:
            result = verb.main(args=args)
        finally:
            sys.stdout = sys.__stdout__

        self.assertEqual(0, result)
        self.assertEqual('INFO\n', captured.getvalue())

    @patch('ros2log.verb.get.call_get_logger_levels')
    @patch('ros2log.verb.get.get_target_node_names')
    @patch('ros2log.verb.get.DirectNode')
    @patch('ros2log.verb.get.NodeStrategy')
    def test_all_nodes_prefixes_output(
        self,
        mock_node_strategy,
        mock_direct_node,
        mock_get_target_node_names,
        mock_call_get_logger_levels,
    ):
        mock_node_strategy.return_value = _make_context_manager(object())
        mock_direct_node.return_value = _make_context_manager(object())
        mock_get_target_node_names.return_value = (['/talker', '/worker'], None)
        mock_call_get_logger_levels.return_value = {
            '/talker': [LoggerLevel(name='talker', level=0)],
            '/worker': [LoggerLevel(name='worker', level=40)],
        }

        verb = GetVerb()
        args = SimpleNamespace(all=True, node_name=None)

        captured = StringIO()
        sys.stdout = captured
        try:
            result = verb.main(args=args)
        finally:
            sys.stdout = sys.__stdout__

        self.assertEqual(0, result)
        self.assertEqual('/talker: UNSET\n/worker: ERROR\n', captured.getvalue())

    @patch('ros2log.verb.get.call_get_logger_levels')
    @patch('ros2log.verb.get.get_target_node_names')
    @patch('ros2log.verb.get.DirectNode')
    @patch('ros2log.verb.get.NodeStrategy')
    def test_single_node_service_exception_returns_message(
        self,
        mock_node_strategy,
        mock_direct_node,
        mock_get_target_node_names,
        mock_call_get_logger_levels,
    ):
        mock_node_strategy.return_value = _make_context_manager(object())
        mock_direct_node.return_value = _make_context_manager(object())
        mock_get_target_node_names.return_value = (['/talker'], None)
        mock_call_get_logger_levels.return_value = {
            '/talker': RuntimeError('boom'),
        }

        verb = GetVerb()
        args = SimpleNamespace(all=False, node_name='/talker')

        self.assertEqual(
            "Exception while calling service of node '/talker': boom",
            verb.main(args=args),
        )
