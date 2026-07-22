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

import unittest
from unittest.mock import patch

from ros2log.api import call_get_logger_levels
from ros2log.api import call_set_logger_levels
from ros2log.api import format_logger_service_unavailable_error
from ros2log.api import get_get_logger_levels_service_name
from ros2log.api import get_logger_name_for_node
from ros2log.api import get_set_logger_levels_service_name
from ros2log.api import get_target_node_names
from ros2node.api import NodeName


class TestLoggerServiceNameHelpers(unittest.TestCase):
    """Test logger service name helpers."""

    def test_empty_node_name_raises_value_error_for_get_service_name(self):
        with self.assertRaisesRegex(ValueError, 'node_name must not be empty'):
            get_get_logger_levels_service_name('')

    def test_empty_node_name_raises_value_error_for_set_service_name(self):
        with self.assertRaisesRegex(ValueError, 'node_name must not be empty'):
            get_set_logger_levels_service_name('')

    def test_empty_node_name_raises_value_error_for_unavailable_error(self):
        with self.assertRaisesRegex(ValueError, 'node_name must not be empty'):
            format_logger_service_unavailable_error('')


class TestGetLoggerNameForNode(unittest.TestCase):
    """Test node-name to logger-name conversion helpers."""

    def test_empty_node_name_raises_value_error(self):
        with self.assertRaisesRegex(ValueError, 'node_name must not be empty'):
            get_logger_name_for_node('')

    def test_root_namespace(self):
        self.assertEqual(get_logger_name_for_node('/talker'), 'talker')

    def test_relative_node_name(self):
        self.assertEqual(get_logger_name_for_node('talker'), 'talker')

    def test_namespaced_node(self):
        self.assertEqual(get_logger_name_for_node('/demo/talker'), 'demo.talker')


class TestGetTargetNodeNames(unittest.TestCase):
    """Test node-resolution for logger service verbs."""

    @patch('ros2log.api.get_logger_service_nodes')
    @patch('ros2log.api.get_node_names')
    def test_all_nodes_returns_sorted_logger_service_nodes(
        self,
        mock_get_node_names,
        mock_get_logger_service_nodes,
    ):
        mock_get_node_names.return_value = []
        mock_get_logger_service_nodes.return_value = [
            NodeName('b', '/', '/b'),
            NodeName('a', '/', '/a'),
        ]

        node_names, error = get_target_node_names(node=object(), all_nodes=True)

        self.assertEqual(['/a', '/b'], node_names)
        self.assertIsNone(error)

    @patch('ros2log.api.get_logger_service_nodes')
    @patch('ros2log.api.get_node_names')
    def test_missing_node_returns_not_found(
        self,
        mock_get_node_names,
        mock_get_logger_service_nodes,
    ):
        mock_get_node_names.return_value = []
        mock_get_logger_service_nodes.return_value = []

        node_names, error = get_target_node_names(node=object(), node_name='/missing')

        self.assertIsNone(node_names)
        self.assertEqual('Node not found', error)

    @patch('ros2log.api.get_logger_service_nodes')
    @patch('ros2log.api.get_node_names')
    def test_node_without_logger_service_returns_specific_error(
        self,
        mock_get_node_names,
        mock_get_logger_service_nodes,
    ):
        mock_get_node_names.return_value = [
            NodeName('listener', '/', '/listener'),
        ]
        mock_get_logger_service_nodes.return_value = []

        node_names, error = get_target_node_names(node=object(), node_name='/listener')

        self.assertIsNone(node_names)
        self.assertEqual(
            format_logger_service_unavailable_error('/listener'),
            error,
        )


class TestLoggerLevelServiceCalls(unittest.TestCase):
    """Test logger-level service call helpers."""

    def test_call_get_logger_levels_rejects_negative_timeout(self):
        with self.assertRaisesRegex(
            ValueError,
            'timeout_sec must be non-negative, got -1.0',
        ):
            call_get_logger_levels(
                node=object(),
                logger_names_by_node={},
                timeout_sec=-1.0,
            )

    def test_call_get_logger_levels_allows_zero_timeout(self):
        self.assertEqual(
            {},
            call_get_logger_levels(
                node=object(),
                logger_names_by_node={},
                timeout_sec=0.0,
            ),
        )

    def test_call_set_logger_levels_rejects_negative_timeout(self):
        with self.assertRaisesRegex(
            ValueError,
            'timeout_sec must be non-negative, got -1.0',
        ):
            call_set_logger_levels(
                node=object(),
                levels_by_node={},
                timeout_sec=-1.0,
            )

    def test_call_set_logger_levels_allows_zero_timeout(self):
        self.assertEqual(
            {},
            call_set_logger_levels(
                node=object(),
                levels_by_node={},
                timeout_sec=0.0,
            ),
        )
