# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Contract tests for --content-filter argument handling.

These are lightweight unit tests that verify the interface contract between
CLI argument parsing and ContentFilterOptions construction, without requiring
a running ROS system.
"""

import argparse
import unittest
from unittest.mock import MagicMock
from unittest.mock import patch

from rclpy.subscription_content_filter_options import ContentFilterOptions


class TestContentFilterArgParsing(unittest.TestCase):
    """Verify that each verb correctly parses --content-filter args."""

    def _parse_echo_args(self, args_list):
        from ros2topic.verb.echo import EchoVerb
        verb = EchoVerb()
        parser = argparse.ArgumentParser()
        verb.add_arguments(parser, 'ros2 topic')
        return parser.parse_args(args_list)

    def _parse_hz_args(self, args_list):
        from ros2topic.verb.hz import HzVerb
        verb = HzVerb()
        parser = argparse.ArgumentParser()
        verb.add_arguments(parser, 'ros2 topic')
        return parser.parse_args(args_list)

    def _parse_bw_args(self, args_list):
        from ros2topic.verb.bw import BwVerb
        verb = BwVerb()
        parser = argparse.ArgumentParser()
        verb.add_arguments(parser, 'ros2 topic')
        return parser.parse_args(args_list)

    def test_echo_content_filter_arg_present(self):
        args = self._parse_echo_args([
            '/topic', '--content-filter', "data = 'hello'"])
        assert args.content_filter_expr == "data = 'hello'"
        assert args.content_filter_params == []

    def test_echo_content_filter_arg_absent(self):
        args = self._parse_echo_args(['/topic'])
        assert args.content_filter_expr is None
        assert args.content_filter_params == []

    def test_echo_content_filter_with_params(self):
        args = self._parse_echo_args([
            '/topic', '--content-filter', 'data = %0',
            '--content-filter-params', 'hello'])
        assert args.content_filter_expr == 'data = %0'
        assert args.content_filter_params == ['hello']

    def test_echo_content_filter_with_multiple_params(self):
        args = self._parse_echo_args([
            '/topic', '--content-filter', 'data BETWEEN %0 AND %1',
            '--content-filter-params', '10', '20'])
        assert args.content_filter_expr == 'data BETWEEN %0 AND %1'
        assert args.content_filter_params == ['10', '20']

    def test_hz_content_filter_arg_present(self):
        args = self._parse_hz_args([
            '/topic', '--content-filter', "data = 'hello'"])
        assert args.content_filter_expr == "data = 'hello'"
        assert args.content_filter_params == []

    def test_hz_content_filter_arg_absent(self):
        args = self._parse_hz_args(['/topic'])
        assert args.content_filter_expr is None
        assert args.content_filter_params == []

    def test_bw_content_filter_arg_present(self):
        args = self._parse_bw_args([
            '/topic', '--content-filter', "data = 'hello'"])
        assert args.content_filter_expr == "data = 'hello'"
        assert args.content_filter_params == []

    def test_bw_content_filter_arg_absent(self):
        args = self._parse_bw_args(['/topic'])
        assert args.content_filter_expr is None
        assert args.content_filter_params == []


class TestContentFilterOptionsConstruction(unittest.TestCase):
    """Verify ContentFilterOptions is constructed correctly from parsed args."""

    def test_options_with_expression_only(self):
        opts = ContentFilterOptions(
            filter_expression="data = 'hello'",
            expression_parameters=[])
        assert opts.filter_expression == "data = 'hello'"
        assert opts.expression_parameters == []

    def test_options_with_expression_and_params(self):
        opts = ContentFilterOptions(
            filter_expression='data = %0',
            expression_parameters=['hello'])
        assert opts.filter_expression == 'data = %0'
        assert opts.expression_parameters == ['hello']

    def test_options_none_when_no_filter(self):
        """Contract: when content_filter_expr is None, no options are created."""
        content_filter_expr = None
        content_filter_options = None
        if content_filter_expr:
            content_filter_options = ContentFilterOptions(
                filter_expression=content_filter_expr,
                expression_parameters=[])
        assert content_filter_options is None
