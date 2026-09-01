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

import argparse
from types import SimpleNamespace
from unittest.mock import call
from unittest.mock import MagicMock
from unittest.mock import patch

from ros2param.verb.load import LoadVerb


def _args(**kwargs):
    values = {
        'node_name': None,
        'parameter_file': 'params.yaml',
        'no_use_wildcard': False,
        'include_hidden_nodes': False,
        'timeout': 1,
        'service_timeout': None,
    }
    values.update(kwargs)
    return SimpleNamespace(**values)


def _run_wait_for_once(predicate, timeout):
    return predicate()


def test_parser_accepts_parameter_file_without_node_name():
    parser = argparse.ArgumentParser()
    LoadVerb().add_arguments(parser, 'ros2 param load')

    args = parser.parse_args(['params.yaml'])

    assert args.node_name is None
    assert args.parameter_file == 'params.yaml'


def test_loads_parameter_file_for_all_matching_nodes():
    args = _args(timeout=3)
    strategy = MagicMock()
    strategy.daemon_node = MagicMock()
    strategy_context = MagicMock()
    strategy_context.__enter__.return_value = strategy
    strategy_context.__exit__.return_value = False
    direct = MagicMock()
    direct_context = MagicMock()
    direct_context.__enter__.return_value = direct
    direct_context.__exit__.return_value = False

    discovered_nodes = [
        SimpleNamespace(full_name='/first'),
        SimpleNamespace(full_name='/second'),
        SimpleNamespace(full_name='/unmatched'),
    ]

    def parse_parameter_file(parameter_file, use_wildcard, target_nodes=None):
        if target_nodes == ['/unmatched']:
            # Per-node matching must not depend on the text of this error.
            raise RuntimeError('no parameters selected')
        return {'parameter': object()}

    with patch('ros2param.verb.load.NodeStrategy', return_value=strategy_context), \
            patch('ros2param.verb.load.DirectNode', return_value=direct_context), \
            patch('ros2param.verb.load.get_node_names', return_value=discovered_nodes), \
            patch(
                'ros2param.verb.load.parameter_dict_from_yaml_file',
                side_effect=parse_parameter_file
            ), \
            patch(
                'ros2param.verb.load.wait_for', side_effect=_run_wait_for_once
            ) as wait_for, \
            patch('ros2param.verb.load.load_parameter_file') as load_parameter_file:
        result = LoadVerb().main(args=args)

    assert result is None
    assert wait_for.call_args.args[1] == 3
    assert load_parameter_file.call_args_list == [
        call(
            node=direct, node_name='/first', parameter_file='params.yaml',
            use_wildcard=True, timeout=None),
        call(
            node=direct, node_name='/second', parameter_file='params.yaml',
            use_wildcard=True, timeout=None),
    ]


def test_returns_error_when_no_running_node_matches_file():
    args = _args()
    strategy = MagicMock()
    strategy.daemon_node = MagicMock()
    strategy_context = MagicMock()
    strategy_context.__enter__.return_value = strategy
    strategy_context.__exit__.return_value = False

    def parse_parameter_file(parameter_file, use_wildcard, target_nodes=None):
        if target_nodes is not None:
            raise RuntimeError('no parameters selected')
        return {'parameter': object()}

    with patch('ros2param.verb.load.NodeStrategy', return_value=strategy_context), \
            patch(
                'ros2param.verb.load.get_node_names',
                return_value=[SimpleNamespace(full_name='/unmatched')]
            ), \
            patch(
                'ros2param.verb.load.parameter_dict_from_yaml_file',
                side_effect=parse_parameter_file
            ), \
            patch('ros2param.verb.load.wait_for', side_effect=_run_wait_for_once):
        result = LoadVerb().main(args=args)

    assert result == 'No matching nodes found'
