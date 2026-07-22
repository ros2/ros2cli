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
import unittest

from ros2log.verb.levels import get_log_levels_from_msg
from ros2log.verb.levels import LEVEL_NAME_TO_VALUE
from ros2log.verb.levels import LEVEL_VALUE_TO_NAME
from ros2log.verb.levels import LevelsVerb
from ros2log.verb.levels import LOG_LEVEL_DESCRIPTIONS


class TestGetLogLevelsFromMsg(unittest.TestCase):
    """Test the get_log_levels_from_msg helper function."""

    def test_returns_list_of_tuples(self):
        """Test that the function returns a list of (name, value) tuples."""
        levels = get_log_levels_from_msg()
        self.assertIsInstance(levels, list)
        for item in levels:
            self.assertIsInstance(item, tuple)
            self.assertEqual(len(item), 2)
            self.assertIsInstance(item[0], str)
            self.assertIsInstance(item[1], int)

    def test_contains_expected_levels(self):
        """Test that all expected log levels are present."""
        levels = get_log_levels_from_msg()
        level_names = [name for name, _ in levels]
        for expected in ['UNSET', 'DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']:
            self.assertIn(expected, level_names)

    def test_sorted_by_value(self):
        """Test that levels are sorted by numeric value."""
        levels = get_log_levels_from_msg()
        values = [value for _, value in levels]
        self.assertEqual(values, sorted(values))

    def test_unset_is_zero(self):
        """Test that UNSET has value 0."""
        levels = get_log_levels_from_msg()
        level_dict = dict(levels)
        self.assertEqual(level_dict['UNSET'], 0)

    def test_level_ordering(self):
        """Test that levels are in expected severity order."""
        levels = get_log_levels_from_msg()
        level_dict = dict(levels)
        self.assertLess(level_dict['UNSET'], level_dict['DEBUG'])
        self.assertLess(level_dict['DEBUG'], level_dict['INFO'])
        self.assertLess(level_dict['INFO'], level_dict['WARN'])
        self.assertLess(level_dict['WARN'], level_dict['ERROR'])
        self.assertLess(level_dict['ERROR'], level_dict['FATAL'])


class TestLevelMaps(unittest.TestCase):
    """Test the module-level level mapping dictionaries."""

    def test_name_to_value_contains_all_levels(self):
        """Test that LEVEL_NAME_TO_VALUE contains all expected levels."""
        for name in ['UNSET', 'DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']:
            self.assertIn(name, LEVEL_NAME_TO_VALUE)

    def test_value_to_name_contains_all_levels(self):
        """Test that LEVEL_VALUE_TO_NAME contains all expected levels."""
        for name in ['UNSET', 'DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']:
            value = LEVEL_NAME_TO_VALUE[name]
            self.assertIn(value, LEVEL_VALUE_TO_NAME)
            self.assertEqual(LEVEL_VALUE_TO_NAME[value], name)

    def test_round_trip(self):
        """Test that name->value->name round-trips correctly."""
        for name, value in LEVEL_NAME_TO_VALUE.items():
            self.assertEqual(LEVEL_VALUE_TO_NAME[value], name)


class TestLogLevelDescriptions(unittest.TestCase):
    """Test the LOG_LEVEL_DESCRIPTIONS dictionary."""

    def test_all_levels_have_descriptions(self):
        """Test that every level returned by get_log_levels_from_msg has a description."""
        levels = get_log_levels_from_msg()
        for name, _ in levels:
            self.assertIn(name, LOG_LEVEL_DESCRIPTIONS)
            self.assertTrue(len(LOG_LEVEL_DESCRIPTIONS[name]) > 0)


class TestLevelsVerb(unittest.TestCase):
    """Test the LevelsVerb class."""

    def _run_verb(self, value=False):
        """Run the LevelsVerb and capture its output."""
        verb = LevelsVerb()

        class Args:
            pass

        args = Args()
        args.value = value

        captured = StringIO()
        sys.stdout = captured
        try:
            result = verb.main(args=args)
        finally:
            sys.stdout = sys.__stdout__

        return result, captured.getvalue()

    def test_main_returns_zero(self):
        """Test that main() returns 0."""
        result, _ = self._run_verb()
        self.assertEqual(result, 0)

    def test_output_contains_all_level_names(self):
        """Test that output contains all expected level names."""
        _, output = self._run_verb()
        for name in ['UNSET', 'DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']:
            self.assertIn(name, output)

    def test_output_contains_descriptions(self):
        """Test that output contains the descriptions."""
        _, output = self._run_verb()
        for desc in LOG_LEVEL_DESCRIPTIONS.values():
            self.assertIn(desc, output)

    def test_output_without_values(self):
        """Test output format without --value flag."""
        _, output = self._run_verb(value=False)
        lines = output.strip().split('\n')
        self.assertGreater(len(lines), 0)
        for line in lines:
            # Should not contain parenthesized numeric values
            self.assertNotRegex(line, r'\(\s*\d+\)')

    def test_output_with_values(self):
        """Test output format with --value flag."""
        _, output = self._run_verb(value=True)
        lines = output.strip().split('\n')
        self.assertGreater(len(lines), 0)
        for line in lines:
            # Each line should contain a parenthesized numeric value
            self.assertRegex(line, r'\(\s*\d+\)')

    def test_output_levels_in_order(self):
        """Test that levels are printed in ascending severity order."""
        _, output = self._run_verb()
        lines = output.strip().split('\n')
        level_names_in_output = []
        for line in lines:
            name = line.split(':')[0].strip().split()[0]
            level_names_in_output.append(name)
        expected_order = ['UNSET', 'DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']
        self.assertEqual(level_names_in_output, expected_order)


if __name__ == '__main__':
    unittest.main()
