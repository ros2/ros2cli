# Copyright 2025 Tomoya Fujita, Fumiya Ohnishi
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

from rcl_interfaces.msg import Log
import rclpy
from rclpy.node import Node

from ros2log.verb.watch import LogWatcher


class TestWatchVerb(unittest.TestCase):
    """Test the watch verb functionality."""

    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Tear down test fixtures."""
        rclpy.shutdown()

    def setUp(self):
        """Set up each test."""
        self.node = Node('test_log_watch_node')

    def tearDown(self):
        """Clean up after each test."""
        self.node.destroy_node()

    def test_level_filter(self):
        """Test that level filtering works correctly."""
        watcher = LogWatcher(
            self.node,
            level_filter='ERROR',
            enable_color=False,
            show_timestamp=False,
            enable_content_filter=False,
        )

        # Create test log messages
        debug_msg = Log(level=Log.DEBUG, name='test_logger', msg='Debug message')
        info_msg = Log(level=Log.INFO, name='test_logger', msg='Info message')
        warn_msg = Log(level=Log.WARN, name='test_logger', msg='Warn message')
        error_msg = Log(level=Log.ERROR, name='test_logger', msg='Error message')
        fatal_msg = Log(level=Log.FATAL, name='test_logger', msg='Fatal message')

        # Capture stdout
        captured_output = StringIO()
        sys.stdout = captured_output

        # Send messages through callback
        watcher._log_callback(debug_msg)
        watcher._log_callback(info_msg)
        watcher._log_callback(warn_msg)
        watcher._log_callback(error_msg)
        watcher._log_callback(fatal_msg)

        # Restore stdout
        sys.stdout = sys.__stdout__

        # Check that only ERROR message was printed
        output = captured_output.getvalue()
        self.assertNotIn('Debug message', output)
        self.assertNotIn('Info message', output)
        self.assertNotIn('Warn message', output)
        self.assertIn('Error message', output)
        self.assertIn('Fatal message', output)

    def test_logger_filter(self):
        """Test that logger name filtering works correctly."""
        watcher = LogWatcher(
            self.node,
            logger_filter='my_logger',
            enable_color=False,
            show_timestamp=False,
            enable_content_filter=False,
        )

        # Create test log messages
        msg1 = Log(level=Log.INFO, name='my_logger', msg='Message from my_logger')
        msg2 = Log(level=Log.INFO, name='other_logger', msg='Message from other_logger')

        # Capture stdout
        captured_output = StringIO()
        sys.stdout = captured_output

        # Send messages through callback
        watcher._log_callback(msg1)
        watcher._log_callback(msg2)

        # Restore stdout
        sys.stdout = sys.__stdout__

        # Check that only my_logger message was printed
        output = captured_output.getvalue()
        self.assertIn('Message from my_logger', output)
        self.assertNotIn('Message from other_logger', output)

    def test_regex_filter(self):
        """Test that regex filtering works correctly."""
        watcher = LogWatcher(
            self.node,
            regex_filter='sensor.*timeout',
            enable_color=False,
            show_timestamp=False,
            enable_content_filter=False,
        )

        # Create test log messages
        msg1 = Log(level=Log.INFO, name='test', msg='sensor camera timeout')
        msg2 = Log(level=Log.INFO, name='test', msg='sensor lidar timeout')
        msg3 = Log(level=Log.INFO, name='test', msg='motor timeout')

        # Capture stdout
        captured_output = StringIO()
        sys.stdout = captured_output

        # Send messages through callback
        watcher._log_callback(msg1)
        watcher._log_callback(msg2)
        watcher._log_callback(msg3)

        # Restore stdout
        sys.stdout = sys.__stdout__

        # Check that only messages matching the pattern were printed
        output = captured_output.getvalue()
        self.assertIn('sensor camera timeout', output)
        self.assertIn('sensor lidar timeout', output)
        self.assertNotIn('motor timeout', output)

    def test_combined_filters(self):
        """Test that multiple filters work together."""
        watcher = LogWatcher(
            self.node,
            level_filter='ERROR',
            logger_filter='my_logger',
            regex_filter='camera',
            enable_color=False,
            show_timestamp=False,
            enable_content_filter=False,
        )

        # Create test log messages
        msg1 = Log(level=Log.ERROR, name='my_logger', msg='camera error')
        msg2 = Log(level=Log.ERROR, name='my_logger', msg='lidar error')
        msg3 = Log(level=Log.ERROR, name='other_logger', msg='camera error')
        msg4 = Log(level=Log.INFO, name='my_logger', msg='camera info')

        # Capture stdout
        captured_output = StringIO()
        sys.stdout = captured_output

        # Send messages through callback
        watcher._log_callback(msg1)
        watcher._log_callback(msg2)
        watcher._log_callback(msg3)
        watcher._log_callback(msg4)

        # Restore stdout
        sys.stdout = sys.__stdout__

        # Check that only the message matching all filters was printed
        output = captured_output.getvalue()
        self.assertIn('camera error', output)
        # Verify the output appears exactly once (only msg1)
        self.assertEqual(output.count('camera error'), 1)

    def test_no_color_output(self):
        """Test that no-color option removes ANSI codes."""
        watcher = LogWatcher(
            self.node,
            enable_color=False,
            show_timestamp=False,
            enable_content_filter=False,
        )

        msg = Log(level=Log.ERROR, name='test', msg='Test message')

        # Capture stdout
        captured_output = StringIO()
        sys.stdout = captured_output

        watcher._log_callback(msg)

        # Restore stdout
        sys.stdout = sys.__stdout__

        output = captured_output.getvalue()
        # Check that no ANSI escape codes are present
        self.assertNotIn('\033[', output)

    def test_function_detail_output(self):
        """Test that function detail option includes function info."""
        watcher = LogWatcher(
            self.node,
            enable_color=False,
            show_timestamp=False,
            show_function_detail=True,
            enable_content_filter=False,
        )

        msg = Log(
            level=Log.INFO,
            name='test',
            msg='Test message',
            function='my_function',
            file='test.py',
            line=42
        )

        # Capture stdout
        captured_output = StringIO()
        sys.stdout = captured_output

        watcher._log_callback(msg)

        # Restore stdout
        sys.stdout = sys.__stdout__

        output = captured_output.getvalue()
        # Check that function details are included
        self.assertIn('my_function', output)
        self.assertIn('test.py', output)
        self.assertIn('42', output)


if __name__ == '__main__':
    unittest.main()
