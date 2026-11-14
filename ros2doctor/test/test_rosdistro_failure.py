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

import os
import unittest
from unittest import mock
import urllib.error
import warnings

from ros2doctor.api import generate_reports
from ros2doctor.api import run_checks
from ros2doctor.api.package import get_distro_package_versions
from ros2doctor.api.package import PackageCheck
from ros2doctor.api.package import PackageReport
from ros2doctor.api.platform import _check_platform_helper
from ros2doctor.api.platform import PlatformCheck
from ros2doctor.api.platform import RosdistroReport


def _unreachable_index(url):
    raise urllib.error.URLError('index is unreachable')


class _FakeEntryPoints:
    """Minimal stand-in for importlib.metadata.entry_points()."""

    def __init__(self, entry_points):
        self._entry_points = entry_points

    def select(self, group=None):
        return list(self._entry_points)

    def get(self, group, default=None):
        return list(self._entry_points)


class _FakeEntryPoint:
    """Entry point whose load() returns the given class."""

    def __init__(self, name, cls):
        self.name = name
        self._cls = cls

    def load(self):
        return self._cls


class _RaisingCheck:

    def category(self):
        return 'bogus'

    def check(self):
        raise RuntimeError('boom-check')


class _RaisingReport:

    def category(self):
        return 'bogus'

    def report(self):
        raise RuntimeError('boom-report')


class TestRosdistroFetchFailure(unittest.TestCase):
    """Checks and reports must degrade gracefully when rosdistro is unreachable."""

    def test_package_helpers_return_none_on_fetch_failure(self):
        """get_distro_package_versions returns None instead of raising URLError."""
        with mock.patch.dict(os.environ, {'ROS_DISTRO': 'rolling'}), \
                mock.patch('ros2doctor.api.package.rosdistro.get_index_url',
                           return_value='https://example.invalid/index-v4.yaml'), \
                mock.patch('ros2doctor.api.package.rosdistro.get_index',
                           side_effect=_unreachable_index), \
                warnings.catch_warnings():
            warnings.simplefilter('ignore')
            self.assertIsNone(get_distro_package_versions())

    def test_platform_helper_returns_none_on_fetch_failure(self):
        """_check_platform_helper returns None instead of raising URLError."""
        with mock.patch.dict(os.environ, {'ROS_DISTRO': 'rolling'}), \
                mock.patch('ros2doctor.api.platform.rosdistro.get_index_url',
                           return_value='https://example.invalid/index-v4.yaml'), \
                mock.patch('ros2doctor.api.platform.rosdistro.get_index',
                           side_effect=_unreachable_index), \
                warnings.catch_warnings():
            warnings.simplefilter('ignore')
            self.assertIsNone(_check_platform_helper())

    def test_package_check_reports_error_on_fetch_failure(self):
        """PackageCheck returns an error result instead of raising URLError."""
        with mock.patch.dict(os.environ, {'ROS_DISTRO': 'rolling'}), \
                mock.patch('ros2doctor.api.package.rosdistro.get_index',
                           side_effect=_unreachable_index), \
                warnings.catch_warnings():
            warnings.simplefilter('ignore')
            result = PackageCheck().check()
            self.assertGreaterEqual(result.error, 1)

    def test_platform_check_reports_error_on_fetch_failure(self):
        """PlatformCheck returns an error result instead of raising URLError."""
        with mock.patch.dict(os.environ, {'ROS_DISTRO': 'rolling'}), \
                mock.patch('ros2doctor.api.platform.rosdistro.get_index',
                           side_effect=_unreachable_index), \
                warnings.catch_warnings():
            warnings.simplefilter('ignore')
            result = PlatformCheck().check()
            self.assertEqual(result.error, 1)

    def test_package_report_is_empty_on_fetch_failure(self):
        """PackageReport returns an empty report instead of raising URLError."""
        with mock.patch.dict(os.environ, {'ROS_DISTRO': 'rolling'}), \
                mock.patch('ros2doctor.api.package.rosdistro.get_index',
                           side_effect=_unreachable_index), \
                warnings.catch_warnings():
            warnings.simplefilter('ignore')
            report = PackageReport().report()
            self.assertEqual(report.name, 'PACKAGE VERSIONS')
            self.assertEqual(report.items, [])

    def test_rosdistro_report_is_empty_on_fetch_failure(self):
        """RosdistroReport returns an empty report instead of raising URLError."""
        with mock.patch.dict(os.environ, {'ROS_DISTRO': 'rolling'}), \
                mock.patch('ros2doctor.api.platform.rosdistro.get_index',
                           side_effect=_unreachable_index), \
                warnings.catch_warnings():
            warnings.simplefilter('ignore')
            report = RosdistroReport().report()
            self.assertEqual(report.name, 'ROS 2 INFORMATION')
            self.assertEqual(report.items, [])

    def test_run_checks_warning_names_the_failure(self):
        """run_checks includes the underlying exception in its warning."""
        entry_points = _FakeEntryPoints([_FakeEntryPoint('BogusCheck', _RaisingCheck)])
        with mock.patch('ros2doctor.api.metadata.entry_points',
                        return_value=entry_points), \
                warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter('always')
            run_checks()
            messages = [str(w.message) for w in caught]
            self.assertTrue(
                any('Fail to call BogusCheck class functions' in m and
                    'boom-check' in m for m in messages),
                f'underlying exception missing from warnings: {messages}')

    def test_generate_reports_warning_names_the_failure(self):
        """generate_reports includes the underlying exception in its warning."""
        entry_points = _FakeEntryPoints([_FakeEntryPoint('BogusReport', _RaisingReport)])
        with mock.patch('ros2doctor.api.metadata.entry_points',
                        return_value=entry_points), \
                warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter('always')
            generate_reports()
            messages = [str(w.message) for w in caught]
            self.assertTrue(
                any('Fail to call BogusReport class functions' in m and
                    'boom-report' in m for m in messages),
                f'underlying exception missing from warnings: {messages}')


if __name__ == '__main__':
    unittest.main()
