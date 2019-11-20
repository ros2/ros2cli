# Copyright 2019 Open Source Robotics Foundation, Inc.
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
from typing import Tuple

from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2doctor.api.format import doctor_warn

try:
    import ifcfg
except ImportError:  # check import error for windows and osx
    doctor_warn('Failed to import ifcfg. '
                'Use `python -m pip install ifcfg` to install needed package.')


def _is_unix_like_platform() -> bool:
    """Return True if conforms to UNIX/POSIX-style APIs."""
    return os.name == 'posix'


def _check_network_config_helper(ifcfg_ifaces: dict) -> Tuple[bool, bool, bool]:
    """Check if loopback and multicast IP addresses are found."""
    has_loopback, has_non_loopback, has_multicast = False, False, False
    for iface in ifcfg_ifaces.values():
        flags = iface.get('flags')
        if flags:
            flags = flags.lower()
            if 'loopback' in flags:
                has_loopback = True
            else:
                has_non_loopback = True
            if 'multicast' in flags:
                has_multicast = True
    return has_loopback, has_non_loopback, has_multicast


class NetworkCheck(DoctorCheck):
    """Check network interface configuration for loopback and multicast."""

    def category(self):
        return 'network'

    def check(self):
        """Check network configuration."""
        result = Result()
        # check ifcfg import for windows and osx users
        try:
            ifcfg_ifaces = ifcfg.interfaces()
        except NameError:
            result.add_error('ERROR: ifcfg is not imported. Unable to run network check.')
            return result

        has_loopback, has_non_loopback, has_multicast = _check_network_config_helper(ifcfg_ifaces)
        if not _is_unix_like_platform():
            if not has_loopback and not has_non_loopback:
                # no flags found, otherwise one of them should be True.
                print('No flags found. \
                    Run `ipconfig` on cmd to check network interfaces.')
                return result
        if not has_loopback:
            result.add_error('ERROR: No loopback IP address is found.')
        if not has_non_loopback:
            result.add_warning('Only loopback IP address is found.')
        if not has_multicast:
            result.add_warning('No multicast IP address is found.')
        return result


class NetworkReport(DoctorReport):
    """Report network configuration."""

    def category(self):
        return 'network'

    def report(self):
        """Print system and ROS network information."""
        # check ifcfg import for windows and osx users
        try:
            ifcfg_ifaces = ifcfg.interfaces()
        except NameError:
            doctor_warn('ifcfg is not imported. Unable to generate network report.')
            return Report('')

        network_report = Report('NETWORK CONFIGURATION')
        for iface in ifcfg_ifaces.values():
            for k, v in iface.items():
                if v:
                    network_report.add_to_report(k, v)
        return network_report
