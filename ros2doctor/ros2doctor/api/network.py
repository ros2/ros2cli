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
import sys

try:
    import ifcfg
except ImportError:
    sys.stderr.write('WARNING: No ifcfg module found. '
                     'Unable to run network check and report. '
                     'Use `python -m pip install ifcfg` to install needed package.\n')

from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report


def _is_unix_like_platform():
    """Return True if conforms to UNIX/POSIX-style APIs."""
    return os.name == 'posix'


def _check_network_config_helper():
    """Check if loopback and multicast IP addresses are found."""
    has_loopback, has_non_loopback, has_multicast = False, False, False
    # temp fix for ifcfg package, maunually pass network check
    if 'ifcfg' not in sys.modules:
        print('WARNING: ifcfg package not imported. Skip network check...\n')
        return True, True, True

    for name, iface in ifcfg.interfaces().items():
        flags = iface.get('flags')
        if 'LOOPBACK' in flags:
            has_loopback = True
        else:
            has_non_loopback = True
        if 'MULTICAST' in flags:
            has_multicast = True
    return has_loopback, has_non_loopback, has_multicast


class NetworkCheck(DoctorCheck):
    """Check network interface configuration for loopback and multicast."""

    def category(self):
        return 'network'

    def check(self):
        """Conduct network checks and output error/warning messages."""
        has_loopback, has_non_loopback, has_multicast = _check_network_config_helper()
        if not has_loopback:
            sys.stderr.write('ERROR: No loopback IP address is found.\n')
        if not has_non_loopback:
            sys.stderr.write('WARNING: Only loopback IP address is found.\n')
        if not has_multicast:
            sys.stderr.write('WARNING: No multicast IP address is found.\n')
        return has_loopback and has_non_loopback and has_multicast


class NetworkReport(DoctorReport):
    """Report network interface configuration."""

    def category(self):
        return 'network'

    def report(self):
        """Print all system and ROS network information."""
        network_report = Report('NETWORK CONFIGURATION')
        # temp fix for ifcfg package, return none for report
        if 'ifcfg' not in sys.modules:
            print('ERROR: ifcfg package not imported. Skipping network report...\n')
            return None

        for name, iface in ifcfg.interfaces().items():
            for k, v in iface.items():
                if v:
                    network_report.add_to_report(k, v)
        return network_report
