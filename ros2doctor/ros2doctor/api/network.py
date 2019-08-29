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
    sys.stderr.write('ERROR: No ifcfg module found. '
                     'Install with `python -m pip install ifcfg`.')


def _is_unix_like_platform():
    """Return True if conforms to UNIX/POSIX-style APIs."""
    return os.name == 'posix'


def check_network_config_helper():
    """Check if loopback and multicast IP addresses are found."""
    has_loopback, has_non_loopback, has_multicast = False, False, False
    for name, iface in ifcfg.interfaces().items():
        flags = iface.get('flags')
        if 'LOOPBACK' in flags:
            has_loopback = True
        else:
            has_non_loopback = True
        if 'MULTICAST' in flags:
            has_multicast = True
    return has_loopback, has_non_loopback, has_multicast


def check_network_config():
    """Conduct network checks and output error/warning messages."""
    has_loopback, has_non_loopback, has_multicast = check_network_config_helper()
    if not has_loopback:
        sys.stderr.write('ERROR: No loopback IP address is found.\n')
    if not has_non_loopback:
        sys.stderr.write('WARNING: Only loopback IP address is found.\n')
    if not has_multicast:
        sys.stderr.write('WARNING: No multicast IP address is found.\n')
    return has_loopback and has_non_loopback and has_multicast


def report_network():
    """Print all system and ROS network information."""
    network_info = [('NAME', 'NETWORK CONFIGURATION')]
    for name, iface in ifcfg.interfaces().items():
        for k, v in iface.items():
            if v:
                network_info.append((k, v))
    return network_info
