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
import socket

import psutil

from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2doctor.api.format import doctor_error
from ros2doctor.api.format import doctor_warn


class InterfaceFlags:
    def __init__(self, interface_name):
        self.flags = ''
        self.has_loopback = False
        self.has_non_loopback = False
        self.has_multicast = False

        all_stats = psutil.net_if_stats()
        if interface_name not in all_stats:
            return

        interface_stats = all_stats[interface_name]

        self.flags = interface_stats.flags

        if 'loopback' in interface_stats.flags:
            self.has_loopback = True
        else:
            self.has_non_loopback = True

        if 'multicast' in interface_stats.flags:
            self.has_multicast = True

    def __str__(self):
        if not self.flags:
            return

        return self.flags.upper()


class NetworkCheck(DoctorCheck):
    """Check network interface configuration for loopback and multicast."""

    def category(self):
        return 'network'

    def check(self):
        """Check network configuration."""
        result = Result()

        has_loopback = False
        has_non_loopback = False
        has_multicast = False
        for interface in psutil.net_if_addrs().keys():
            flags = InterfaceFlags(interface)
            has_loopback |= flags.has_loopback
            has_non_loopback |= flags.has_non_loopback
            has_multicast |= flags.has_multicast

        if os.name != 'posix':
            if not has_loopback and not has_non_loopback:
                # no flags found, otherwise one of them should be True.
                doctor_warn(
                    'Interface flags are not available on Windows. '
                    'Run `ipconfig` to see what interfaces are available.')
                result.add_warning()
                return result
        if not has_loopback:
            doctor_error('No loopback IP address is found.')
            result.add_error()
        if not has_non_loopback:
            doctor_warn('Only loopback IP address is found.')
            result.add_warning()
        if not has_multicast:
            doctor_warn('No multicast IP address is found.')
            result.add_warning()
        return result


class NetworkReport(DoctorReport):
    """Report network configuration."""

    def category(self):
        return 'network'

    def report(self):
        """Print system and ROS network information."""
        if_stats = psutil.net_if_stats()

        network_report = Report('NETWORK CONFIGURATION')

        for interface, addrs in psutil.net_if_addrs().items():
            if_info = {
                'inet': None,
                'inet4': [],
                'ether': None,
                'inet6': [],
                'netmask': None,
                'device': interface,
                'flags': None,
                'mtu': None,
                'broadcast': None,
            }
            for addr in addrs:
                if addr.family == socket.AddressFamily.AF_INET:
                    if_info['inet'] = addr.address
                    if_info['inet4'].append(addr.address)
                    if_info['netmask'] = addr.netmask
                    if_info['broadcast'] = addr.broadcast
                elif addr.family == socket.AddressFamily.AF_INET6:
                    if_info['inet6'].append(addr.address)
                elif 'AF_PACKET' in socket.AddressFamily.__members__:
                    if addr.family == socket.AddressFamily.AF_PACKET:
                        # Loopback reports an all-zero MAC address, which is
                        # bogus and should not be reported
                        if addr.address != '00:00:00:00:00:00':
                            if_info['ether'] = addr.address

            if interface in if_stats:
                if_info['mtu'] = if_stats[interface].mtu

            flags = InterfaceFlags(interface)
            if_info['flags'] = str(flags)

            for k, v in if_info.items():
                if v:
                    network_report.add_to_report(k, v)
        return network_report
