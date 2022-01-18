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
import struct
import sys

import psutil

from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2doctor.api.format import doctor_error
from ros2doctor.api.format import doctor_warn


class InterfaceFlags:
    POSIX_NET_FLAGS = (
        ('UP', 0),
        ('BROADCAST', 1),
        ('DEBUG', 2),
        ('LOOPBACK', 3),

        ('PTP', 4),
        ('NOTRAILERS', 5),
        ('RUNNING', 6),
        ('NOARP', 7),

        ('PROMISC', 8),
        ('ALLMULTI', 9),
        ('MASTER', 10),
        ('SLAVE', 11),

        ('MULTICAST', 12),
        ('PORTSEL', 13),
        ('AUTOMEDIA', 14),
        ('DYNAMIC', 15),
    )

    def __init__(self, interface_name):
        self.flags = -1
        self.flag_list = set()
        self.has_loopback = False
        self.has_non_loopback = False
        self.has_multicast = False
        self.get(interface_name)

    def get(self, interface_name):
        if os.name != 'posix':
            return

        import fcntl

        if sys.platform == 'darwin':
            SIOCGIFFLAGS = 0xc0206911
        else:
            SIOCGIFFLAGS = 0x8913

        # We need to pass a 'struct ifreq' to the SIOCGIFFLAGS ioctl.  Nominally the
        # structure looks like:
        #
        #            struct ifreq {
        #               char ifr_name[IFNAMSIZ]; /* Interface name */
        #               union {
        #                   struct sockaddr ifr_addr;
        #                   struct sockaddr ifr_dstaddr;
        #                   struct sockaddr ifr_broadaddr;
        #                   struct sockaddr ifr_netmask;
        #                   struct sockaddr ifr_hwaddr;
        #                   short           ifr_flags;
        #                   int             ifr_ifindex;
        #                   int             ifr_metric;
        #                   int             ifr_mtu;
        #                   struct ifmap    ifr_map;
        #                   char            ifr_slave[IFNAMSIZ];
        #                   char            ifr_newname[IFNAMSIZ];
        #                   char           *ifr_data;
        #               };
        #           };
        #
        # Where IFNAMSIZ is 16 bytes long.  The caller (that's us) sets the ifr_name
        # to the interface in question, and the call fills in the union.  In the case
        # of this particular ioctl, only the 'ifr_flags' is valid after the call.
        # Either way, we need to provide enough room in the provided structure for the
        # ioctl to fill it out, so we just provide an additional 256 bytes which should
        # be more than enough.

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            try:
                result = fcntl.ioctl(s.fileno(), SIOCGIFFLAGS, interface_name + '\0' * 256)
            except OSError:
                return

        # On success, result contains the structure filled in with the info we asked
        # for (the flags).  We just need to extract it from the correct place.

        self.flags = struct.unpack('H', result[16:18])[0]
        for flagname, bit in self.POSIX_NET_FLAGS:
            if self.flags & (1 << bit):
                self.flag_list.add(flagname)

        if 'LOOPBACK' in self.flag_list:
            self.has_loopback = True
        else:
            self.has_non_loopback = True

        if 'MULTICAST' in self.flag_list:
            self.has_multicast = True

    def __str__(self):
        if self.flags == -1:
            return ''

        output_flags = ','.join(self.flag_list)

        return f'{self.flags}<{output_flags}>'


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
