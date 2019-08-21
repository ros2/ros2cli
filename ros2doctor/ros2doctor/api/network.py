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

import ipaddress
import netifaces
import os
import platform
import socket
import sys


def _is_unix_like_platform():
    """Return True is conforms to UNIX/POSIX-style APIs."""
    return platform.system() in ['Linux', 'FreeBSD', 'Darwin']


def get_local_addresses():
    """Fetch a list of local IP addresses."""
    local_addrs = []
    if _is_unix_like_platform():
        ipv4_addrs = []
        ipv6_addrs = []
        for i in netifaces.interfaces():
            addrs = netifaces.ifaddresses(i)
            if socket.AF_INET in addrs:
                # get ipv4 addresses from all interfaces
                ipv4_addrs.extend([addr['addr'] for addr in addrs[socket.AF_INET]])
            if socket.AF_INET6 in addrs:
                # get ipv6 addresses from all interfaces
                ipv6_addrs.extend([addr['addr'] for addr in addrs[socket.AF_INET6]])
        if socket.has_ipv6:
            local_addrs = ipv4_addrs + ipv6_addrs
        else:
            local_addrs = ipv4_addrs
    else:
        # can only resolve one address on other platforms?
        if socket.has_ipv6:
            local_addrs = [host[4][0] for host in socket.getaddrinfo(socket.gethostname(),\
                                0, 0, 0, socket.SOL_TCP)]
        else:
            local_addrs = [host[4][0] for host in socket.getaddrinfo(socket.gethostname(),\
                                0, socket.AF_INET, 0, socket.SOL_TCP)]
    return local_addrs


def check_sys_ips(local_addrs):
    """Check if loopback and multicast IP addresses are present."""
    has_loopback = False
    has_others = False
    for addr in local_addrs:
        try:
            addr_obj = ipaddress.ip_address(addr)
        except ValueError:
            continue
        if addr_obj.is_loopback:
            has_loopback = True
        else:
            has_others = True
    return has_loopback, has_others


def check_network():
    """Conduct network checks and output error/warning messages."""
    result = False
    local_addrs = get_local_addresses()
    if not local_addrs:
        sys.stderr.write('ERROR: No local IP addresses found.\n')
        return result
    else:
        has_loopback, has_others = check_sys_ips(local_addrs)
        if not has_loopback:
            sys.stderr.write('ERROR: No loopback IP address is found.\n')
        if not has_others:
            sys.stderr.write('WARNING: Only loopback IP address is found.\n')
        if has_loopback and has_others:
            result = True
    return result


def print_network():
    """Print all system and ROS network information."""
    pass
