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

import netifaces
import platform
import socket
import sys


def _is_unix_like_platform():
    """Return True is conforms to UNIX/POSIX-style APIs."""
    return platform.system() in ['Linux', 'FreeBSD', 'Darwin']


def _use_ipv6():
    """Check if ROS uses IPV6"""
    ipv6 = os.environ.get('ROS_IPV6')
    return ipv6 and ipv6 == 'on'


def get_local_addresses():
    """Fetch a list of local IP addresses."""
    local_addrs = []
    if _is_unix_like_platform():
        ipv4_addrs = []
        ipv6_addrs = []
        for i in netifaces.interfaces():
            addrs = netifaces.ifaddresses(i)
            if socket.AF_INET in addrs:
                v4addrs.extend([addr['addr'] for addr in addrs[socket.AF_INET]])
            if socket.AF_INET6 in addrs:
                v6addrs.extend([addr['addr'] for addr in addrs[socket.AF_INET6]])
        if _use_ipv6():
            local_addrs = ipv4_addrs + ipv6_addrs
        else:
            local_addrs = ipv4_addrs
    else:
        # can only resolve one address on other platforms?
        if _use_ipv6():
            local_addrs = [host[4][0] for host in socket.getaddrinfo(socket.gethostname(),\
                                0, 0, 0, socket.SOL_TCP)]
        else:
            local_addrs = [host[4][0] for host in socket.getaddrinfo(socket.gethostname(),\
                                0, socket.AF_INET, 0, socket.SOL_TCP)]
    return local_addrs


def check_sys_ips():
    """Compare roslib's routine against socket resolution to see if they agree."""
    #TODO figure out the significance of this check
    pass

# def check_ros_hostname():
#     """Check if ROS_HOSTNAME resolves to a local IP address."""
#     hostname = 
#     pass

def check_ros_ip():
    """Check if ROS_IP is a local IP address."""

    pass


# Error / Warning Rules

warnings = {}

errors = {}


def check_network():
    """Conduct network checks and output error/warning messages."""

    pass


def print_network():
    """Print all system and ROS network information."""

    pass
