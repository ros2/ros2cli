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
import platform

import netifaces
import rosdistro


def print_sys_info():
    """Print out platform information."""
    # platform info
    print('System Information')
    print('system   : ', platform.system())
    print('Linux distribution   : ', platform.dist())
    print('Mac OS version       : ', platform.mac_ver())
    print('release  : ', platform.release())
    print('processor: ', platform.processor())
    print()

    # python info
    print('Python')
    print('version      : ', platform.python_version())
    print('compiler     : ', platform.python_compiler())
    print('build        : ', platform.python_build())
    print()


def print_ros2_reqs():
    """Print out ROS2 distribution info using `rosdistro`."""
    distro_name = os.environ.get('ROS_DISTRO').lower()
    u = rosdistro.get_index_url()
    i = rosdistro.get_index(u)
    distro_info = i.distributions.get(distro_name)
    distro_data = rosdistro.get_distribution(i, distro_name).get_data()

    print('ROS Information')
    print('distribution name    : ', distro_name)
    print('distribution type    : ', distro_info.get('distribution_type'))
    print('distribution status  : ', distro_info.get('distribution_status'))
    print('release platforms    : ', distro_data.get('release_platforms'))
    print()


def setup_checks():
    """Check platform information against ROS2 requirements."""
    distro_name = os.environ.get('ROS_DISTRO').lower()
    u = rosdistro.get_index_url()
    i = rosdistro.get_index(u)
    distro_info = i.distributions.get(distro_name)
    distro_data = rosdistro.get_distribution(i, distro_name).get_data()

    # check distro status
    if distro_info.get('distribution_status') == 'prerelease':
        print('WARNING: Distribution is not fully supported or tested.\
            To get more stable features,\
                Download a stable version at\
                    https://index.ros.org/doc/ros2/Installation/')
    elif distro_info.get('distribution_status') == 'end-of-life':
        print('WARNING: Distribution is no longer supported or deprecated.\
            To get the latest features,\
                Download the latest version at\
                    https://index.ros.org/doc/ros2/Installation/')
    else:
        pass

    # check system platform and distro release platforms
    supported_platforms = distro_data.get('release_platforms')
    if platform.system() == 'Linux':
        releases = supported_platforms.get(platform.dist()[0].lower())
        if not releases or platform.dist()[2].lower() not in releases:
            print('WARNING: Current system platform is not supported by this ROS distribution.\
                User `ros2 debug setup -r` to check report for detail.')
        else:
            pass
    else:
        print('WARNING:\
            Limited information on platform requirements on Windows and OSX\
                are available for auto-check.\
                    Use `ros2 debug setup -r` for more detail.')


def print_network_interface():
    """Print out network interface."""
    ids = netifaces.interfaces()
    for i in ids:
        addrs = netifaces.ifaddresses(i)
        link = addrs[netifaces.AF_LINK]  # link layer interface eg.Ethernet
        internet = addrs[netifaces.AF_INET]  # normal internet addresses
        ipv6 = addrs[netifaces.AF_INET6]  # IPv6
        print(link)
        print(internet)
        print(ipv6)
    pass
