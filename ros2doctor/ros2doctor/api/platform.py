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
import sys

import rosdistro


def print_platform_info():
    """Print out platform information."""
    platform_name = platform.system()
    # platform info
    print('PLATFORM INFORMATION')
    print('system               : ', platform_name)
    print('Platform Info        : ', platform.platform())
    if platform_name == 'Darwin':
        print('Mac OS version       : ', platform.mac_ver())
    print('release              : ', platform.release())
    print('processor            : ', platform.processor())

    # python info
    print('PYTHON INFORMATION')
    print('version              : ', platform.python_version())
    print('compiler             : ', platform.python_compiler())
    print('build                : ', platform.python_build())


def check_platform_helper():
    """Check ROS_DISTRO related environment variables and distribution name."""
    distro_name = os.environ.get('ROS_DISTRO')
    if not distro_name:
        sys.stderr.write('WARNING: ROS_DISTRO is not set.\n')
        return
    else:
        distro_name = distro_name.lower()
    u = rosdistro.get_index_url()
    if not u:
        sys.stderr.write('WARNING: Unable to access ROSDISTRO_INDEX_URL '
                         'or DEFAULT_INDEX_URL.\n')
        return
    i = rosdistro.get_index(u)
    distro_info = i.distributions.get(distro_name)
    if not distro_info:
        sys.stderr.write("WARNING: Distribution name '%s' is not found\n" % distro_name)
        return
    distro_data = rosdistro.get_distribution(i, distro_name).get_data()
    return distro_name, distro_info, distro_data


def print_ros2_info():
    """Print out ROS2 distribution info using `rosdistro`."""
    distros = check_platform_helper()
    if not distros:
        return
    distro_name, distro_info, distro_data = distros

    print('ROS INFORMATION')
    print('distribution name    : ', distro_name)
    print('distribution type    : ', distro_info.get('distribution_type'))
    print('distribution status  : ', distro_info.get('distribution_status'))
    print('release platforms    : ', distro_data.get('release_platforms'))


def check_platform():
    """Check platform information against ROS 2 requirements."""
    passed = False
    distros = check_platform_helper()
    if not distros:
        return passed
    _, distro_info, _ = distros

    # check distro status
    if distro_info.get('distribution_status') == 'prerelease':
        sys.stderr.write('WARNING: Distribution is not fully supported or tested. '
                         'To get more stable features, download a stable version at '
                         'https://index.ros.org/doc/ros2/Installation/\n')
    elif distro_info.get('distribution_status') == 'end-of-life':
        sys.stderr.write('WARNING: Distribution is no longer supported or deprecated. '
                         'To get the latest features, download the latest version at '
                         'https://index.ros.org/doc/ros2/Installation/')
    else:
        passed = True
    return passed
