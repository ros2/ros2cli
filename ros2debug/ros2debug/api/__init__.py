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


# Installation system requirements of ROS 2
ROS2_REQS = {'crystal': {'linux': {'version': 'Ubuntu',
                                   'release': ['16.04', '18.04'], 
                                   'processor': 'x86_64'},
                        'darwin': {'version': '',
                                   'release': ['15.0.0', '15.6.0',
                                               '16.0.0', '16.5.0',
                                               '16.6.0'],
                                   'processor': ''},
                        'windows': {'version': '10',
                                    'release': [],
                                    'processor': 'AMD64'}
                        },
             'dashing': {'linux': {'version': 'Ubuntu',
                                  'release': '18.04',
                                  'processor': 'x86_64'},
                        'darwin': {'version': '',
                                   'release': ['14.0.0', '14.5.0', '15.0.0',
                                               '15.6.0', '16.0.0', '16.5.0',
                                               '16.6.0', '17.0.0', '17.5.0',
                                               '17.6.0', '17.7.0'],
                                   'processor': ''},
                        'windows': {'version': '10',
                                    'release': [],
                                    'processor': 'AMD64'}
                        }
            }

def print_sys_info():
    # platform info
    print('System Information')
    print('system   :', platform.system())
    print('release  :', platform.release())
    print('version  :', platform.version())
    print('processor:', platform.processor())
    print('normal   :', platform.platform())
    print()

    # python info 
    print('Python')
    print('version      :', platform.python_version())
    print('compiler     :', platform.python_compiler())
    print('build        :', platform.python_build())
    print()


def print_ros2_reqs(*, ros2_reqs=ROS2_REQS):
    distro = os.environ.get('ROS_DISTRO').lower()
    system_name = platform.system()
    system_req = ros2_reqs.get(distro).get(system_name.lower())

    print('ROS2 System Requirements')
    print('ROS2 distribution:', distro)
    print('system   :', system_name)
    print('version  :', system_req.get('version'))
    print('release  :', system_req.get('release'))
    print('machine  :', system_req.get('processor'))
    print('----------------------------------------------------')


def setup_checks(*, ros2_reqs=ROS2_REQS):
    distro_check = True
    os_check = True
    version_check = True
    release_check = True
    processor_check = True

    distro = os.environ.get('ROS_DISTRO').lower()
    system_name = platform.system()
    distro_reqs = ros2_reqs.get(distro)

    if distro_reqs:
        system_reqs = distro_reqs.get(system_name.lower())
    else:
        distro_check = False
        system_reqs = distro_reqs.get('crystal').get(system_name.lower())
    
    if system_reqs:        
        sys_version = system_reqs.get('version')
        sys_release = system_reqs.get('release')
        sys_processor = system_reqs.get('processor')
        if sys_version and sys_version not in platform.version():
            version_check = False
        if sys_release:
            if system_name == 'Darwin' and platform.release() not in sys_release:
                release_check = False
            elif system_name == 'Linux' and platform.dist()[1] not in sys_release:
                processor_check = False
        if sys_processor and platform.machine() not in sys_processor:
            processor_check = False
    else:
        os_check = False
    return distro_check, os_check, version_check, release_check, processor_check


def print_warning_msg(distro, os, version, release, processor):
    os_type = platform.system()
    if distro == False:
        print("WARNING: Current ROS2 distribution %s is no longer supported.\
                Checking other requirements using Crystal's standard" \
                    % os.environ.get('ROS_DISTRO'))
    if os == False: 
        print('WARNING: Current OS %s is not fully supported by our tests.' % os_type)
    if os_type == 'Linux':
        if version == False:
            print('WARNING: Current OS version is not fully supported by ROS2.\
                    Check report for detail.')
        if release == False:
            print('WARNING: Current OS release is not fully supported by ROS2.\
                    Check report for detail.')
        if processor == False: 
            print('WARNING: Current processor is not supported by ROS2.\
                    Check report for detail.')
    elif os_type == 'Darwin':
        if release == False:
            print('WARNING: Current OS release is not fully supported by ROS2.\
                    Check report for detail.')
    elif os_type == 'Windows':
        if version == False:
            print('WARNING: Current OS version is not fully supported by ROS2.\
                    Check report for detail.')
        if processor == False:
            print('WARNING: Current processor is not supported by ROS2.\
                    Check report for detail.')
