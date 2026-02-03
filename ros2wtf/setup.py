# Copyright 2024 Open Source Robotics Foundation, Inc.
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

from setuptools import find_packages, setup

package_name = 'ros2wtf'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nivesh Dandyan',
    maintainer_email='niveshdandyan@users.noreply.github.com',
    description='What The Failure - Debug helper for ROS 2 systems',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'wtf = ros2wtf.command.wtf:WtfCommand',
        ],
        'ros2cli.extension_point': [
            'ros2wtf.verb = ros2wtf.verb:VerbExtension',
        ],
        'ros2wtf.verb': [
            'analyze = ros2wtf.verb.wtf:WtfVerb',
        ],
    },
)
