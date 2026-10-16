# Copyright 2026 Open Source Robotics Foundation, Inc.
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

from ros2cli.xmlrpc.local_server import is_local_address

# An environment whose interface list does not include loopback, e.g. a Windows container.
INTERFACE_ADDRESSES = ['172.28.91.15']


def test_loopback_is_local_even_if_not_listed():
    assert is_local_address('127.0.0.1', INTERFACE_ADDRESSES)


def test_listed_address_is_local():
    assert is_local_address('172.28.91.15', INTERFACE_ADDRESSES)


def test_unlisted_address_is_not_local():
    assert not is_local_address('203.0.113.7', INTERFACE_ADDRESSES)
