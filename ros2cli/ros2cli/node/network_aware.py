# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import functools
import inspect

import psutil
import rclpy

from ros2cli.node.direct import DirectNode


def get_interfaces_ip_addresses():
    addresses_by_interfaces = psutil.net_if_addrs()
    print(f'Addresses by interfaces: {addresses_by_interfaces}')
    return addresses_by_interfaces


class NetworkAwareNode:
    """A direct node, that resets itself when a network interface changes."""

    def __init__(self, args):
        self.args = args
        # TODO(ivanpauno): A race condition is possible here, since it isn't possible to know
        # exactly which interfaces were available at node creation.
        self.node = DirectNode(args)
        self.addresses_at_start = get_interfaces_ip_addresses()

    def __enter__(self):
        self.node.__enter__()
        return self

    def __getattr__(self, name):
        attr = getattr(self.node, name)

        if inspect.ismethod(attr):
            @functools.wraps(attr)
            def wrapper(*args, **kwargs):
                self.reset_if_addresses_changed()
                # The attribute has to be get here again, in case self.node changed
                return getattr(self.node, name)(*args, **kwargs)
            wrapper.__signature__ = inspect.signature(attr)
            return wrapper
        self.reset_if_addresses_changed()
        return attr

    def __exit__(self, exc_type, exc_value, traceback):
        self.node.__exit__(exc_type, exc_value, traceback)

    def reset_if_addresses_changed(self):
        new_addresses = get_interfaces_ip_addresses()
        if new_addresses != self.addresses_at_start:
            self.addresses_at_start = new_addresses
            self.node.destroy_node()
            rclpy.shutdown()
            self.node = DirectNode(self.args)
            self.node.__enter__()
            print('Network interfaces changed, daemon node was reset!')
