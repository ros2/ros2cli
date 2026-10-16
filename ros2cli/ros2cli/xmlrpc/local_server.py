# Copyright 2020 Open Source Robotics Foundation, Inc.
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
import os
import socket
# Import SimpleXMLRPCRequestHandler to re-export it.
from xmlrpc.server import SimpleXMLRPCRequestHandler  # noqa
from xmlrpc.server import SimpleXMLRPCServer

import psutil


def get_local_ipaddrs():
    return [
        addr.address
        for _, addrs in psutil.net_if_addrs().items()
        for addr in addrs
        if addr.family == socket.AF_INET
    ]


def is_local_address(address, local_ipaddrs):
    # Loopback is always local, but not every environment lists it among the interface
    # addresses (e.g. Windows containers), which would make the daemon drop every client.
    return ipaddress.ip_address(address).is_loopback or address in local_ipaddrs


class LocalXMLRPCServer(SimpleXMLRPCServer):

    # Allow re-binding even if another server instance was recently bound (i.e. we are still in
    # TCP TIME_WAIT). This is already the default behavior on Windows, and further SO_REUSEADDR can
    # lead to undefined behavior on Windows; see
    # https://learn.microsoft.com/en-us/windows/win32/winsock/using-so-reuseaddr-and-so-exclusiveaddruse.  # noqa
    # So we don't set the option for Windows.
    allow_reuse_address = False if os.name == 'nt' else True

    def verify_request(self, request, client_address):
        if not is_local_address(client_address[0], get_local_ipaddrs()):
            return False
        return super(LocalXMLRPCServer, self).verify_request(request, client_address)
