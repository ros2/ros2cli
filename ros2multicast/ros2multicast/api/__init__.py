# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import socket
import struct

DEFAULT_GROUP = '225.0.0.1'
DEFAULT_PORT = 49150


def send(data, *, group=DEFAULT_GROUP, port=DEFAULT_PORT, ttl=None):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    if ttl is not None:
        packed_ttl = struct.pack('b', ttl)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, packed_ttl)
    try:
        s.sendto(data, (group, port))
    finally:
        s.close()


def receive(*, group=DEFAULT_GROUP, port=DEFAULT_PORT, timeout=None):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            # not available on Windows
            pass
        s.bind(('', port))

        s.settimeout(timeout)

        mreq = struct.pack('4sl', socket.inet_aton(group), socket.INADDR_ANY)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        try:
            data, sender_addr = s.recvfrom(4096)
        except socket.timeout:
            if socket.timeout == TimeoutError:
                # Python >= 3.10
                raise
            else:
                # Python < 3.10
                raise TimeoutError
        finally:
            s.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
    finally:
        s.close()
    return data, sender_addr
