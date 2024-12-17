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

# Alias xmlrpc.client module objects to ensure client code uses ros2cli.xmlrpc
from xmlrpc.client import ProtocolError
from xmlrpc.client import ServerProxy
from xmlrpc.client import Transport


__all__ = [
    'ProtocolError',
    'ServerProxy',
    'TimeoutTransport',
]


class TimeoutTransport(Transport):

    def __init__(self, *args, timeout=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._timeout = timeout

    def make_connection(self, *args, **kwargs):
        connection = super().make_connection(*args, **kwargs)
        if self._timeout is not None:
            connection.timeout = self._timeout
        return connection
