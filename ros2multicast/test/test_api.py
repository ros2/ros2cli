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

import threading
import time

from ros2multicast.api import receive
from ros2multicast.api import send


def test_api():
    sent_data = b'test_api'
    received_data = None

    def target():
        nonlocal received_data
        received_data, _ = receive(timeout=1.0)

    t = threading.Thread(target=target)
    t.start()
    time.sleep(0.1)
    send(sent_data)
    t.join()
    assert sent_data == received_data
