# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from inspect import isclass

import rclpy.qos
from rpyutils import add_dll_directories_from_env

# Since Python 3.8, on Windows we should ensure DLL directories are explicitly added
# to the search path.
# See https://docs.python.org/3/whatsnew/3.8.html#bpo-36085-whatsnew
with add_dll_directories_from_env('PATH'):
    from netperf_tool.netperf_tool_impl import ClientRunner as _ClientRunner
    from netperf_tool.netperf_tool_impl import ServerRunner as _ServerRunner


class _RunnerImpl:
    """Simple wrapper of a netperf_tool::NodeRunner<NodeT> template instance or derived class."""

    _impl_cls = None
    """
    The netperf_tool::NodeRunner<NodeT> template instance being wrapped.
    """

    _BLOCK_FOREVER = -1.0
    """
    Duration used to indicate the runner to block forever.
    Any other negative number work as well.
    """

    def __init__(
        self,
        experiment_duration: float,
        qos: rclpy.qos.QoSProfile,
        *args
    ):
        """
        Construct.

        :param experiment_duration: Total time to be spinning the node.
            A negative number can be used to spin for ever, e.g. `cls._BLOCK_FOREVER`.
        :param qos: Qos profile used to create the publisher or subscription.
        :param *args: Other arguments required to construct the node.
        """
        assert self._impl_cls is not None and isclass(self._impl_cls)
        self._impl = self._impl_cls(qos.get_c_qos_profile(), *args)
        self._experiment_duration = experiment_duration

    def __enter__(self):
        """
        Enter context manager.

        Asynchronously spinning is started and a reference to the
        spinning node is returned.
        """
        self._impl.start(self._experiment_duration)
        return self._impl.get_node()

    def __exit__(self, _, _1, _2):
        """
        Exit context manager.

        Signal the spinning thread to stop and wait until the thread stops running.
        """
        self._impl.stop()
        self._impl.join()

    def stop(self):
        """Signal the spinning thread to stop."""
        self._impl.stop()

    def wait_for_experiment_to_complete(self):
        """Wait until experiment duration expires."""
        self._impl.join()


class ClientRunner(_RunnerImpl):
    """Simple wrapper of a netperf_tool::ClientRunner."""

    _impl_cls = _ClientRunner

    def __init__(
        self,
        *,
        experiment_duration_s: float,
        qos: rclpy.qos.QoSProfile,
        message_size_bytes: int,
        target_pub_period_s: float,
        server_timeout_s: float,
    ):
        """
        Construct.

        :param experiment_duration_s: Total time to be spinning.
        :param qos: Qos profile to be used when creating the publisher.
        :param message_size_bytes: Size of the message to be used.
            The total serialized message size will be a bit bigger, because of some extra metadata
            included to calculate latencies and identify messages.
        :param server_timeout_s: Timeout used to wait for a netperf server.
        """
        super().__init__(
            experiment_duration_s, qos, message_size_bytes, target_pub_period_s, server_timeout_s)


class ServerRunner(_RunnerImpl):
    """Simple wrapper of a netperf_tool::NodeRunner<ServerNode>."""

    _impl_cls = _ServerRunner

    def __init__(
        self,
        *,
        qos: rclpy.qos.QoSProfile
    ):
        """
        Construct.

        :param qos: Qos profile to be used when creating the subscription.
        """
        super().__init__(self._BLOCK_FOREVER, qos)


__all__ = [
    'ClientRunner',
    'ServerRunner',
]
