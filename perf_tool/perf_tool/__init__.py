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
from rpyutils import add_dll_directories_from_env

# Since Python 3.8, on Windows we should ensure DLL directories are explicitly added
# to the search path.
# See https://docs.python.org/3/whatsnew/3.8.html#bpo-36085-whatsnew
with add_dll_directories_from_env('PATH'):
    from perf_tool.perf_tool_impl import (
        ClientRunner as _ClientRunner,
        ServerRunner as _ServerRunner,
        ClientResults,
        ServerResults,
    )


class RunnerImpl:
    _impl_cls = None

    def __init__(
        self,
        *args
    ):
        if self._impl_cls is None or not isclass(self._impl_cls):
            raise RuntimeError('internal error')
        self._impl = self._impl_cls()
        self._start_args = args

    def __enter__(self):
        self._impl.start(*self._start_args)
        return self

    def stop(self):
        self._impl.stop()

    def get_results(self):
        return self._impl.get_results()

    def wait_for_experiment_to_complete(self):
        self._impl.join()

    def __exit__(self, type, value, traceback):
        self._impl.stop()
        self._impl.join()


class ClientRunner(RunnerImpl):
    _impl_cls = _ClientRunner


class ServerRunner(RunnerImpl):
    _impl_cls = _ServerRunner


__all__ = [
    'ClientResults',
    'ClientRunner',
    'ServerResults',
    'ServerRunner',
]
