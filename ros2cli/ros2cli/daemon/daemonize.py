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

import copyreg
import os
import pickle
import platform
import socket
import subprocess
import sys
import threading

from ros2cli.helpers import wait_for


class PicklerForProcess(pickle.Pickler):

    def __init__(self, process, *args, **kwargs):
        super().__init__(process.stdin, *args, **kwargs)
        self.process = process
        self.dispatch_table = copyreg.dispatch_table.copy()
        self.dispatch_table[socket.socket] = self.reduce_socket
        self.dispatch_table[threading.Event] = self.reduce_event

    def reduce_event(self, obj):
        return threading.Event, ()

    @staticmethod
    def load_socket(data):
        if platform.system() == 'Windows':
            return socket.socket.fromshare(data)
        return socket.socket(fileno=data)

    def reduce_socket(self, obj):
        if platform.system() == 'Windows':
            data = obj.share(self.process.pid)
        else:
            data = obj.fileno()
        return PicklerForProcess.load_socket, (data,)

    def dump(self, *args, **kwargs):
        super().dump(*args, **kwargs)
        self.process.stdin.flush()


def main():
    func = pickle.load(sys.stdin.buffer)
    sys.stdin.close()
    os.close(0)  # force C stream close
    return func()


def daemonize(func, tags={}, timeout=None, debug=False):
    prog = f'from {__name__} import main; main()'
    cmd = [sys.executable, '-c', prog]
    for name, value in tags.items():
        flag = '--' + name.replace('_', '-')
        cmd += [flag, str(value)]
    kwargs = {}
    if platform.system() == 'Windows':
        # Process Creation Flag documented in the MSDN
        DETACHED_PROCESS = 0x00000008  # noqa: N806
        kwargs.update(creationflags=DETACHED_PROCESS)
        # avoid showing cmd windows for subprocess
        si = subprocess.STARTUPINFO()
        si.dwFlags = subprocess.STARTF_USESHOWWINDOW
        si.wShowWindow = subprocess.SW_HIDE
        kwargs['startupinfo'] = si
        # don't keep handle of current working directory in daemon process
        kwargs.update(cwd=os.environ.get('SYSTEMROOT', None))

    kwargs['stdin'] = subprocess.PIPE
    if not debug:
        kwargs['stdout'] = subprocess.DEVNULL
        kwargs['stderr'] = subprocess.DEVNULL
    kwargs['close_fds'] = False

    process = subprocess.Popen(cmd, **kwargs)

    pickler = PicklerForProcess(process)

    pickler.dump(func)

    if timeout is not None:
        def daemon_ready():
            try:
                pickler.dump(None)
                return False
            except OSError:
                return True
        if not wait_for(daemon_ready, timeout):
            process.terminate()
            raise RuntimeError(
                'Timed out waiting for '
                'daemon to become ready'
            )
    if process.poll() is not None:
        rc = process.returncode
        raise RuntimeError(
            'Daemon process died '
            f'with returncode {rc}'
        )
    return
