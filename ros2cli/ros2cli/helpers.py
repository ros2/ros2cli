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

from argparse import ArgumentTypeError
import functools
import inspect
import os
import subprocess
import sys
import time

from typing import Dict
from typing import List
from typing import Optional


def get_ros_domain_id():
    return int(os.environ.get('ROS_DOMAIN_ID', 0))


def wait_for(predicate, timeout, period=0.1):
    """
    Wait for a predicate to evaluate to `True`.

    :param timeout: duration, in seconds, to wait
      for the predicate to evaluate to `True`.
      Non-positive durations will result in an
      indefinite wait.
    :param period: predicate evaluation period,
      in seconds.
    :return: predicate result
    """
    if timeout < 0:
        timeout = float('+inf')
    deadline = time.time() + timeout
    while not predicate():
        if time.time() > deadline:
            return predicate()
        time.sleep(period)
    return True


def bind(func, *args, **kwargs):
    """
    Bind a function with a set of arguments.

    A functools.partial equivalent that is actually a function.
    """
    partial = functools.partial(func, *args, **kwargs)

    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        return partial(*args, **kwargs)
    wrapper.__signature__ = inspect.signature(func)
    return wrapper


def pretty_print_call(func, *args, **kwargs):
    """
    Print a function invocation.

    See `before_invocation` for usage as a hook.
    """
    name = func.__name__
    arguments = ', '.join(
        [f'{v!r}' for v in args] +
        [f'{k}={v!r}' for k, v in kwargs.items()]
    )
    print(f'{name}({arguments})')


def before_invocation(func, hook):
    """
    Invoke a `hook` before every `func` invocation.

    `hook` may take no arguments or take the `func`
    and arbitrary positional and keyword arguments.
    """
    signature = inspect.signature(hook)
    nargs = len(signature.parameters)
    if inspect.ismethod(hook):
        nargs = nargs - 1
    if nargs > 0:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            hook(func, *args, **kwargs)
            return func(*args, **kwargs)
    else:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            hook()
            return func(*args, **kwargs)
    wrapper.__signature__ = inspect.signature(func)
    return wrapper


def unsigned_int(string):
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value < 0:
        raise ArgumentTypeError('value must be non-negative integer')
    return value


def collect_stdin():
    lines = b''
    while True:
        line = sys.stdin.buffer.readline()
        if not line:
            break
        lines += line
    return lines


def get_rmw_additional_env(rmw_implementation: str) -> Dict[str, str]:
    """Get a dictionary of additional environment variables based on rmw."""
    if rmw_implementation == 'rmw_zenoh_cpp':
        return {
            'RMW_IMPLEMENTATION': rmw_implementation,
            'RUST_LOG': 'z=error'
        }
    else:
        return {
            'RMW_IMPLEMENTATION': rmw_implementation,
        }


def interactive_select(
    items: List[str],
    prompt: str = 'Select an item:'
) -> Optional[str]:
    """
    Launch interactive fuzzy search using fzf to select from a list of items.

    :param items: List of items to select from
    :param prompt: Prompt message to display in fzf
    :return: Selected item or None if user cancelled or fzf not available
    """
    if not items:
        print('No items available to select from.', file=sys.stderr)
        return None

    try:
        # Check if fzf is available
        result = subprocess.run(
            ['fzf', '--version'],
            capture_output=True,
            text=True,
            timeout=1
        )
        if result.returncode != 0:
            raise FileNotFoundError()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print(
            'Error: fzf is not installed but is a dependency for this package. You can install it with rosdep',
            file=sys.stderr
        )
        return None

    try:
        # Launch fzf with items as input - using direct TTY access
        process = subprocess.Popen(
            ['fzf', '--prompt', prompt + ' ', '--height', '40%', '--reverse'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True
        )

        # Send items to fzf
        stdout, _ = process.communicate(input='\n'.join(items))

        # Check if user cancelled (Ctrl-C or ESC)
        if process.returncode != 0:
            return None

        # Return selected item (strip newline)
        selected = stdout.strip()
        return selected if selected else None

    except Exception as e:
        print(f'Error during interactive selection: {e}', file=sys.stderr)
        return None

