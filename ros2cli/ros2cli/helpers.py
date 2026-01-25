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
import shutil
import subprocess
import sys
import time

from typing import Dict
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


# Module-level flag to ensure the discovery warning is only shown once per process
_discovery_warning_shown = False


def check_discovery_configuration():
    """
    Check for invalid ROS discovery configuration and print warning if needed.

    Warns when ROS_AUTOMATIC_DISCOVERY_RANGE=OFF is set without ROS_STATIC_PEERS,
    which results in no discovery mechanism being available.

    The warning is only shown once per process to avoid duplicate warnings
    when multiple nodes are created.
    """
    global _discovery_warning_shown

    # Skip if warning has already been shown
    if _discovery_warning_shown:
        return

    discovery_range = os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '')
    static_peers = os.environ.get('ROS_STATIC_PEERS', '')

    if discovery_range == 'OFF' and not static_peers.strip():
        print(
            'Warning: ROS_AUTOMATIC_DISCOVERY_RANGE=OFF with no ROS_STATIC_PEERS configured.\n'
            'No discovery mechanism is available. Results will be empty.\n'
            'Either:\n'
            '  - Set ROS_STATIC_PEERS to specify peers explicitly, or\n'
            '  - Change ROS_AUTOMATIC_DISCOVERY_RANGE to LOCALHOST or SUBNET',
            file=sys.stderr
        )
        _discovery_warning_shown = True


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
    items: list[str],
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

    # Check if we're in an interactive terminal
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print('Error: Interactive selection requires a TTY terminal.', file=sys.stderr)
        return None

    # Check if fzf is available
    if shutil.which('fzf') is None:
        print(
            'Error: fzf is not installed...',
            file=sys.stderr
        )
        return None

    try:
        # Launch fzf with items as input
        process = subprocess.Popen(
            ['fzf', '--prompt', prompt + ' ', '--height', '40%', '--reverse'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True
        )

        try:
            # Send items to fzf
            stdout, _ = process.communicate(input='\n'.join(items))
        except KeyboardInterrupt:
            # Handle Ctrl+C gracefully to avoid leaving terminal in bad state
            process.terminate()
            process.wait()
            # Reset terminal to normal mode after fzf interruption
            subprocess.run(['stty', 'sane'], check=False)
            return None
        finally:
            # Ensure terminal is restored even if an exception occurs
            if process.poll() is None:
                process.terminate()
                process.wait()
            subprocess.run(['stty', 'sane'], check=False)

        # Check if user cancelled (Ctrl-C or ESC)
        if process.returncode != 0:
            return None

        # Return selected item (strip newline)
        selected = stdout.strip()
        return selected if selected else None

    except (OSError, subprocess.SubprocessError) as e:
        print(f'Error during interactive selection: {e}', file=sys.stderr)
        return None
