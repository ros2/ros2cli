# Copyright 2026 Open Source Robotics Foundation, Inc.
# Copyright 2026 Tanneguy de Villemagne
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

import sys
from typing import List, Tuple
import io


def run_tf_tree(argv: List[str]) -> Tuple[int, str]:
    """Call upstream `tf_tree_terminal.tree_node.main()` while capturing stdout.

    Returns a tuple (exit_code, captured_output).
    """
    try:
        from tf_tree_terminal.tree_node import main as tf_main
    except Exception as e:  # pragma: no cover - import environment dependent
        return (1, f"[ros2tftree] tf_tree_terminal not available: {e}\n")

    # set sys.argv for the delegated parser and capture stdout/stderr
    saved_argv = sys.argv[:]
    saved_stdout = sys.stdout
    saved_stderr = sys.stderr
    sys_stdout_buf = io.StringIO()
    sys_stderr_buf = io.StringIO()
    try:
        sys.argv = [saved_argv[0]] + argv
        sys.stdout = sys_stdout_buf
        sys.stderr = sys_stderr_buf
        try:
            tf_main()
            exit_code = 0
        except SystemExit:
            exit_code = 0
        except Exception as e:
            exit_code = 2
            print(f"[ros2tftree] tf-tree execution failed: {e}", file=sys.stderr)
        out = sys_stdout_buf.getvalue() + sys_stderr_buf.getvalue()
        return (exit_code, out)
    finally:
        sys.argv = saved_argv
        sys.stdout = saved_stdout
        sys.stderr = saved_stderr

def run_tf_tree_live(argv: List[str]) -> int:
    """Run the upstream tf-tree command without capturing stdout/stderr.

    Use this when preserving exact output ordering is important (e.g. `show`).
    Returns numeric exit code.
    """
    try:
        from tf_tree_terminal.tree_node import main as tf_main
    except Exception as e:  # pragma: no cover - environment dependent
        print(f"[ros2tftree] tf_tree_terminal not available: {e}")
        return 1

    saved_argv = sys.argv[:]
    try:
        sys.argv = [saved_argv[0]] + argv
        try:
            tf_main()
            return 0
        except SystemExit:
            return 0
        except Exception as e:
            print(f"[ros2tftree] tf-tree execution failed: {e}")
            return 2
    finally:
        sys.argv = saved_argv

