# Copyright 2017-2026 Open Source Robotics Foundation, Inc.
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

# Register ros2 tab-completion for fish shell via argcomplete.
if type -q register-python-argcomplete
    register-python-argcomplete --shell fish ros2 | source
else if type -q register-python-argcomplete3
    register-python-argcomplete3 --shell fish ros2 | source
end
