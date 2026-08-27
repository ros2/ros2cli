# Copyright 2017-2018 Open Source Robotics Foundation, Inc.
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

autoload -U +X compinit && compinit
autoload -U +X bashcompinit && bashcompinit

# Get this scripts directory
__ros2cli_completion_dir=${0:a:h}
# Just source the bash version, it works in zsh too
source "$__ros2cli_completion_dir/ros2-argcomplete.bash"
# Cleanup
unset __ros2cli_completion_dir

# argcomplete's zsh completion function hands the candidates to _describe,
# which quotes shell metacharacters when inserting a match into the command
# line. A leading '~' is one of them, so `ros2 bag info ~/foo<TAB>` ends up as
# `\~/foo`, which zsh no longer expands to the home directory.
# See https://github.com/ros2/ros2cli/issues/1279 and
# https://github.com/kislyuk/argcomplete/issues/503.
#
# This function does the same as argcomplete's, except that a leading '~' or
# '~user' is moved out of the part being completed (the "ignored prefix"), so
# it is left as typed while the rest of the path is still quoted as needed.
_ros2_argcomplete() {
  local IFS=$'\013'
  local -a completions
  completions=($(IFS="$IFS" \
      COMP_LINE="$BUFFER" \
      COMP_POINT="$CURSOR" \
      _ARGCOMPLETE=1 \
      _ARGCOMPLETE_SHELL="zsh" \
      _ARGCOMPLETE_SUPPRESS_SPACE=1 \
      __python_argcomplete_run ${words[1]}))
  local -a nosort nospace
  autoload -Uz is-at-least
  if is-at-least 5.8; then
    nosort=(-o nosort)
  fi
  # Do not append a space after a match ending in a continuation character.
  if [[ "${completions-}" =~ ([^\\\\]): && "${match[1]}" =~ [=/:] ]]; then
    nospace=(-S '')
  fi
  if [[ "$PREFIX" == '~'* ]]; then
    local tilde="${PREFIX%%/*}"
    local -a stripped
    local completion
    for completion in "${completions[@]}"; do
      if [[ "${completion[1,${#tilde}]}" == "$tilde" ]]; then
        stripped+=("${completion[${#tilde}+1,-1]}")
      else
        stripped+=("$completion")
      fi
    done
    completions=("${stripped[@]}")
    compset -P "${(b)tilde}"
  fi
  _describe "${words[1]}" completions "${nosort[@]}" "${nospace[@]}"
}
if (( ${+functions[__python_argcomplete_run]} )); then
  compdef _ros2_argcomplete ros2
fi
