if [ -z "$BASH_VERSION" ]; then
  return 0 2>/dev/null || exit 0
fi

ros2() {
  local list_cmd=""
  local fzf_prompt=""
  local preview_cmd=""

  case "$*" in
    "topic echo" | "topic info" | "topic hz" | \
    "topic bw"   | "topic pub"  | "topic type" | \
    "topic delay")
      list_cmd="ros2 topic list"
      fzf_prompt="topic> "
      preview_cmd="ros2 topic info {} 2>/dev/null"
      ;;
    "service call" | "service type" | "service find")
      list_cmd="ros2 service list"
      fzf_prompt="service> "
      preview_cmd="ros2 service type {} 2>/dev/null"
      ;;
    "node info")
      list_cmd="ros2 node list"
      fzf_prompt="node> "
      preview_cmd="ros2 node info {} 2>/dev/null"
      ;;
    "param get"  | "param set"  | "param list" | \
    "param describe" | "param delete" | \
    "param dump" | "param load")
      list_cmd="ros2 node list"
      fzf_prompt="node> "
      preview_cmd="ros2 node info {} 2>/dev/null"
      ;;
    *)
      command ros2 "$@"
      return $?
      ;;
  esac

  if ! command -v fzf &> /dev/null; then
    echo "fzf is not installed. Install with: sudo apt install fzf" >&2
    command ros2 "$@"
    return $?
  fi

  local selected
  selected=$(
    command $list_cmd 2>/dev/null | fzf \
      --height 40% --reverse --border \
      --prompt "$fzf_prompt" \
      --preview "$preview_cmd" \
      --preview-window "right:50%:wrap"
  )

  if [ -z "$selected" ]; then
    command ros2 "$@"
    return $?
  fi

  printf '\e[A\r\e[2K'

  local prefilled="ros2 $* ${selected} "
  local full_cmd
  read -e -p "${PS1@P}" -i "$prefilled" full_cmd

  if [ -z "$full_cmd" ]; then
    return 0
  fi

  history -s -- "$full_cmd"
  eval -- "$full_cmd"
}
