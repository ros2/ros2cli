if [ -z "$ZSH_VERSION" ]; then
  return 0
fi

__ros2_zle_last_offered=""

_ros2_zle_accept_line() {
  local trimmed="${BUFFER%"${BUFFER##*[![:space:]]}"}"
  local selected=""
  local list_cmd=""
  local fzf_prompt=""
  local preview_cmd=""

  case "$trimmed" in
    "ros2 topic echo" | \
    "ros2 topic info" | \
    "ros2 topic hz"   | \
    "ros2 topic bw"   | \
    "ros2 topic pub"  | \
    "ros2 topic type" | \
    "ros2 topic delay")
      list_cmd="ros2 topic list"
      fzf_prompt="topic> "
      preview_cmd="ros2 topic info {}"
      ;;
    "ros2 service call" | \
    "ros2 service type" | \
    "ros2 service find")
      list_cmd="ros2 service list"
      fzf_prompt="service> "
      preview_cmd="ros2 service type {}"
      ;;
    "ros2 node info")
      list_cmd="ros2 node list"
      fzf_prompt="node> "
      preview_cmd="ros2 node info {}"
      ;;
    "ros2 param get"      | \
    "ros2 param set"      | \
    "ros2 param list"     | \
    "ros2 param describe" | \
    "ros2 param delete"   | \
    "ros2 param dump"     | \
    "ros2 param load")
      list_cmd="ros2 node list"
      fzf_prompt="node> "
      preview_cmd="ros2 node info {}"
      ;;
    *)
      __ros2_zle_last_offered=""
      zle .accept-line
      return
      ;;
  esac

  if [[ "$trimmed" == "$__ros2_zle_last_offered" ]]; then
    __ros2_zle_last_offered=""
    zle .accept-line
    return
  fi

  if ! command -v fzf &> /dev/null; then
    zle -M "fzf is not installed. Install with: sudo apt install fzf"
    zle .accept-line
    return
  fi

  selected=$(
    ${=list_cmd} 2>/dev/null | fzf \
      --height 40% --reverse --border \
      --prompt "$fzf_prompt" \
      --preview "$preview_cmd 2>/dev/null" \
      --preview-window "right:50%:wrap"
  )

  if [[ -n "$selected" ]]; then
    __ros2_zle_last_offered=""
    BUFFER="${trimmed} ${selected} "
    CURSOR=${#BUFFER}
  else
    __ros2_zle_last_offered="$trimmed"
  fi

  zle reset-prompt
}

zle -N accept-line _ros2_zle_accept_line
