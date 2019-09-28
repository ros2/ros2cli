# reduced from ament_package/template/package_level/local_setup.fish.in

# provide AMENT_CURRENT_PREFIX to shell script
set -l AMENT_CURRENT_PREFIX (cd (dirname (status -f))/../.. && pwd)
# store AMENT_CURRENT_PREFIX to restore it before each environment hook
set -l _package_local_setup_AMENT_CURRENT_PREFIX $AMENT_CURRENT_PREFIX

# unset AMENT_ENVIRONMENT_HOOKS
# if not appending to them for return
if [ -z "$AMENT_RETURN_ENVIRONMENT_HOOKS"]
  set -e AMENT_ENVIRONMENT_HOOKS
end

# restore AMENT_CURRENT_PREFIX before evaluating the environment hooks
set -l AMENT_CURRENT_PREFIX $_package_local_setup_AMENT_CURRENT_PREFIX
# list all environment hooks of this package
ament_append_value AMENT_ENVIRONMENT_HOOKS "$AMENT_CURRENT_PREFIX/share/ros2cli/environment/ros2-argcomplete.fish"
# source all shell-specific environment hooks of this package
# if not returning them
if [ -z "$AMENT_RETURN_ENVIRONMENT_HOOKS"]
  set -l _package_local_setup_IFS $IFS
  set -l IFS ":"
  for _hook in $AMENT_ENVIRONMENT_HOOKS
    # restore AMENT_CURRENT_PREFIX for each environment hook
    set AMENT_CURRENT_PREFIX $_package_local_setup_AMENT_CURRENT_PREFIX
    # restore IFS before sourcing other files
    set IFS $_package_local_setup_IFS
    source "$_hook"
  end
  set IFS $_package_local_setup_IFS
end
