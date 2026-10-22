#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# The convenient wrapper uses the commissioned physical bench by default.
# Explicit simulate:=true selects simulated motors without elevation.
simulate=false
for argument in "$@"; do
  case "$argument" in
    simulate:=true) simulate=true ;;
    simulate:=false) simulate=false ;;
    simulate:=*) echo "simulate must be true or false" >&2; exit 2 ;;
  esac
done
if [[ "$simulate" == false && "$EUID" -ne 0 ]]; then
  exec sudo --preserve-env=DISPLAY,XAUTHORITY,ROS_DOMAIN_ID bash "$workspace_dir/scripts/run_system_test.sh" "$@"
fi
source "$workspace_dir/scripts/ros2_env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-44}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
if [[ "$simulate" == true ]]; then
  echo "FORGE: simulated motors; real sensors and Pocket controller."
  defaults=(simulate:=true execute:=false mapping_confirmed:=false radio_failsafe_confirmed:=false)
else
  echo "FORGE: ACTUAL MOTORS — commissioned three-motor bench, Pocket right stick."
  defaults=(simulate:=false execute:=true mapping_confirmed:=true radio_failsafe_confirmed:=true)
fi
# Explicit arguments win, including disabling any authorization flag.
exec ros2 launch bringup system_test.launch.py "${defaults[@]}" "$@"
