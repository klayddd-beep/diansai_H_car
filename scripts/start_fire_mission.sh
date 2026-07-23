#!/usr/bin/env bash
# Desktop-autostart entry point for the complete fire-car ROS mission.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
workspace_root=$(cd -- "$script_dir/.." && pwd -P)
gpio_init="$script_dir/init_start_button_gpio.sh"
gpio_value=/sys/class/gpio/gpio35/value

if [[ "${1:-}" == --gpio-only ]]; then
  if [[ $EUID -ne 0 ]]; then
    echo "--gpio-only must run as root" >&2
    exit 2
  fi
  exec "$gpio_init"
fi

# Do not rely on ~/.bashrc: desktop autostart normally does not read it. Use a
# car-specific override name so an accidentally inherited ROS_DOMAIN_ID cannot
# move the mission into another DDS domain.
car_ros_domain_id="${CAR_ROS_DOMAIN_ID:-6}"
if [[ ! "$car_ros_domain_id" =~ ^[0-9]+$ ]] ||
   (( 10#$car_ros_domain_id > 232 )); then
  echo "invalid CAR_ROS_DOMAIN_ID: $car_ros_domain_id (expected 0..232)" >&2
  exit 2
fi
export ROS_DOMAIN_ID="$((10#$car_ros_domain_id))"
export ROS_LOCALHOST_ONLY=0

state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/diansai-fire"
mkdir -p "$state_dir/ros"
log_file="$state_dir/fire-mission.log"
# Keep one bounded previous log without requiring a system log service.
if [[ -f "$log_file" ]] && (( $(stat -c %s "$log_file") > 10485760 )); then
  mv -f -- "$log_file" "$log_file.previous"
fi
exec > >(tee -a "$log_file") 2>&1
export ROS_LOG_DIR="$state_dir/ros"

exec 9>"$state_dir/launch.lock"
if ! flock -n 9; then
  echo "$(date --iso-8601=seconds) fire mission is already running"
  exit 0
fi
printf '%s\n' "$$" > "$state_dir/launch.pid"

echo "$(date --iso-8601=seconds) starting fire mission from: $workspace_root"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"

# The physical button needs root once per boot. A root boot hook may call this
# file with --gpio-only. When passwordless sudo is already configured, this
# desktop entry also initializes it itself; otherwise the touch button remains
# available and the warning is preserved in the fixed log.
if [[ ! -r "$gpio_value" ]]; then
  if [[ $EUID -eq 0 ]]; then
    "$gpio_init"
  elif ! sudo -n "$gpio_init" 2>/dev/null; then
    echo "WARNING: $gpio_value is unavailable; configure the root boot hook:" >&2
    echo "  $workspace_root/scripts/start_fire_mission.sh --gpio-only" >&2
  fi
fi

# A desktop autostart normally supplies these. The fallbacks cover common
# LightDM/GDM X11 sessions when the launcher provides a minimal environment.
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
display_number="${DISPLAY#:}"
display_number="${display_number%%.*}"
if [[ -z "${XAUTHORITY:-}" ]]; then
  candidates=(
    "$HOME/.Xauthority"
    "$XDG_RUNTIME_DIR/gdm/Xauthority"
    "$XDG_RUNTIME_DIR/.mutter-Xwaylandauth.*"
  )
  for candidate_pattern in "${candidates[@]}"; do
    for candidate in $candidate_pattern; do
      if [[ -r "$candidate" ]]; then
        export XAUTHORITY="$candidate"
        break 2
      fi
    done
  done
fi

for _ in {1..60}; do
  [[ -S "/tmp/.X11-unix/X${display_number}" ]] && break
  sleep 1
done
if [[ ! -S "/tmp/.X11-unix/X${display_number}" ]]; then
  echo "X11 display was not ready after 60s (DISPLAY=$DISPLAY)" >&2
  exit 1
fi

# XFCE autostart can run while X11 is still using its 1024x768 fallback.
# Wait for a usable geometry to remain stable before the dashboard queries
# XRandR. The dashboard also tracks later mode changes (for slow HDMI links).
stable_geometry=
stable_geometry_count=0
for _ in {1..30}; do
  xrandr_header=$(xrandr --current 2>/dev/null | head -n 1 || true)
  if [[ "$xrandr_header" =~ current[[:space:]]+([0-9]+)[[:space:]]+x[[:space:]]+([0-9]+) ]]; then
    screen_width="${BASH_REMATCH[1]}"
    screen_height="${BASH_REMATCH[2]}"
    geometry="${screen_width}x${screen_height}"
    if (( screen_width >= 1280 && screen_height >= 720 )); then
      if [[ "$geometry" == "$stable_geometry" ]]; then
        ((stable_geometry_count += 1))
      else
        stable_geometry="$geometry"
        stable_geometry_count=1
      fi
      if (( stable_geometry_count >= 3 )); then
        echo "X11 display geometry is stable: $geometry"
        break
      fi
    else
      stable_geometry=
      stable_geometry_count=0
    fi
  fi
  sleep 1
done
if (( stable_geometry_count < 3 )); then
  echo "WARNING: X11 display geometry did not stabilize; dashboard will track changes at runtime" >&2
fi

if [[ ! -r "$workspace_root/install/setup.bash" ]]; then
  echo "workspace has not been built: $workspace_root/install/setup.bash" >&2
  exit 1
fi

# ROS 2 environment hooks are not guaranteed to be nounset-safe.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$workspace_root/install/setup.bash"
set -u
cd "$workspace_root"
exec ros2 launch car_launch fire_mission.launch.py
