#!/usr/bin/env bash
set -euo pipefail

state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/diansai-fire"
pid_file="$state_dir/launch.pid"
if [[ ! -r "$pid_file" ]]; then
  echo "fire mission is not running (no $pid_file)"
  exit 0
fi

pid=$(<"$pid_file")
if [[ ! "$pid" =~ ^[0-9]+$ ]] || [[ ! -r "/proc/$pid/cmdline" ]]; then
  echo "fire mission is not running (stale PID file)"
  rm -f -- "$pid_file"
  exit 0
fi
command_line=$(tr '\0' ' ' < "/proc/$pid/cmdline")
if [[ "$command_line" != *"ros2 launch car_launch fire_mission.launch.py"* ]]; then
  echo "refusing to signal unrelated PID $pid: $command_line" >&2
  exit 1
fi

kill -INT "$pid"
for _ in {1..100}; do
  if ! kill -0 "$pid" 2>/dev/null; then
    rm -f -- "$pid_file"
    echo "fire mission stopped"
    exit 0
  fi
  sleep 0.1
done
echo "fire mission did not stop within 10 seconds (PID $pid)" >&2
exit 1
