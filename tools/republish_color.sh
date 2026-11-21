#!/usr/bin/env bash
# Republish the suite's compressed color camera topics as raw
# sensor_msgs/Image so RViz2's Image display can select them
# (the picker filters out CompressedImage).
#
# Usage:
#   ./tools/republish_color.sh                     # all five cameras
#   ./tools/republish_color.sh front sideLeft      # only the named cameras
#
# Stop with Ctrl+C — all instances are cleaned up.
# Notes:
# - Full-name remapping is required; image_transport republish does not
#   accept the `in:=` short syntax.
# - One process per camera; with all five running, mind the JPEG decode
#   CPU cost (about 50 color frames/s in total — on a weak machine start
#   only the cameras you actually view).

set -u

ALL_CAMS="front sideLeft sideRight rearLeft rearRight"
if [ $# -eq 0 ]; then
  set -- $ALL_CAMS
fi

# ROS + workspace environments (idempotent, skip if missing)
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -f "$HOME/colcon_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/colcon_ws/install/setup.bash"
fi

pids=""
for cam in "$@"; do
  echo "republish: /ac/camera/$cam/image_raw/compressed -> /ac/camera/$cam/image_raw"
  ros2 run image_transport republish compressed raw --ros-args \
    -r /in/compressed:=/ac/camera/$cam/image_raw/compressed \
    -r /out:=/ac/camera/$cam/image_raw \
    -r __node:=republish_"$cam" &
  pids="$pids $!"
done

cleanup() {
  echo ""
  echo "stopping all republish instances..."
  # shellcheck disable=SC2086
  kill $pids 2>/dev/null
  # shellcheck disable=SC2086
  wait $pids 2>/dev/null
  echo "done."
}
trap cleanup INT TERM

wait
