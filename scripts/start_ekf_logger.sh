#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Build this workspace first; see $workspace_dir/README.md" >&2
  exit 1
fi
source "$workspace_dir/install/setup.bash"
exec ros2 launch here4_logger ekf_logger.launch.py "$@"
