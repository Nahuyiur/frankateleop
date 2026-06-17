#!/bin/bash
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"

# Keep the single-left-arm launcher on the same Robotiq path as
# left_franka/2_launch_gripper.sh; only the gripper server port differs.
export LEFT_GRIPPER_SERVER_PORT="${LEFT_GRIPPER_SERVER_PORT:-50052}"
exec bash "$REPO_ROOT/left_franka/2_launch_gripper.sh" "$@"
