# Resolve the FR3 repo root so Python can import robot_servers from any cwd.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# Source the setup.bash file for the second ROS workspace
source /home/franka/catkin_ws/devel/setup.bash

# Ensure the FR3 repo is importable for `python -m robot_servers.franka_server`.
export PYTHONPATH="${REPO_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

# Change the ROS master URI to a different port
export ROS_MASTER_URI=http://localhost:11511

# Run the second instance of franka_server.py in the background
python -m robot_servers.franka_server \
    --robot_ip=172.16.0.3 \
    --ros_port=11511 \
    --gripper_type=Franka \
    --flask_url=127.0.0.2 \
    --flask_port=5000 \
    --side=right
