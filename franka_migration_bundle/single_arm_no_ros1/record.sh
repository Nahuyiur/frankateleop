#!/bin/bash

# 1. Start the Robot Server in the background
# Replace the path below with your actual server start command if different
bash /home/ubuntu/FR3/franky_servers/start_server.sh &
SERVER_PID=$!
echo "Server started with PID: $SERVER_PID"

# Wait a few seconds for the server to initialize hardware and ports
sleep 3

# 2. Setup variables
ID=$1   # Episode Number from command line argument
TASK="test"
OUTPUT_DIR="/home/ubuntu/FR3/record_data"

# 3. Start Teleoperation in the background
# This allows the recording script to have the foreground for keyboard inputs
~/robot_env/bin/python /home/ubuntu/FR3/teleop/run.py &
TELEOP_PID=$!
echo "Teleop started with PID: $TELEOP_PID"

# 4. Start Recording in the foreground
# This script manages the OpenCV window; press 'q' in the window to finish
~/robot_env/bin/python /home/ubuntu/FR3/scripts/record_single.py \
    --task "${TASK}" \
    --output_root "${OUTPUT_DIR}" \
    --index "${ID}"

# 5. Cleanup: Kill the background processes after record_single.py exits
echo "Cleaning up processes..."
kill $TELEOP_PID
kill $SERVER_PID

# Optional: Force kill if they don't exit gracefully
sleep 1
kill -9 $TELEOP_PID $SERVER_PID 2>/dev/null

echo "Processes terminated. Data saved for index ${ID}."