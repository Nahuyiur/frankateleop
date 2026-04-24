#!/bin/bash
# Franky Server Startup Script
# This script starts the Franky HTTP server with proper configuration

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="/tmp/franky_server.log"
PID_FILE="/tmp/franky_server.pid"
PYTHON_BIN="${PYTHON_BIN:-/home/ubuntu/robot_env/bin/python}"
ROBOT_IP="${ROBOT_IP:-172.16.0.2}"

# Kill existing server if running
if [ -f "$PID_FILE" ]; then
    OLD_PID=$(cat "$PID_FILE")
    if ps -p "$OLD_PID" > /dev/null 2>&1; then
        echo "Killing existing server (PID: $OLD_PID)"
        kill "$OLD_PID" 2>/dev/null || true
        sleep 1
    fi
    rm -f "$PID_FILE"
fi

# Start server
echo "Starting Franky server..."
cd "$SCRIPT_DIR"
nohup "$PYTHON_BIN" server.py > "$LOG_FILE" 2>&1 < /dev/null &
SERVER_PID=$!
echo $SERVER_PID > "$PID_FILE"

# Wait for server to start
sleep 2

# Check if server is running
if ! ps -p "$SERVER_PID" > /dev/null 2>&1; then
    echo "ERROR: Server failed to start. Check log: $LOG_FILE"
    tail -20 "$LOG_FILE"
    exit 1
fi

# Check health endpoint
if curl -s --connect-timeout 2 http://127.0.0.1:5000/health > /dev/null 2>&1; then
    curl -s -X POST http://127.0.0.1:5000/connect \
        -H 'Content-Type: application/json' \
        -d "{\"ip\":\"${ROBOT_IP}\",\"motion_async\":true,\"recover\":true}" > /dev/null 2>&1 || true

    echo "✓ Franky server started successfully (PID: $SERVER_PID)"
    echo "  Log file: $LOG_FILE"
    echo "  Health check: http://127.0.0.1:5000/health"
    echo "  Python: $PYTHON_BIN"
    echo "  Robot IP: $ROBOT_IP"

    # Show current status
    echo ""
    echo "Current server status:"
    curl -s http://127.0.0.1:5000/health | python -m json.tool 2>/dev/null || curl -s http://127.0.0.1:5000/health
else
    echo "ERROR: Server started but health check failed"
    exit 1
fi
