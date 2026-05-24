#!/usr/bin/env bash
set -u

REPO_ID="Nahuyiur/light_condition"
LOCAL_FILE="/home/pnp/Desktop/franka_record_data/light_condition.zip"
PATH_IN_REPO="light_condition.zip"
LOG_DIR="/home/pnp/frankateleop/logs/modelscope_upload_light_condition"
STATE_DIR="/home/pnp/frankateleop/logs/modelscope_upload_light_condition/state"
PID_FILE="$STATE_DIR/supervisor.pid"
STATUS_FILE="$STATE_DIR/status.txt"
LATEST_LOG_FILE="$STATE_DIR/latest_log.txt"
UPLOAD_PATTERN="modelscope upload ${REPO_ID} ${LOCAL_FILE}"

mkdir -p "$LOG_DIR" "$STATE_DIR"

if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
  echo "Supervisor already running: $(cat "$PID_FILE")"
  exit 0
fi

echo "$$" > "$PID_FILE"

write_status() {
  printf '%s %s\n' "$(date '+%F %T')" "$*" | tee -a "$STATUS_FILE"
}

upload_finished_in_log() {
  local log_file="$1"
  [[ -n "$log_file" && -f "$log_file" ]] && grep -q "Finished uploading to ${REPO_ID}" "$log_file"
}

verify_remote_file() {
  REPO_ID="$REPO_ID" LOCAL_FILE="$LOCAL_FILE" PATH_IN_REPO="$PATH_IN_REPO" python - <<'PY'
import os
import sys
from modelscope.hub.api import HubApi

repo_id = os.environ["REPO_ID"]
local_file = os.environ["LOCAL_FILE"]
path_in_repo = os.environ["PATH_IN_REPO"]
local_size = os.path.getsize(local_file)

api = HubApi()
remote_size = None
page = 1
while True:
    files = api.get_dataset_files(
        repo_id,
        recursive=True,
        page_number=page,
        page_size=100,
    )
    if not files:
        break
    for item in files:
        if item.get("Type") == "blob" and (item.get("Path") or item.get("Name")) == path_in_repo:
            remote_size = item.get("Size")
            break
    if remote_size is not None or len(files) < 100:
        break
    page += 1

print(f"local_size={local_size} remote_size={remote_size}")
sys.exit(0 if remote_size == local_size else 1)
PY
}

find_running_upload_pid() {
  pgrep -af "$UPLOAD_PATTERN" \
    | awk -v self="$$" '$1 != self && $0 !~ /supervisor/ { print $1; exit }'
}

cleanup() {
  write_status "supervisor exiting"
  rm -f "$PID_FILE"
}
trap cleanup EXIT

attempt=0
write_status "supervisor started for ${REPO_ID}"

while true; do
  existing_pid="$(find_running_upload_pid || true)"
  if [[ -n "$existing_pid" ]]; then
    write_status "monitoring existing upload pid=${existing_pid}"
    while kill -0 "$existing_pid" 2>/dev/null; do
      sleep 60
    done
  fi

  attempt=$((attempt + 1))
  log_file="$LOG_DIR/light_condition_$(date +%Y%m%d_%H%M%S)_attempt${attempt}.log"
  echo "$log_file" > "$LATEST_LOG_FILE"
  write_status "starting upload attempt=${attempt}; log=${log_file}"

  stdbuf -oL -eL modelscope upload "$REPO_ID" "$LOCAL_FILE" "$PATH_IN_REPO" \
    --repo-type dataset \
    --commit-message "Upload light_condition dataset" \
    > "$log_file" 2>&1
  rc=$?

  if [[ "$rc" -eq 0 ]] && upload_finished_in_log "$log_file"; then
    if verify_output="$(verify_remote_file 2>&1)"; then
      write_status "upload completed and verified; attempt=${attempt}; ${verify_output}; log=${log_file}"
      exit 0
    fi
    write_status "upload attempt=${attempt} reported success but verification failed: ${verify_output}; retrying"
  fi

  sleep_seconds=$((attempt * 60))
  if [[ "$sleep_seconds" -gt 600 ]]; then
    sleep_seconds=600
  fi
  write_status "upload attempt=${attempt} failed rc=${rc}; retrying after ${sleep_seconds}s"
  sleep "$sleep_seconds"
done
