#!/usr/bin/env bash
set -u

REPO_ID="${REPO_ID:-Nahuyiur/pick_sth_in_drawer}"
LOCAL_DIR="${LOCAL_DIR:-/home/pnp/Desktop/pick_sth_in_drawer}"
LOG_DIR="${LOG_DIR:-/home/pnp/frankateleop/logs/modelscope_upload}"
STATE_DIR="${STATE_DIR:-$LOG_DIR/state}"
PID_FILE="$STATE_DIR/supervisor.pid"
STATUS_FILE="$STATE_DIR/status.txt"
LATEST_LOG_FILE="$STATE_DIR/latest_log.txt"
MAX_WORKERS="${MAX_WORKERS:-4}"
UPLOAD_PATTERN="modelscope upload ${REPO_ID} ${LOCAL_DIR}"

mkdir -p "$LOG_DIR" "$STATE_DIR"

if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
  echo "Supervisor already running: $(cat "$PID_FILE")"
  exit 0
fi

echo "$$" > "$PID_FILE"

write_status() {
  printf '%s %s\n' "$(date '+%F %T')" "$*" | tee -a "$STATUS_FILE"
}

latest_upload_log() {
  ls -t "$LOG_DIR"/pick_sth_in_drawer_*.log 2>/dev/null | head -1
}

upload_finished_in_log() {
  local log_file="$1"
  [[ -n "$log_file" && -f "$log_file" ]] && grep -q "Finished uploading to ${REPO_ID}" "$log_file"
}

verify_remote_count() {
  REPO_ID="$REPO_ID" LOCAL_DIR="$LOCAL_DIR" python - <<'PY'
import os
import sys
from modelscope.hub.api import HubApi

repo_id = os.environ["REPO_ID"]
local_dir = os.environ["LOCAL_DIR"]
local_count = 0
for _, _, files in os.walk(local_dir):
    local_count += sum(1 for name in files if name != ".ms_upload_cache")

api = HubApi()
remote_paths = set()
page = 1
page_size = 100
while True:
    files = api.get_dataset_files(
        repo_id,
        recursive=True,
        page_number=page,
        page_size=page_size,
    )
    if not files:
        break
    for item in files:
        if item.get("Type") == "blob":
            path = item.get("Path") or item.get("Name")
            if path not in {".gitattributes", "README.md"}:
                remote_paths.add(path)
    if len(files) < page_size:
        break
    page += 1

print(f"local_files={local_count} remote_data_files={len(remote_paths)}")
sys.exit(0 if len(remote_paths) >= local_count else 1)
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

    log_file="$(latest_upload_log || true)"
    if upload_finished_in_log "$log_file"; then
      echo "$log_file" > "$LATEST_LOG_FILE"
      if verify_output="$(verify_remote_count 2>&1)"; then
        write_status "upload completed and verified via existing process; ${verify_output}; log=${log_file}"
        exit 0
      fi
      write_status "existing upload reported success but verification failed: ${verify_output}; retrying"
      sleep 60
      continue
    fi

    write_status "existing upload stopped before success; retrying"
  fi

  attempt=$((attempt + 1))
  log_file="$LOG_DIR/pick_sth_in_drawer_$(date +%Y%m%d_%H%M%S)_attempt${attempt}.log"
  echo "$log_file" > "$LATEST_LOG_FILE"
  write_status "starting upload attempt=${attempt}; max_workers=${MAX_WORKERS}; log=${log_file}"

  stdbuf -oL -eL modelscope upload "$REPO_ID" "$LOCAL_DIR" \
    --repo-type dataset \
    --commit-message "Upload pick_sth_in_drawer dataset" \
    --max-workers "$MAX_WORKERS" \
    > "$log_file" 2>&1
  rc=$?

  if [[ "$rc" -eq 0 ]] && upload_finished_in_log "$log_file"; then
    if verify_output="$(verify_remote_count 2>&1)"; then
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
