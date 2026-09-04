#!/usr/bin/env bash
set -euo pipefail

SCRIPT_UNDER_TEST="${1:?usage: test_bi_arm_stack_only_contract.sh /path/to/15_record_bi_arm_pipeline.sh}"
TEST_ROOT="$(mktemp -d /tmp/bi-arm-stack-contract.XXXXXX)"
trap 'rm -rf "$TEST_ROOT"' EXIT

# shellcheck source=/dev/null
source "$SCRIPT_UNDER_TEST"

# checker LD path contract
grep -Fq 'BI_ARM_CHECK_POLYMETIS_ROOT' "$SCRIPT_UNDER_TEST"
grep -Fq 'POLYMETIS_TORCH_ISOLATION' "$SCRIPT_UNDER_TEST"



grep -Fq 'umi_genrobot/bridge/das_finger_polymetis_server.py' "$SCRIPT_UNDER_TEST"
grep -Fq 'polymetis/polymetis/conf/robot_client/right_franka_hardware.yaml' "$SCRIPT_UNDER_TEST"
if grep -Eq '170->131|on 131|path on 131' "$SCRIPT_UNDER_TEST"; then
    echo "stale .131 host wording remains" >&2
    exit 1
fi

PREFLIGHT_COMMAND="$TEST_ROOT/preflight-command"
remote_bash() {
    printf '%s\n' "$1" > "$PREFLIGHT_COMMAND"
}
abort() {
    echo "unexpected abort: $*" >&2
    return 1
}
check_remote_right_runtime_prerequisites
grep -Fq "$RIGHT_REPO/umi_genrobot/bridge/das_finger_polymetis_server.py" "$PREFLIGHT_COMMAND"
grep -Fq "$RIGHT_REPO/umi_genrobot/vendor/gen_finger_con_python_sdk_release" "$PREFLIGHT_COMMAND"
grep -Fq '/dev/ttyFingerRight' "$PREFLIGHT_COMMAND"
grep -Fq 'FTBJTKP2-if00-port0' "$PREFLIGHT_COMMAND"

TRACE="$TEST_ROOT/main-trace"
MOUNT_RESULT=0
mkdir() { printf 'mkdir:%s\n' "$*" >> "$TRACE"; }
require_muka_nas_mounted() {
    printf 'mount-check:%s\n' "$1" >> "$TRACE"
    return "$MOUNT_RESULT"
}
setup_ssh_askpass() { :; }
provision_local_sudo_file() { :; }
prepare_sudo() { :; }
prepare_remote() { :; }
check_remote_right_fci_ready() { :; }
sync_remote_right_scripts() { :; }
check_remote_right_runtime_prerequisites() { :; }
cleanup_stale_ports() { :; }
kill_port_local() { :; }
kill_port_remote() { :; }
start_local_script() { :; }
start_remote_script() { :; }
start_tunnel() { :; }
check_local_script() { :; }
check_remote_script() { :; }
wait_for_external_recorder() { :; }
run_recording() { printf 'recording:%s\n' "$*" >> "$TRACE"; }
cleanup_all() { :; }
log() { :; }

STACK_ONLY=1
RIGHT_ONLY=1
main gui_right_stack --contract-test
if grep -Fq 'mount-check:' "$TRACE"; then
    echo "stack-only mode unexpectedly checked the NAS mount" >&2
    exit 1
fi
if grep -Fq "mkdir:-p $DEFAULT_OUTPUT_ROOT" "$TRACE"; then
    echo "stack-only mode unexpectedly created the recording output root" >&2
    exit 1
fi

: > "$TRACE"
STACK_ONLY=0
RIGHT_ONLY=0
MOUNT_RESULT=23
set +e
(
    set -e
    main recording_task "$DEFAULT_OUTPUT_ROOT" --contract-test
)
result=$?
set -e
if [[ "$result" -ne 23 ]]; then
    echo "non-stack recording did not fail closed on NAS error: rc=$result" >&2
    exit 1
fi
grep -Fq "mount-check:$DEFAULT_OUTPUT_ROOT" "$TRACE"
if grep -Fq 'recording:' "$TRACE"; then
    echo "recording continued after NAS preflight failure" >&2
    exit 1
fi

echo "bi-arm stack-only contract: OK"
