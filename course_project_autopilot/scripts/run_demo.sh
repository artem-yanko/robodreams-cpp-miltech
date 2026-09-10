#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEST_RUNNER="${SCRIPT_DIR}/run_sim_tests.sh"
LOG_DIR="${DEMO_LOG_DIR:-${PROJECT_DIR}/demo_logs}"
TEST_ID="${2:-T01}"
SCENARIO="${1:-}"

usage() {
    echo "Usage: $0 <scenario> [T01|T02|...|T10]"
    echo
    echo "Scenarios:"
    echo "  normal-auto       Switch MANUAL to AUTO in QGroundControl"
    echo "  control-loss      Close QGroundControl while the drone is in MANUAL"
    echo "  failsafe-return   Lose targets and QGroundControl, then return"
    echo "  failsafe-recovery Restore QGroundControl during failsafe return"
    echo
    echo "Logs: ${LOG_DIR}"
}

print_common_instructions() {
    echo "QGroundControl must listen on UDP port ${MAVLINK_PORT:-14550}."
    echo "Wait for: MAVLink GCS connected"
}

case "${SCENARIO}" in
    normal-auto)
        DEBUG_AUTO_VALUE=false
        TARGET_LOSS_AFTER=
        TIMEOUT_SECONDS=90
        echo "Demo: normal AUTO mission"
        print_common_instructions
        echo "In QGroundControl set CPA_MODE to 1."
        echo "Expected: MANUAL -> AUTO -> DROP -> MissionCompleteState."
        ;;
    control-loss)
        DEBUG_AUTO_VALUE=false
        TARGET_LOSS_AFTER=
        TIMEOUT_SECONDS=90
        echo "Demo: control link loss during MANUAL"
        print_common_instructions
        echo "Keep CPA_MODE at 0, then close QGroundControl."
        echo "Expected: MANUAL -> AUTO takeover -> DROP -> MissionCompleteState."
        ;;
    failsafe-return)
        DEBUG_AUTO_VALUE=true
        TARGET_LOSS_AFTER=1
        TIMEOUT_SECONDS=150
        echo "Demo: failsafe return"
        print_common_instructions
        echo "Close QGroundControl about 5 seconds after connection."
        echo "Expected: target timeout -> FailsafeState -> return -> MissionCompleteState."
        echo "Press Ctrl+C after FAILSAFE RETURN completed."
        ;;
    failsafe-recovery)
        DEBUG_AUTO_VALUE=true
        TARGET_LOSS_AFTER=1
        TIMEOUT_SECONDS=150
        echo "Demo: control link recovery during failsafe return"
        print_common_instructions
        echo "Close QGroundControl about 5 seconds after connection."
        echo "Open it again after FAILSAFE RETURN started."
        echo "Expected: FailsafeState -> MANUAL."
        echo "Press Ctrl+C after CONTROL LINK RESTORED."
        ;;
    -h|--help|"")
        usage
        exit 0
        ;;
    *)
        echo "Unknown scenario: ${SCENARIO}" >&2
        usage >&2
        exit 1
        ;;
esac

if [[ ! -x "${TEST_RUNNER}" ]]; then
    echo "Test runner is not executable: ${TEST_RUNNER}" >&2
    exit 1
fi

mkdir -p "${LOG_DIR}"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="${LOG_DIR}/${SCENARIO}-${TEST_ID}-${TIMESTAMP}.log"

echo "Test: ${TEST_ID}"
echo "Log: ${LOG_FILE}"
echo

set +e
env \
    DEBUG_AUTO="${DEBUG_AUTO_VALUE}" \
    DEBUG_TARGET_LOSS_AFTER="${TARGET_LOSS_AFTER}" \
    DEBUG_TARGET_LOSS_DURATION="" \
    MAVLINK=true \
    TEST_TIMEOUT_SEC="${TIMEOUT_SECONDS}" \
    "${TEST_RUNNER}" "${TEST_ID}" 2>&1 | tee "${LOG_FILE}"
RUNNER_STATUS=${PIPESTATUS[0]}
set -e

echo
echo "Demo log saved to ${LOG_FILE}"
exit "${RUNNER_STATUS}"
