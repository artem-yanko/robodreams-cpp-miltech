#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

APP_BIN="${APP_BIN:-${REPO_DIR}/build/debug/course_project_autopilot/course_project_autopilot_app}"
CHECKER_BIN="${CHECKER_BIN:-${REPO_DIR}/course_project_autopilot/data/checker/checker_pi_arm64}"
UART_DEVICE="${UART_DEVICE:-/tmp/ttyA}"
CHECKER_UART="${CHECKER_UART:-/tmp/ttyH}"
SIM_BANK="${SIM_BANK:-/tmp/cpa-sim-bank}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_DIR}/course_project_autopilot/results}"
STUDENT_ID="${STUDENT_ID:-1035}"
PUBLISH="${PUBLISH:-false}"
DEBUG_AUTO="${DEBUG_AUTO:-true}"
DEBUG_TARGET_LOSS_AFTER="${DEBUG_TARGET_LOSS_AFTER:-}"
DEBUG_TARGET_LOSS_DURATION="${DEBUG_TARGET_LOSS_DURATION:-}"
MAVLINK="${MAVLINK:-false}"
MAVLINK_HOST="${MAVLINK_HOST:-192.168.56.1}"
MAVLINK_PORT="${MAVLINK_PORT:-14550}"
TEST_TIMEOUT_SEC="${TEST_TIMEOUT_SEC:-90}"
SOCAT_BIN="${SOCAT_BIN:-socat}"

TESTS=(T01 T02 T03 T04 T05 T06 T07 T08 T09 T10)
REPORT=()

usage() {
    echo "Usage: $0 [T01|T02|...|T10]"
    echo
    echo "Environment overrides:"
    echo "  APP_BIN=${APP_BIN}"
    echo "  CHECKER_BIN=${CHECKER_BIN}"
    echo "  UART_DEVICE=${UART_DEVICE}"
    echo "  CHECKER_UART=${CHECKER_UART}"
    echo "  SIM_BANK=${SIM_BANK}"
    echo "  OUTPUT_DIR=${OUTPUT_DIR}"
    echo "  STUDENT_ID=${STUDENT_ID}"
    echo "  PUBLISH=${PUBLISH}"
    echo "  DEBUG_AUTO=${DEBUG_AUTO}"
    echo "  DEBUG_TARGET_LOSS_AFTER=${DEBUG_TARGET_LOSS_AFTER}"
    echo "  DEBUG_TARGET_LOSS_DURATION=${DEBUG_TARGET_LOSS_DURATION}"
    echo "  MAVLINK=${MAVLINK}"
    echo "  MAVLINK_HOST=${MAVLINK_HOST}"
    echo "  MAVLINK_PORT=${MAVLINK_PORT}"
    echo "  SOCAT_BIN=${SOCAT_BIN}"
}

checker_mission_number() {
    case "$1" in
        T01) echo "1" ;;
        T02) echo "2" ;;
        T03) echo "3" ;;
        T04) echo "4" ;;
        T05) echo "5" ;;
        T06) echo "6" ;;
        T07) echo "7" ;;
        T08) echo "8" ;;
        T09) echo "9" ;;
        T10) echo "10" ;;
        *) return 1 ;;
    esac
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

if [[ $# -gt 1 ]]; then
    usage
    exit 1
fi

if [[ $# -eq 1 ]]; then
    TESTS=("$1")
fi

if [[ ! -x "${APP_BIN}" ]]; then
    echo "App binary is not executable: ${APP_BIN}" >&2
    exit 1
fi

if [[ ! -x "${CHECKER_BIN}" ]]; then
    echo "Checker binary is not executable: ${CHECKER_BIN}" >&2
    exit 1
fi

for test_id in "${TESTS[@]}"; do
    mission_number="$(checker_mission_number "${test_id}")"
    echo "[${test_id}] cleaning ${SIM_BANK}"
    rm -rf "${SIM_BANK}"
    rm -f "${UART_DEVICE}" "${CHECKER_UART}"

    echo "[${test_id}] starting socat"
    "${SOCAT_BIN}" -d -d "pty,raw,echo=0,link=${UART_DEVICE}" "pty,raw,echo=0,link=${CHECKER_UART}" &
    socat_pid=$!
    sleep 1

    echo "[${test_id}] starting checker mission ${mission_number}"
    "${CHECKER_BIN}" "${mission_number}" --uart "${CHECKER_UART}" --sim-bank "${SIM_BANK}" &
    checker_pid=$!

    cleanup() {
        kill "${checker_pid}" >/dev/null 2>&1 || true
        wait "${checker_pid}" >/dev/null 2>&1 || true
        kill "${socat_pid}" >/dev/null 2>&1 || true
        wait "${socat_pid}" >/dev/null 2>&1 || true
    }
    trap cleanup EXIT

    sleep 1
    if ! kill -0 "${checker_pid}" >/dev/null 2>&1; then
        echo "[${test_id}] checker exited before app start" >&2
        cleanup
        trap - EXIT
        REPORT+=("${test_id}: checker-failed")
        continue
    fi

    echo "[${test_id}] starting app"
    app_args=(
        --sim
        --uart "${UART_DEVICE}"
        --sim-bank "${SIM_BANK}"
        --test-id "${test_id}"
        --output-dir "${OUTPUT_DIR}"
        --student-id "${STUDENT_ID}"
        --publish "${PUBLISH}"
        --stop-after-drop true
    )

    if [[ "${DEBUG_AUTO}" == "true" ]]; then
        app_args+=(--debug-auto)
    fi

    if [[ -n "${DEBUG_TARGET_LOSS_AFTER}" ]]; then
        app_args+=(--debug-target-loss-after "${DEBUG_TARGET_LOSS_AFTER}")
    fi

    if [[ -n "${DEBUG_TARGET_LOSS_DURATION}" ]]; then
        app_args+=(--debug-target-loss-duration "${DEBUG_TARGET_LOSS_DURATION}")
    fi

    if [[ "${MAVLINK}" == "true" ]]; then
        app_args+=(
            --mavlink true
            --mavlink-host "${MAVLINK_HOST}"
            --mavlink-port "${MAVLINK_PORT}"
        )
    fi

    set +e
    timeout "${TEST_TIMEOUT_SEC}" "${APP_BIN}" "${app_args[@]}"
    app_status=$?
    set -e

    cleanup
    trap - EXIT

    if [[ ${app_status} -eq 0 ]]; then
        REPORT+=("${test_id}: ok")
    elif [[ ${app_status} -eq 124 ]]; then
        REPORT+=("${test_id}: timeout")
    else
        REPORT+=("${test_id}: failed(${app_status})")
    fi
done

echo "Summary:"
for line in "${REPORT[@]}"; do
    echo "  ${line}"
done
