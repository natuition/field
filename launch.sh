#!/usr/bin/env bash
set -euo pipefail

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-80}"
WORKERS="${WORKERS:-1}"
APP_MODULE="${APP_MODULE:-uiWebRobot.application:app}"
DIRECT_MODULE="${DIRECT_MODULE:-uiWebRobot.application}"
LAUNCH_MODE="${LAUNCH_MODE:-python}"
WORKDIR="${WORKDIR:-/home/violette/field}"

if [[ ! -d "${WORKDIR}" ]]; then
	echo "Error: working directory not found: ${WORKDIR}" >&2
	exit 1
fi

cd "${WORKDIR}"

if [[ "${EUID}" -ne 0 ]] && command -v sudo >/dev/null 2>&1; then
	SUDO_CMD=(sudo)
else
	SUDO_CMD=()
fi

if [[ "${LAUNCH_MODE}" == "python" ]]; then
	echo "Starting Python module: ${DIRECT_MODULE}"
	exec "${SUDO_CMD[@]}" python3 -m "${DIRECT_MODULE}"
elif [[ "${LAUNCH_MODE}" == "gunicorn" ]]; then
	echo "Starting Gunicorn: ${APP_MODULE} on ${HOST}:${PORT} with ${WORKERS} worker(s)"
	exec "${SUDO_CMD[@]}" python3 -m gunicorn \
		--workers "${WORKERS}" \
		--bind "${HOST}:${PORT}" \
		"${APP_MODULE}"
else
	echo "Error: unsupported LAUNCH_MODE '${LAUNCH_MODE}'. Use 'gunicorn' or 'python'." >&2
	exit 1
fi