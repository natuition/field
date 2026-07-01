#!/usr/bin/env bash
set -euo pipefail

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-80}"
WORKERS="${WORKERS:-1}"
APP_MODULE="${APP_MODULE:-uiWebRobot.application:app}"
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

echo "Starting Gunicorn: ${APP_MODULE} on ${HOST}:${PORT} with ${WORKERS} worker(s)"
exec "${SUDO_CMD[@]}" python3 -m gunicorn \
	--workers "${WORKERS}" \
	--bind "${HOST}:${PORT}" \
	"${APP_MODULE}"