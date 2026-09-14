#!/usr/bin/env bash
# SSH to Pi host and run t1ctl detect (SD026). Not Darwin-only. No deploy/docker.
# Invoked by `make pi-detect`. make pi-detect ARGS=mac -> t1ctl detect mac
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PI_HOST="${PI_HOST:-pi@192.168.88.56}"
PI_PASSWORD="${PI_PASSWORD:-raspberrypi}"

PATH="/opt/homebrew/bin:/usr/local/bin:${PATH}"

if ! command -v sshpass >/dev/null 2>&1; then
  echo "error: sshpass is required (expected /opt/homebrew/bin/sshpass)" >&2
  exit 1
fi

MODE="${1:-}"
if [[ -z "${MODE}" ]]; then
  REMOTE_CMD="t1ctl detect offline"
elif [[ "${MODE}" == "mac" ]]; then
  REMOTE_CMD="t1ctl detect mac"
else
  echo "error: unknown ARGS=${MODE}; use empty (offline) or mac" >&2
  exit 1
fi

export SSHPASS="${PI_PASSWORD}"
# Cursor/macOS often has DISPLAY + ssh-agent keys; without these, ssh hits askpass
# or "Too many authentication failures".
export SSH_ASKPASS_REQUIRE=never
SSH_CONTROL_DIR="${TMPDIR:-/tmp}/mentorpi-t1-ssh.$$"
mkdir -p "${SSH_CONTROL_DIR}"
SSH_OPTS=(
  -o StrictHostKeyChecking=accept-new
  -o UserKnownHostsFile="${HOME}/.ssh/known_hosts"
  -o LogLevel=ERROR
  -o ConnectTimeout=10
  -o PreferredAuthentications=password
  -o PubkeyAuthentication=no
  -o NumberOfPasswordPrompts=1
  -o KbdInteractiveAuthentication=no
  -o ControlMaster=auto
  -o ControlPath="${SSH_CONTROL_DIR}/cm"
  -o ControlPersist=30
)

ssh_pi() {
  env SSH_ASKPASS_REQUIRE=never DISPLAY= SSH_AUTH_SOCK= \
    sshpass -e ssh "${SSH_OPTS[@]}" "${PI_HOST}" -- "$@"
}

cleanup_ssh() {
  env SSH_ASKPASS_REQUIRE=never DISPLAY= SSH_AUTH_SOCK= \
    sshpass -e ssh "${SSH_OPTS[@]}" -O exit "${PI_HOST}" 2>/dev/null || true
  rm -rf "${SSH_CONTROL_DIR}"
}
trap cleanup_ssh EXIT

echo "==> Pi ${PI_HOST}"
echo "==> ${REMOTE_CMD}"
ssh_pi "${REMOTE_CMD}"
