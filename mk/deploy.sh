#!/usr/bin/env bash
# Copy arm64 overlay + t1ctl to the Pi. If mentorpi-t1 exists: replace install/,
# docker restart the container, then t1ctl restart so ROS loads the new overlay.
# Does not rebuild image mentorpi-t1 or docker rm MentorPi.
# Invoked by `make deploy` only.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PI_HOST="${PI_HOST:-pi@192.168.88.56}"
PI_PASSWORD="${PI_PASSWORD:-raspberrypi}"
PI_STAGING="${PI_STAGING:-/home/pi/mentorpi_t1_ws}"
CONTAINER="${CONTAINER:-mentorpi-t1}"
OVERLAY_DEST="/home/ubuntu/mentorpi_t1_ws"

INSTALL_DIR="${ROOT}/build-arm64/ros/install"
T1CTL_BIN="${ROOT}/build-arm64/t1ctl/bin/t1ctl"
UNIT_SRC="${ROOT}/host/systemd/mentorpi-t1.service"
SUDOERS_SRC="${ROOT}/host/sudoers.d/t1ctl"
SYSCTL_SRC="${ROOT}/host/sysctl.d/60-mentorpi-t1-dds.conf"
DESKTOP_LAUNCHER_SRC="${ROOT}/host/desktop/mentorpi-rviz"
DESKTOP_ENTRY_SRC="${ROOT}/host/desktop/mentorpi-rviz.desktop"

PATH="/opt/homebrew/bin:/usr/local/bin:${PATH}"

if ! command -v sshpass >/dev/null 2>&1; then
  echo "error: sshpass is required (expected /opt/homebrew/bin/sshpass)" >&2
  exit 1
fi

if [[ ! -f "${INSTALL_DIR}/setup.bash" ]]; then
  echo "error: overlay not built (${INSTALL_DIR}/setup.bash). Run make build first." >&2
  exit 1
fi
if [[ ! -x "${T1CTL_BIN}" ]]; then
  echo "error: t1ctl not built (${T1CTL_BIN}). Run make build first." >&2
  exit 1
fi

FILE_DESC="$(file "${T1CTL_BIN}")"
if echo "${FILE_DESC}" | grep -q 'Mach-O'; then
  echo "error: refusing to install Mach-O t1ctl on Pi: ${FILE_DESC}" >&2
  exit 1
fi
if ! echo "${FILE_DESC}" | grep -Eq 'ARM aarch64|aarch64'; then
  echo "error: t1ctl is not linux aarch64: ${FILE_DESC}" >&2
  exit 1
fi

export SSHPASS="${PI_PASSWORD}"
# Cursor/macOS often has DISPLAY + ssh-agent keys; without these, later ssh
# calls hit askpass or "Too many authentication failures" after rsync.
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
RSYNC_RSH="env SSH_ASKPASS_REQUIRE=never DISPLAY= SSH_AUTH_SOCK= sshpass -e ssh ${SSH_OPTS[*]}"

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
ssh_pi "mkdir -p '${PI_STAGING}/install' '${PI_STAGING}/config/common' '${PI_STAGING}/config/platform/t1'"

echo "==> rsync overlay -> ${PI_HOST}:${PI_STAGING}/install"
rsync -a --delete -e "${RSYNC_RSH}" \
  "${INSTALL_DIR}/" \
  "${PI_HOST}:${PI_STAGING}/install/"

echo "==> install t1ctl -> /usr/local/bin/t1ctl"
rsync -a -e "${RSYNC_RSH}" \
  "${T1CTL_BIN}" \
  "${PI_HOST}:${PI_STAGING}/t1ctl"
ssh_pi "printf '%s\n' '${PI_PASSWORD}' | sudo -S -p '' install -m 0755 '${PI_STAGING}/t1ctl' /usr/local/bin/t1ctl"

if [[ -f "${DESKTOP_LAUNCHER_SRC}" && -f "${DESKTOP_ENTRY_SRC}" ]]; then
  echo "==> install desktop launcher mentorpi-rviz"
  rsync -a -e "${RSYNC_RSH}" \
    "${DESKTOP_LAUNCHER_SRC}" \
    "${DESKTOP_ENTRY_SRC}" \
    "${PI_HOST}:${PI_STAGING}/"
  ssh_pi "mkdir -p /home/pi/Desktop"
  ssh_pi "printf '%s\n' '${PI_PASSWORD}' | sudo -S -p '' install -m 0755 '${PI_STAGING}/mentorpi-rviz' /usr/local/bin/mentorpi-rviz"
  ssh_pi "printf '%s\n' '${PI_PASSWORD}' | sudo -S -p '' install -m 0644 -o pi -g pi '${PI_STAGING}/mentorpi-rviz.desktop' /home/pi/Desktop/mentorpi-rviz.desktop"
fi

echo "==> remove leftover host chassis-cmd"
ssh_pi "printf '%s\n' '${PI_PASSWORD}' | sudo -S -p '' rm -f /usr/local/bin/chassis-cmd '${PI_STAGING}/chassis-cmd'" || true

if [[ -f "${UNIT_SRC}" ]]; then
  echo "==> copy unit"
  rsync -a -e "${RSYNC_RSH}" \
    "${UNIT_SRC}" \
    "${PI_HOST}:${PI_STAGING}/mentorpi-t1.service"
  ssh_pi "printf '%s\n' '${PI_PASSWORD}' | sudo -S -p '' install -m 0644 -o root -g root '${PI_STAGING}/mentorpi-t1.service' /etc/systemd/system/mentorpi-t1.service"
  ssh_pi "printf '%s\n' '${PI_PASSWORD}' | sudo -S -p '' systemctl daemon-reload"
fi

if [[ -f "${SUDOERS_SRC}" ]]; then
  echo "==> copy sudoers.d/t1ctl"
  rsync -a -e "${RSYNC_RSH}" \
    "${SUDOERS_SRC}" \
    "${PI_HOST}:${PI_STAGING}/t1ctl.sudoers"
  ssh_pi "printf '%s\n' '${PI_PASSWORD}' | sudo -S -p '' visudo -cf '${PI_STAGING}/t1ctl.sudoers'"
  ssh_pi "printf '%s\n' '${PI_PASSWORD}' | sudo -S -p '' install -m 0440 -o root -g root '${PI_STAGING}/t1ctl.sudoers' /etc/sudoers.d/t1ctl"
fi

if [[ -f "${SYSCTL_SRC}" ]]; then
  echo "==> copy sysctl.d/60-mentorpi-t1-dds.conf"
  rsync -a -e "${RSYNC_RSH}" \
    "${SYSCTL_SRC}" \
    "${PI_HOST}:${PI_STAGING}/60-mentorpi-t1-dds.conf"
  ssh_pi "printf '%s\n' '${PI_PASSWORD}' | sudo -S -p '' install -m 0644 -o root -g root '${PI_STAGING}/60-mentorpi-t1-dds.conf' /etc/sysctl.d/60-mentorpi-t1-dds.conf"
  ssh_pi "printf '%s\n' '${PI_PASSWORD}' | sudo -S -p '' sysctl -p /etc/sysctl.d/60-mentorpi-t1-dds.conf >/dev/null"
  RMEM_MAX="$(ssh_pi "/sbin/sysctl -n net.core.rmem_max")"
  echo "==> sysctl net.core.rmem_max=${RMEM_MAX}"
fi

CONTAINER_STATE="$(ssh_pi "docker inspect --type container -f '{{.State.Running}}' '${CONTAINER}' 2>/dev/null || true")"
if [[ -z "${CONTAINER_STATE}" ]]; then
  echo "==> container ${CONTAINER} not found"
  echo "    overlay left at ${PI_STAGING} (provision will COPY it into image ${CONTAINER})"
  echo "T6_CONTAINER=missing"
else
  echo "==> stop launch (t1-stop) before replacing overlay"
  ssh_pi "sudo -n /usr/bin/systemctl stop mentorpi-t1.service" || true
  echo "==> replace overlay in ${CONTAINER} and docker restart"
  ssh_pi "bash -s" <<REMOTE
set -euo pipefail
CONTAINER='${CONTAINER}'
STAGING='${PI_STAGING}'
DEST='${OVERLAY_DEST}'
docker update --restart=no "\${CONTAINER}" >/dev/null
if [[ "\$(docker inspect --type container -f '{{.State.Running}}' "\${CONTAINER}")" != "true" ]]; then
  docker start "\${CONTAINER}"
fi
docker exec "\${CONTAINER}" rm -rf "\${DEST}/install"
docker exec "\${CONTAINER}" mkdir -p "\${DEST}/install" "\${DEST}/config/common" "\${DEST}/config/platform/t1"
docker cp "\${STAGING}/install/." "\${CONTAINER}:\${DEST}/install/"
if docker exec "\${CONTAINER}" id ubuntu >/dev/null 2>&1; then
  docker exec "\${CONTAINER}" chown -R ubuntu:ubuntu "\${DEST}"
fi
docker restart -t 5 "\${CONTAINER}"
REMOTE
  echo "==> t1ctl restart (source new overlay)"
  ssh_pi "t1ctl restart"
  echo "T6_CONTAINER=present restarted=1"
fi

echo "==> verify t1ctl --version"
VERSION_OUT="$(ssh_pi "t1ctl --version")"
echo "    ${VERSION_OUT}"

echo "==> done"
