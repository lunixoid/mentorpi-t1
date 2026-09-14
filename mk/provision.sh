#!/usr/bin/env bash
# Invoked by `make provision` only. Does not docker rm MentorPi.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PI_HOST="${PI_HOST:-pi@192.168.88.56}"
PI_PASSWORD="${PI_PASSWORD:-raspberrypi}"
PI_STAGING="${PI_STAGING:-/home/pi/mentorpi_t1_ws}"
STOCK_CONTAINER="${STOCK_CONTAINER:-MentorPi}"
CONTAINER="${CONTAINER:-mentorpi-t1}"
OUR_IMAGE="${OUR_IMAGE:-mentorpi-t1}"
OVERLAY_DEST="/home/ubuntu/mentorpi_t1_ws"
UNIT_SRC="${ROOT}/host/systemd/mentorpi-t1.service"
SUDOERS_SRC="${ROOT}/host/sudoers.d/t1ctl"
DESKTOP_LAUNCHER_SRC="${ROOT}/host/desktop/mentorpi-rviz"
DESKTOP_ENTRY_SRC="${ROOT}/host/desktop/mentorpi-rviz.desktop"
DEPLOY_SH="${ROOT}/mk/deploy.sh"
DOCKERFILE_DIR="${ROOT}/docker/mentorpi-t1"
# Rebuild image mentorpi-t1 even if the tag already exists (default: skip).
FORCE_REBUILD="${FORCE_REBUILD:-0}"

if ! command -v sshpass >/dev/null 2>&1; then
  echo "error: sshpass is required (install: sudo apt install sshpass)" >&2
  exit 1
fi

if [[ ! -f "${DOCKERFILE_DIR}/Dockerfile" ]]; then
  echo "error: missing ${DOCKERFILE_DIR}/Dockerfile" >&2
  exit 1
fi
if [[ ! -f "${DOCKERFILE_DIR}/ros.key" ]]; then
  echo "error: missing ${DOCKERFILE_DIR}/ros.key (ROS apt keyring, vendored for Pi docker build)" >&2
  exit 1
fi

export SSHPASS="${PI_PASSWORD}"
export SSH_ASKPASS_REQUIRE=never
SSH_CONTROL_DIR="${TMPDIR:-/tmp}/mentorpi-t1-ssh.$$"
mkdir -p "${SSH_CONTROL_DIR}"
SSH_OPTS=(
  -o StrictHostKeyChecking=accept-new
  -o UserKnownHostsFile="${HOME}/.ssh/known_hosts"
  -o LogLevel=ERROR
  -o ConnectTimeout=10
  -o ServerAliveInterval=30
  -o ServerAliveCountMax=40
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

sudo_pi() {
  ssh_pi "printf '%s\n' '${PI_PASSWORD}' | sudo -S -p '' $*"
}

ensure_staging() {
  if ssh_pi "test -f '${PI_STAGING}/install/setup.bash'"; then
    return 0
  fi
  if [[ ! -f "${DEPLOY_SH}" ]]; then
    echo "error: overlay missing at ${PI_STAGING}/install/setup.bash and ${DEPLOY_SH} is not present" >&2
    exit 1
  fi
  echo "==> staging overlay missing; calling make deploy"
  bash "${DEPLOY_SH}"
  if ! ssh_pi "test -f '${PI_STAGING}/install/setup.bash'"; then
    echo "error: make deploy ran but ${PI_STAGING}/install/setup.bash is still missing" >&2
    exit 1
  fi
}

install_host_files_if_missing() {
  if ! ssh_pi "test -f /etc/systemd/system/mentorpi-t1.service"; then
    if [[ ! -f "${UNIT_SRC}" ]]; then
      echo "error: missing ${UNIT_SRC}" >&2
      exit 1
    fi
    echo "==> install unit /etc/systemd/system/mentorpi-t1.service"
    ssh_pi "mkdir -p '${PI_STAGING}'"
    rsync -a -e "${RSYNC_RSH}" \
      "${UNIT_SRC}" \
      "${PI_HOST}:${PI_STAGING}/mentorpi-t1.service"
    sudo_pi "install -m 0644 -o root -g root '${PI_STAGING}/mentorpi-t1.service' /etc/systemd/system/mentorpi-t1.service"
  else
    echo "==> unit already present"
  fi

  if ! ssh_pi "test -f /etc/sudoers.d/t1ctl"; then
    if [[ ! -f "${SUDOERS_SRC}" ]]; then
      echo "error: missing ${SUDOERS_SRC}" >&2
      exit 1
    fi
    echo "==> install sudoers /etc/sudoers.d/t1ctl"
    ssh_pi "mkdir -p '${PI_STAGING}'"
    rsync -a -e "${RSYNC_RSH}" \
      "${SUDOERS_SRC}" \
      "${PI_HOST}:${PI_STAGING}/t1ctl.sudoers"
    sudo_pi "visudo -cf '${PI_STAGING}/t1ctl.sudoers'"
    sudo_pi "install -m 0440 -o root -g root '${PI_STAGING}/t1ctl.sudoers' /etc/sudoers.d/t1ctl"
  else
    echo "==> sudoers already present"
  fi
}

install_desktop_launcher() {
  if [[ ! -f "${DESKTOP_LAUNCHER_SRC}" ]]; then
    echo "error: missing ${DESKTOP_LAUNCHER_SRC}" >&2
    exit 1
  fi
  if [[ ! -f "${DESKTOP_ENTRY_SRC}" ]]; then
    echo "error: missing ${DESKTOP_ENTRY_SRC}" >&2
    exit 1
  fi
  echo "==> install desktop launcher mentorpi-rviz"
  ssh_pi "mkdir -p '${PI_STAGING}' /home/pi/Desktop"
  rsync -a -e "${RSYNC_RSH}" \
    "${DESKTOP_LAUNCHER_SRC}" \
    "${DESKTOP_ENTRY_SRC}" \
    "${PI_HOST}:${PI_STAGING}/"
  sudo_pi "install -m 0755 '${PI_STAGING}/mentorpi-rviz' /usr/local/bin/mentorpi-rviz"
  sudo_pi "install -m 0644 -o pi -g pi '${PI_STAGING}/mentorpi-rviz.desktop' /home/pi/Desktop/mentorpi-rviz.desktop"
}

echo "==> Pi ${PI_HOST}"
ssh_pi "true"

ensure_staging

echo "==> copy Dockerfile to ${PI_STAGING}"
ssh_pi "mkdir -p '${PI_STAGING}/install' '${PI_STAGING}/config/common' '${PI_STAGING}/config/platform/t1' && touch '${PI_STAGING}/config/common/.keep' '${PI_STAGING}/config/platform/t1/.keep'"
rsync -a -e "${RSYNC_RSH}" \
  "${DOCKERFILE_DIR}/Dockerfile" \
  "${DOCKERFILE_DIR}/.dockerignore" \
  "${DOCKERFILE_DIR}/ros.key" \
  "${PI_HOST}:${PI_STAGING}/"

echo "==> build / create / start (remote)"
ssh_pi "bash -s" <<REMOTE
set -euo pipefail

STOCK='${STOCK_CONTAINER}'
CONTAINER='${CONTAINER}'
OUR_IMAGE='${OUR_IMAGE}'
STAGING='${PI_STAGING}'
OVERLAY='${OVERLAY_DEST}'
FORCE_REBUILD='${FORCE_REBUILD}'

if ! docker inspect --type container "\${STOCK}" >/dev/null 2>&1; then
  echo "error: stock container \${STOCK} is not present; refusing to invent a base image" >&2
  exit 1
fi

STOCK_IMAGE="\$(docker inspect --type container -f '{{.Config.Image}}' "\${STOCK}")"
if [[ -z "\${STOCK_IMAGE}" ]]; then
  echo "error: container \${STOCK} has empty Config.Image" >&2
  exit 1
fi
if ! docker image inspect "\${STOCK_IMAGE}" >/dev/null 2>&1; then
  echo "error: image \${STOCK_IMAGE} (from \${STOCK}) is not present" >&2
  exit 1
fi
echo "    stock image=\${STOCK_IMAGE}"

DEPTRUM_SRC='/home/ubuntu/third_party_ros2/third_party_ws/install/deptrum-ros-driver-aurora930'
DEPTRUM_NODE="\${DEPTRUM_SRC}/lib/deptrum-ros-driver-aurora930/aurora930_node"
# Apt binary from ros-humble-imu-complementary-filter (SD010). Not a source
# install under third_party_ws.
IMU_FILTER_NODE='/opt/ros/humble/lib/imu_complementary_filter/complementary_filter_node'

image_has_camera_stack() {
  docker run --rm --network none --entrypoint /bin/bash "\${OUR_IMAGE}" -lc \
    "test -x '\${DEPTRUM_NODE}' && ls /usr/local/lib/libopencv_core.so* >/dev/null 2>&1"
}

image_has_imu_filter() {
  docker run --rm --network none --entrypoint /bin/bash "\${OUR_IMAGE}" -lc \
    "test -x '\${IMU_FILTER_NODE}'"
}

image_has_voice_stack() {
  docker run --rm --network none --entrypoint /bin/bash "\${OUR_IMAGE}" -lc \
    "command -v RHVoice-test >/dev/null && ldconfig -p | grep -q 'libasound.so.2'"
}

image_has_vision_msgs() {
  docker run --rm --network none --entrypoint /bin/bash "\${OUR_IMAGE}" -lc \
    "test -e /opt/ros/humble/lib/libvision_msgs__rosidl_typesupport_cpp.so"
}

image_has_rviz2() {
  docker run --rm --network none --entrypoint /bin/bash "\${OUR_IMAGE}" -lc \
    "command -v rviz2 >/dev/null"
}

stage_camera_vendor_from_stock() {
  echo "==> copy Deptrum + OpenCV 4.10 from \${STOCK} RW into build context"
  rm -rf "\${STAGING}/vendor-deptrum" "\${STAGING}/vendor-opencv" "\${STAGING}/.vendor-usr-local-lib"
  if ! docker cp "\${STOCK}:\${DEPTRUM_SRC}" "\${STAGING}/vendor-deptrum"; then
    echo "error: \${STOCK} is missing \${DEPTRUM_SRC}" >&2
    exit 1
  fi
  if [[ ! -x "\${STAGING}/vendor-deptrum/lib/deptrum-ros-driver-aurora930/aurora930_node" ]]; then
    echo "error: staged vendor-deptrum has no aurora930_node" >&2
    exit 1
  fi
  mkdir -p "\${STAGING}/.vendor-usr-local-lib"
  docker cp "\${STOCK}:/usr/local/lib/." "\${STAGING}/.vendor-usr-local-lib/"
  mkdir -p "\${STAGING}/vendor-opencv"
  found=0
  for f in "\${STAGING}/.vendor-usr-local-lib"/libopencv*; do
    if [[ -e "\${f}" ]]; then
      cp -a "\${f}" "\${STAGING}/vendor-opencv/"
      found=1
    fi
  done
  rm -rf "\${STAGING}/.vendor-usr-local-lib"
  if [[ "\${found}" -ne 1 ]] || ! ls "\${STAGING}/vendor-opencv"/libopencv_core.so* >/dev/null 2>&1; then
    echo "error: \${STOCK} /usr/local/lib has no libopencv_core.so*" >&2
    exit 1
  fi
}

need_build=0
if [[ "\${FORCE_REBUILD}" == "1" ]]; then
  need_build=1
elif ! docker image inspect "\${OUR_IMAGE}" >/dev/null 2>&1; then
  need_build=1
elif ! image_has_camera_stack; then
  echo "==> image \${OUR_IMAGE} exists but Deptrum/OpenCV stack is missing — rebuild"
  need_build=1
elif ! image_has_imu_filter; then
  echo "==> image \${OUR_IMAGE} exists but imu_complementary_filter is missing — rebuild"
  need_build=1
elif ! image_has_voice_stack; then
  echo "==> image \${OUR_IMAGE} exists but voice stack (RHVoice/ALSA) is missing — rebuild"
  need_build=1
elif ! image_has_vision_msgs; then
  echo "==> image \${OUR_IMAGE} exists but ros-humble-vision-msgs is missing — rebuild"
  need_build=1
elif ! image_has_rviz2; then
  echo "==> image \${OUR_IMAGE} exists but ros-humble-rviz2 is missing — rebuild"
  need_build=1
fi

if [[ "\${need_build}" -eq 1 ]]; then
  stage_camera_vendor_from_stock
  echo "==> docker build --network host -t \${OUR_IMAGE} FROM \${STOCK_IMAGE}"
  docker build --network host --build-arg BASE_IMAGE="\${STOCK_IMAGE}" -t "\${OUR_IMAGE}" "\${STAGING}"
else
  echo "==> image \${OUR_IMAGE} already exists with camera stack, imu_complementary_filter, voice stack, vision_msgs, and rviz2, skip build"
fi

if docker inspect --type container "\${CONTAINER}" >/dev/null 2>&1; then
  container_id="\$(docker inspect --type container -f '{{.Image}}' "\${CONTAINER}")"
  image_id="\$(docker image inspect -f '{{.Id}}' "\${OUR_IMAGE}")"
  config_dest="\${OVERLAY}/config"
  has_config="\$(docker inspect --type container -f '{{range .Mounts}}{{println .Destination}}{{end}}' "\${CONTAINER}" | grep -Fx "\${config_dest}" || true)"
  if [[ "\${container_id}" != "\${image_id}" ]]; then
    echo "==> \${CONTAINER} is on stale image \${container_id}"
    echo "    tag \${OUR_IMAGE} is \${image_id} — recreate (does not rm \${STOCK})"
    docker stop -t 5 "\${CONTAINER}" >/dev/null || true
    docker rm "\${CONTAINER}" >/dev/null
  elif [[ -z "\${has_config}" ]]; then
    echo "==> \${CONTAINER} is missing config bind-mount \${STAGING}/config:\${config_dest}"
    echo "    recreate (does not rm \${STOCK})"
    docker stop -t 5 "\${CONTAINER}" >/dev/null || true
    docker rm "\${CONTAINER}" >/dev/null
  else
    has_display="\$(docker inspect --type container -f '{{range .Config.Env}}{{println .}}{{end}}' "\${CONTAINER}" | grep -Fx 'DISPLAY=:0' || true)"
    if [[ -z "\${has_display}" ]]; then
      echo "==> \${CONTAINER} is missing DISPLAY=:0 env"
      echo "    recreate (does not rm \${STOCK})"
      docker stop -t 5 "\${CONTAINER}" >/dev/null || true
      docker rm "\${CONTAINER}" >/dev/null
    fi
  fi
fi

if docker inspect --type container "\${CONTAINER}" >/dev/null 2>&1; then
  echo "==> container \${CONTAINER} already exists, skip create"
  img="\$(docker inspect --type container -f '{{.Config.Image}}' "\${CONTAINER}")"
  echo "    image=\${img}"
  docker update --restart=no "\${CONTAINER}" >/dev/null
  echo "    --restart no"
else
  echo "==> create container \${CONTAINER} from \${OUR_IMAGE}"
  binds=()
  while IFS= read -r line; do
    [[ -n "\${line}" ]] && binds+=("\${line}")
  done < <(docker inspect --type container -f '{{range .HostConfig.Binds}}{{println .}}{{end}}' "\${STOCK}" 2>/dev/null || true)

  if [[ \${#binds[@]} -eq 0 ]]; then
    while IFS= read -r line; do
      [[ -n "\${line}" ]] && binds+=("\${line}")
    done < <(docker inspect --type container -f '{{range .Mounts}}{{if eq .Type "bind"}}{{.Source}}:{{.Destination}}{{if .RW}}{{else}}:ro{{end}}{{println}}{{end}}{{end}}' "\${STOCK}" 2>/dev/null || true)
  fi

  if [[ \${#binds[@]} -eq 0 ]]; then
    echo "    inspect binds empty; using last-known MentorPi binds"
    fallback=(
      '/var/lib/dbus:/var/lib/dbus'
      '/dev:/dev'
      '/home/pi/docker/tmp:/home/ubuntu/shared'
      '/run/user/1000/pulse:/run/user/1000/pulse'
      '/tmp/.X11-unix:/tmp/.X11-unix'
    )
    for spec in "\${fallback[@]}"; do
      src="\${spec%%:*}"
      if [[ -e "\${src}" ]]; then
        binds+=("\${spec}")
      else
        echo "    warning: bind source missing, skip \${spec}" >&2
      fi
    done
  fi

  net="\$(docker inspect --type container -f '{{.HostConfig.NetworkMode}}' "\${STOCK}")"
  [[ -n "\${net}" ]] || net=host
  # Restart=no: stock mode docker-stops this container and it must stay down
  # (MentorPi is restart=always; copying that would bring both up on boot).
  config_bind="\${STAGING}/config:\${OVERLAY}/config"
  already_bound=0
  for b in "\${binds[@]}"; do
    dest="\${b#*:}"
    dest="\${dest%%:*}"
    if [[ "\${dest}" == "\${OVERLAY}/config" ]]; then
      already_bound=1
    fi
  done
  if [[ "\${already_bound}" -eq 0 ]]; then
    binds+=("\${config_bind}")
  fi

  echo "    --restart no"
  echo "    --env DISPLAY=:0"
  echo "    --env QT_X11_NO_MITSHM=1"
  create_cmd=(docker create --name "\${CONTAINER}" --network "\${net}" --privileged --restart no --env DISPLAY=:0 --env QT_X11_NO_MITSHM=1)
  for b in "\${binds[@]}"; do
    echo "    -v \${b}"
    create_cmd+=("-v" "\${b}")
  done
  create_cmd+=("\${OUR_IMAGE}" tail -f /dev/null)
  "\${create_cmd[@]}"
fi

echo "==> docker start \${CONTAINER}"
docker start "\${CONTAINER}" >/dev/null

if ! docker exec "\${CONTAINER}" id ubuntu >/dev/null 2>&1; then
  echo "error: user ubuntu missing inside \${CONTAINER}" >&2
  exit 1
fi
if ! docker exec "\${CONTAINER}" test -f '${OVERLAY_DEST}/install/setup.bash'; then
  echo "error: overlay missing at ${OVERLAY_DEST}/install/setup.bash inside \${CONTAINER}" >&2
  exit 1
fi
REMOTE

install_host_files_if_missing
install_desktop_launcher

echo "==> systemctl daemon-reload"
sudo_pi systemctl daemon-reload

# Never two ROS launches: stock games off before our stage1.
echo "==> disable --now start_node.service (stock games off; start_node.sh not edited)"
sudo_pi systemctl disable --now start_node.service
echo "==> enable --now mentorpi-t1.service (our stage1 launch)"
sudo_pi systemctl enable --now mentorpi-t1.service

echo "==> docker ps -a (one running container per mode; do not rm ${STOCK_CONTAINER})"
ssh_pi "docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Command}}'"
echo "==> units"
ssh_pi "systemctl is-enabled mentorpi-t1.service; systemctl is-active mentorpi-t1.service; systemctl is-enabled start_node.service || true; systemctl is-active start_node.service || true"

echo "==> done"
echo "    stock container ${STOCK_CONTAINER} was not removed"
echo "    our launch: ros2 launch mentorpi_bringup stage1.launch.py in ${CONTAINER}"
echo "    stock launch (t1ctl stock): bringup.launch.py in ${STOCK_CONTAINER}"
