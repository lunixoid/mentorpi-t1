#!/usr/bin/env bash
# Native macOS person_detect (SD013). Not Docker. Does not install t1ctl.
# Invoked by `make mac-detect`. Extra python args: make mac-detect ARGS='...'
set -euo pipefail

if [[ "$(uname -s)" != "Darwin" ]]; then
  echo "error: native macOS only (got $(uname -s); not linux/arm64 Docker)" >&2
  exit 1
fi
if [[ "$(uname -m)" != "arm64" ]]; then
  echo "error: Apple Silicon (arm64) required for MPS" >&2
  exit 1
fi
if ! command -v pixi >/dev/null 2>&1; then
  echo "error: pixi is required on this Mac (RoboStack Humble + YOLO)." >&2
  echo "install: curl -fsSL https://pixi.sh/install.sh | bash" >&2
  exit 1
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG="${ROOT}/host/mac_person_detect"
cd "${PKG}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONUNBUFFERED=1
export PYTORCH_ENABLE_MPS_FALLBACK="${PYTORCH_ENABLE_MPS_FALLBACK:-1}"

IMAGE_TOPIC="${IMAGE_TOPIC:-/aurora/rgb/image_raw}"
CONFIDENCE_THRESHOLD="${CONFIDENCE_THRESHOLD:-0.25}"
if [[ -z "${WEIGHTS:-}" ]]; then
  if [[ -f "${PKG}/yolo11n.pt" ]]; then
    WEIGHTS="${PKG}/yolo11n.pt"
  else
    WEIGHTS="yolo11n.pt"
  fi
fi

echo "person_detect ROS_DOMAIN_ID=${ROS_DOMAIN_ID} image=${IMAGE_TOPIC} weights=${WEIGHTS} conf=${CONFIDENCE_THRESHOLD}"

# Generate FastDDS LAN profile (pixi activation may not source this on `pixi run --`).
# shellcheck source=../host/mac_person_detect/activate-dds.sh
source "${PKG}/activate-dds.sh"

exec pixi run -- python person_detect.py \
  --image-topic "${IMAGE_TOPIC}" \
  --weights "${WEIGHTS}" \
  --confidence-threshold "${CONFIDENCE_THRESHOLD}" \
  "$@"
