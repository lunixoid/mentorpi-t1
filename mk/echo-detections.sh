#!/usr/bin/env bash
# Echo /perception/detections_2d from Mac with the same FastDDS env as the detector.
# Invoked by `make echo-detections`.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG="${ROOT}/host/mac_person_detect"
cd "${PKG}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
# shellcheck source=../host/mac_person_detect/activate-dds.sh
source "${PKG}/activate-dds.sh"
exec pixi run -- python echo_detections.py
