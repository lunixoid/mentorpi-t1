# GNU Make 3.81 compatible (macOS /usr/bin/make). Do not use .ONESHELL or !=.
# Operator entry: make (help). Do not compile overlay on the Pi.

MAKEFILE_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
ROOT := $(abspath $(MAKEFILE_DIR))

PLATFORM := linux/arm64
OVERLAY_IMAGE := mentorpi-overlay-builder:arm64
T1CTL_IMAGE := mentorpi-t1ctl-builder:arm64
OVERLAY_DOCKERFILE := $(ROOT)/docker/overlay-builder/Dockerfile
T1CTL_DOCKERFILE := $(ROOT)/docker/t1ctl-builder/Dockerfile
OVERLAY_STAMP := $(ROOT)/build-arm64/.stamp-overlay-builder
T1CTL_STAMP := $(ROOT)/build-arm64/.stamp-t1ctl-builder
INSTALL_SETUP := $(ROOT)/build-arm64/ros/install/setup.bash
T1CTL_BIN := $(ROOT)/build-arm64/t1ctl/bin/t1ctl

.DEFAULT_GOAL := help

.PHONY: help need-docker env env-overlay env-t1ctl env-clean \
	build overlay t1ctl deploy provision mac-detect pi-detect echo-detections clean

help:
	@echo "Targets:"
	@echo "  help              this list (default; no docker/colcon)"
	@echo "  env               builder images (overlay + t1ctl)"
	@echo "  env-overlay       image $(OVERLAY_IMAGE)"
	@echo "  env-t1ctl         image $(T1CTL_IMAGE)"
	@echo "  build             overlay + t1ctl (linux/arm64) into build-arm64/"
	@echo "  overlay           ROS overlay only"
	@echo "  t1ctl             t1ctl only"
	@echo "  deploy            copy overlay + t1ctl to Pi; restart mentorpi-t1"
	@echo "  provision         Pi image/container + our unit (not docker rm MentorPi)"
	@echo "  mac-detect        native macOS person_detect (pixi)"
	@echo "  pi-detect         SSH t1ctl detect offline (ARGS=mac -> mac)"
	@echo "  echo-detections   echo /perception/detections_2d on Mac"
	@echo "  clean             remove build-arm64/ (not docker images)"
	@echo "  env-clean         remove builder image tags"
	@echo ""
	@echo "Env: PI_HOST PI_PASSWORD PI_STAGING CONTAINER FORCE_REBUILD"
	@echo "     WEIGHTS IMAGE_TOPIC CONFIDENCE_THRESHOLD ROS_DOMAIN_ID ARGS"

need-docker:
	@command -v docker >/dev/null 2>&1 || { echo "error: docker is required" >&2; exit 1; }

$(OVERLAY_STAMP): $(OVERLAY_DOCKERFILE) | need-docker
	docker build --platform $(PLATFORM) -t $(OVERLAY_IMAGE) $(ROOT)/docker/overlay-builder
	mkdir -p $(dir $@)
	touch $@

$(T1CTL_STAMP): $(T1CTL_DOCKERFILE) | need-docker
	docker build --platform $(PLATFORM) -t $(T1CTL_IMAGE) $(ROOT)/docker/t1ctl-builder
	mkdir -p $(dir $@)
	touch $@

env: env-overlay env-t1ctl

env-overlay: $(OVERLAY_STAMP)
	@if ! docker image inspect $(OVERLAY_IMAGE) >/dev/null 2>&1; then \
	  rm -f $(OVERLAY_STAMP); \
	  $(MAKE) $(OVERLAY_STAMP); \
	fi

env-t1ctl: $(T1CTL_STAMP)
	@if ! docker image inspect $(T1CTL_IMAGE) >/dev/null 2>&1; then \
	  rm -f $(T1CTL_STAMP); \
	  $(MAKE) $(T1CTL_STAMP); \
	fi

build: overlay t1ctl
	@echo "==> done"
	@echo "    overlay $(ROOT)/build-arm64/ros/install"
	@echo "    t1ctl   $(T1CTL_BIN)"

overlay: $(OVERLAY_STAMP)
	@if ! docker image inspect $(OVERLAY_IMAGE) >/dev/null 2>&1; then \
	  rm -f $(OVERLAY_STAMP); \
	  $(MAKE) $(OVERLAY_STAMP); \
	fi
	@mkdir -p $(ROOT)/build-arm64/ros $(ROOT)/build-arm64/t1ctl $(ROOT)/build-arm64/t1ctl-build
	@echo "==> ROS overlay ($(OVERLAY_IMAGE) $(PLATFORM))"
	docker run --rm --platform $(PLATFORM) \
	  -v "$(ROOT):/workspace" \
	  -w /workspace \
	  $(OVERLAY_IMAGE) \
	  bash -c 'set -eo pipefail; set +u; source /opt/ros/humble/setup.bash; set -u; set -eo pipefail; colcon --log-base /workspace/build-arm64/ros/log build --base-paths src --packages-up-to mentorpi_bringup mentorpi_platform mentorpi_calibration --build-base /workspace/build-arm64/ros/build --install-base /workspace/build-arm64/ros/install --cmake-args -DCMAKE_BUILD_TYPE=Release'
	@test -f "$(INSTALL_SETUP)" || { echo "error: missing $(INSTALL_SETUP)" >&2; exit 1; }
	@echo "    install: $(ROOT)/build-arm64/ros/install"

t1ctl: $(T1CTL_STAMP)
	@if ! docker image inspect $(T1CTL_IMAGE) >/dev/null 2>&1; then \
	  rm -f $(T1CTL_STAMP); \
	  $(MAKE) $(T1CTL_STAMP); \
	fi
	@mkdir -p $(ROOT)/build-arm64/ros $(ROOT)/build-arm64/t1ctl $(ROOT)/build-arm64/t1ctl-build
	@echo "==> t1ctl ($(T1CTL_IMAGE) $(PLATFORM))"
	docker run --rm --platform $(PLATFORM) \
	  -v "$(ROOT):/workspace" \
	  -w /workspace \
	  $(T1CTL_IMAGE) \
	  bash -c 'set -euo pipefail; cmake -S /workspace/host/t1ctl -B /workspace/build-arm64/t1ctl-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/workspace/build-arm64/t1ctl; cmake --build /workspace/build-arm64/t1ctl-build --parallel "$$(nproc)"; cmake --install /workspace/build-arm64/t1ctl-build'
	@test -x "$(T1CTL_BIN)" || { echo "error: missing $(T1CTL_BIN)" >&2; exit 1; }
	@desc="$$(file "$(T1CTL_BIN)")"; echo "    $$desc"; \
	echo "$$desc" | grep -q 'Mach-O' && { echo "error: t1ctl is Mach-O; expected ELF aarch64" >&2; exit 1; }; \
	echo "$$desc" | grep -Eq 'ARM aarch64|aarch64' || { echo "error: t1ctl is not aarch64: $$desc" >&2; exit 1; }

deploy:
	@bash $(ROOT)/mk/deploy.sh

provision:
	@bash $(ROOT)/mk/provision.sh

mac-detect:
	@bash $(ROOT)/mk/mac-detect.sh $(ARGS)

pi-detect:
	@bash $(ROOT)/mk/pi-detect.sh $(ARGS)

echo-detections:
	@bash $(ROOT)/mk/echo-detections.sh

clean:
	rm -rf $(ROOT)/build-arm64

env-clean:
	-docker rmi $(OVERLAY_IMAGE) $(T1CTL_IMAGE)
	rm -f $(OVERLAY_STAMP) $(T1CTL_STAMP)
