# Cross-build environment for the ROS overlay (linux/arm64).
# Apt lives here; project builds mount the repo and run colcon without apt-get.
# Distro OpenCV + Humble vision stack. Do not use Deptrum /usr/local OpenCV 4.10.
# NCNN (CPU, static, no Vulkan) for mentorpi_person_detect — SD026 T1.
# Vosk aarch64 + small-ru model for mentorpi_voice — SD031 T2.
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

ARG NCNN_GIT_TAG=20260526
ENV NCNN_ROOT=/opt/ncnn
ENV CMAKE_PREFIX_PATH=/opt/ncnn:${CMAKE_PREFIX_PATH}

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        ros-humble-vision-msgs \
        ros-humble-cv-bridge \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        libopencv-dev \
        libasound2-dev \
        nlohmann-json3-dev \
        unzip \
        curl \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Static ncnn for colcon link without apt at make build time. No Vulkan/Mesa.
RUN git clone --depth 1 --branch "${NCNN_GIT_TAG}" https://github.com/Tencent/ncnn.git /tmp/ncnn \
    && cd /tmp/ncnn \
    && git submodule update --init --depth 1 \
    && cmake -S /tmp/ncnn -B /tmp/ncnn/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${NCNN_ROOT}" \
        -DNCNN_VULKAN=OFF \
        -DNCNN_SHARED_LIB=OFF \
        -DNCNN_BUILD_TOOLS=OFF \
        -DNCNN_BUILD_EXAMPLES=OFF \
        -DNCNN_BUILD_BENCHMARK=OFF \
        -DNCNN_BUILD_TESTS=OFF \
        -DNCNN_OPENCV=OFF \
    && cmake --build /tmp/ncnn/build --parallel "$(nproc)" \
    && cmake --install /tmp/ncnn/build \
    && rm -rf /tmp/ncnn

# libvosk.so + vosk_api.h in /opt/vosk; model in /opt/vosk-models. Network only at make env-overlay.
ARG VOSK_VERSION=0.3.45
ARG VOSK_SHA256=45e95d37755deb07568e79497d7feba8c03aee5a9e071df29961aa023fd94541
ARG VOSK_MODEL=vosk-model-small-ru-0.22
ARG VOSK_MODEL_SHA256=961d5ff98a17f4aa6de69864d0aa71fa5bac682301d2b5d17a3f24c5c99a46d4
ENV VOSK_ROOT=/opt/vosk
ENV VOSK_MODEL_DIR=/opt/vosk-models/${VOSK_MODEL}

RUN curl -fsSL -o /tmp/vosk.zip \
      "https://github.com/alphacep/vosk-api/releases/download/v${VOSK_VERSION}/vosk-linux-aarch64-${VOSK_VERSION}.zip" \
    && echo "${VOSK_SHA256}  /tmp/vosk.zip" | sha256sum -c - \
    && mkdir -p "${VOSK_ROOT}" \
    && unzip -j -q /tmp/vosk.zip -d "${VOSK_ROOT}" \
    && rm -f /tmp/vosk.zip \
    && curl -fsSL -o /tmp/vosk-model.zip \
      "https://alphacephei.com/vosk/models/${VOSK_MODEL}.zip" \
    && echo "${VOSK_MODEL_SHA256}  /tmp/vosk-model.zip" | sha256sum -c - \
    && mkdir -p /opt/vosk-models \
    && unzip -q /tmp/vosk-model.zip -d /opt/vosk-models \
    && rm -f /tmp/vosk-model.zip \
    && test -f "${VOSK_ROOT}/libvosk.so" \
    && test -f "${VOSK_ROOT}/vosk_api.h" \
    && test -d "${VOSK_MODEL_DIR}/am" \
    && test -d "${VOSK_MODEL_DIR}/graph"

WORKDIR /workspace
