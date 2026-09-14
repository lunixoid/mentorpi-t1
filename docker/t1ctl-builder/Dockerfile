# Cross-build environment for t1ctl (linux/arm64).
# Apt lives here; project builds mount the repo and run cmake without apt-get.
FROM debian:12

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        g++ \
        cmake \
        git \
        make \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
