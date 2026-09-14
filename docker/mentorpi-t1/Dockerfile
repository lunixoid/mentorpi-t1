# Overlay layer on the image MentorPi was created from (not ros:humble from Docker Hub).
# pyserial lives in MentorPi's RW layer, not in the stock image; ros_robot_controller
# imports serial and dies without it. foxglove_bridge is not in the stock image either;
# mentorpi_bringup launches it for SD005 viewer. Vendor imu_filter.launch.py needs
# complementary_filter_node; the base image has no imu_complementary_filter — apt
# ros-humble-imu-complementary-filter (not a CCNY source build, not the imu-tools
# metapackage which pulls rviz). Install into the overlay image so SIT survives recreate.
#
# SD008: deptrum-ros-driver-aurora930 + OpenCV 4.10 also live only in MentorPi RW
# (third_party_ros2 / /usr/local/lib), not the base image. provision-pi.sh copies
# them into this build context (vendor-deptrum, vendor-opencv). Recreate the
# container after FORCE_REBUILD=1 or the running mentorpi-t1 keeps the old image.
ARG BASE_IMAGE
FROM ${BASE_IMAGE}
USER root
# Base MentorPi/ros:humble may pin apt to mirrors.tuna.tsinghua.edu.cn (404 off China) and
# ship an expired ROS signing key (EXPKEYSIG F42ED6FBAB17C654 after 2025-06).
# Key is vendored (ros.key) — docker build on the Pi cannot resolve
# raw.githubusercontent.com (curl exit 6).
COPY ros.key /usr/share/keyrings/ros-archive-keyring.gpg
# SD031: RHVoice 1.2.3 + ALSA runtime (jammy multiverse).
# SD013: person_perception / person_detect_pi link Humble vision_msgs; the stock
# image does not ship it (only the overlay-builder does). After SD031 rebuilt
# mentorpi-t1 FROM stock, those nodes exited 127 until this apt line.
RUN find /etc/apt \( -name '*.list' -o -name 'sources.list' \) -print0 \
    | xargs -0 sed -i \
        -e 's|https\?://mirrors\.tuna\.tsinghua\.edu\.cn/ubuntu-ports/|http://ports.ubuntu.com/ubuntu-ports/|g' \
        -e 's|https\?://mirrors\.tuna\.tsinghua\.edu\.cn/ros2/ubuntu|http://packages.ros.org/ros2/ubuntu|g' \
    && cp /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/ros2-latest-archive-keyring.gpg \
    && cp /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/ros2-archive-keyring.gpg \
    && if ! grep -RqsE '(^|[[:space:]])multiverse' /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then \
        . /etc/os-release; \
        echo "deb http://ports.ubuntu.com/ubuntu-ports ${VERSION_CODENAME} multiverse" > /etc/apt/sources.list.d/sd031-multiverse.list; \
        echo "deb http://ports.ubuntu.com/ubuntu-ports ${VERSION_CODENAME}-updates multiverse" >> /etc/apt/sources.list.d/sd031-multiverse.list; \
    fi \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-foxglove-bridge \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-imu-complementary-filter \
        ros-humble-rviz2 \
        libasound2 \
        rhvoice \
        rhvoice-russian \
    && rm -rf /var/lib/apt/lists/*
# Isolated colcon prefix from MentorPi third_party_ws (not on overlay AMENT path
# until camera_layer and the unit source local_setup.bash).
COPY --chown=ubuntu:ubuntu vendor-deptrum /home/ubuntu/third_party_ros2/third_party_ws/install/deptrum-ros-driver-aurora930
COPY vendor-opencv /tmp/vendor-opencv
RUN cp -a /tmp/vendor-opencv/libopencv* /usr/local/lib/ \
    && ldconfig \
    && rm -rf /tmp/vendor-opencv
USER ubuntu
RUN python3 -m pip install --user --no-cache-dir pyserial
COPY --chown=ubuntu:ubuntu install /home/ubuntu/mentorpi_t1_ws/install
COPY --chown=ubuntu:ubuntu config /home/ubuntu/mentorpi_t1_ws/config
