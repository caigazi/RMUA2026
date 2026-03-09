FROM osrf/ros:noetic-desktop-full-focal

ADD src /basic_dev/src/
ADD setup.bash /

RUN chmod +x /setup.bash

USER root

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    ros-noetic-geographic-msgs \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    libopencv-dev \
    net-tools \
 && rm -rf /var/lib/apt/lists/*

# 关键：把 apt 安装的 OpenCV CMake 配置“对齐”到仓库写死的 OpenCV_DIR=/usr/local/lib/cmake/opencv4
RUN set -eux; \
    mkdir -p /usr/local/lib/cmake; \
    if [ -d /usr/lib/x86_64-linux-gnu/cmake/opencv4 ]; then \
      ln -sfn /usr/lib/x86_64-linux-gnu/cmake/opencv4 /usr/local/lib/cmake/opencv4; \
    elif [ -d /usr/lib/cmake/opencv4 ]; then \
      ln -sfn /usr/lib/cmake/opencv4 /usr/local/lib/cmake/opencv4; \
    else \
      echo "ERROR: cannot find opencv4 cmake dir"; \
      find /usr -maxdepth 5 -type f -name OpenCVConfig.cmake -o -name opencv-config.cmake || true; \
      exit 1; \
    fi

WORKDIR /basic_dev/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && catkin_make --only-pkg-with-deps airsim_ros \
 && . devel/setup.sh \
 && catkin_make --only-pkg-with-deps basic_dev

ENTRYPOINT [ "/setup.bash" ]
