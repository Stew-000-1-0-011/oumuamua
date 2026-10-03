# ROS 2 Lyrical Luth (Ubuntu 26.04, GCC 15) development image for oumuamua.
ARG BASE_IMAGE=ros:lyrical
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive

# Resolve package dependencies at image build time so the container is ready to build.
COPY src /tmp/oumuamua_src
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      ccache \
      libboost-thread-dev \
      git \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths /tmp/oumuamua_src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
 && rm -rf /tmp/oumuamua_src /var/lib/apt/lists/*

# sotoba (header-only, not in rosdep) for sotoba_ros. Its CMake also installs Sophus >= 1.24.6
# into the same prefix, since ROS's ros-lyrical-sophus (1.22) is too old.
ARG SOTOBA_REPO=https://github.com/Stew-000-1-0-011/sotoba.git
ARG SOTOBA_REF=main
RUN git clone --depth 1 --branch "${SOTOBA_REF}" "${SOTOBA_REPO}" /tmp/sotoba \
 && cmake -S /tmp/sotoba -B /tmp/sotoba/build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/sotoba -DBUILD_DEVS=OFF -DBUILD_EXAMPLES=OFF \
      -DBUILD_STATIC_LIB=OFF -DBUILD_SHARED_LIB=OFF -DBUILD_MODULE_LIB=OFF \
 && cmake --build /tmp/sotoba/build --target install \
 && rm -rf /tmp/sotoba
ENV CMAKE_PREFIX_PATH=/opt/sotoba

ENV COLCON_DEFAULTS_FILE=/ws/colcon_defaults.yaml
WORKDIR /ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc
