FROM nvcr.io/nvidia/l4t-cuda:10.2.460-runtime

ARG ROS_PKG=ros_base
ENV ROS_DISTRO=galactic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
  build-essential \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
RUN python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools

RUN python3 -m pip install -U importlib-metadata importlib-resources
# Install new Cmake
RUN mkdir -p /opt/cmake && cd /opt/cmake && \
  wget https://github.com/Kitware/CMake/releases/download/v3.23.2/cmake-3.23.2-linux-aarch64.sh && \
  chmod +x cmake-3.23.2-linux-aarch64.sh && \
  yes | ./cmake-3.23.2-linux-aarch64.sh && \
  export PATH=/opt/cmake/cmake-3.23.2-linux-aarch64/bin/:$PATH

RUN ls -l /opt/cmake/cmake-3.23.2-linux-aarch64/bin/

RUN cmake --version
# Compile ROS
RUN mkdir -p ${ROS_ROOT}/src && \
  cd ${ROS_ROOT} && \
  wget https://raw.githubusercontent.com/Marcus-D-Forte/nomad-ros2-base/master/ros2.repos && \
  ls && \
  vcs import src < ros2.repos && \
  apt upgrade -y && \
  rosdep init && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers" && \
  apt-get clean

# build it!
RUN cd ${ROS_ROOT} && \ 
  colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# remove build files
RUN rm -rf ${ROS_ROOT}/src && \
rm -rf ${ROS_ROOT}/logs && \
rm -rf ${ROS_ROOT}/build && \
rm ${ROS_ROOT}/*.rosinstall

