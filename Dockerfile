FROM nvcr.io/nvidia/l4t-cuda:11.4.19-devel

ENV ROS_DISTRO=humble
ENV ROS_ROOT=/ros
ENV DEPS_ROOT=/deps
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash

WORKDIR ${ROS_ROOT}

SHELL ["/bin/bash", "-c"] 

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Zip is for debugging.
RUN apt update && apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools \
  libjsoncpp-dev \
  zip

# install some pip packages needed for testing. SMBUS is for I2C
RUN python3 -m pip install -U \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   "pytest>=5.3" \
   pytest-repeat \
   pytest-rerunfailures \
   smbus

# Install new Cmake
WORKDIR ${DEPS_ROOT}/cmake
RUN  wget https://github.com/Kitware/CMake/releases/download/v3.23.2/cmake-3.23.2-linux-aarch64.sh && \
  chmod +x cmake-3.23.2-linux-aarch64.sh && \
  yes | ./cmake-3.23.2-linux-aarch64.sh
ENV PATH=${DEPS_ROOT}/cmake/cmake-3.23.2-linux-aarch64/bin/:${PATH}

# RUN apt-get update --fix-missing
# Compile & Install PCL
RUN apt-get install -y \
  libeigen3-dev \
  libboost-all-dev \
  libflann-dev

WORKDIR ${DEPS_ROOT}
RUN git clone https://github.com/google/googletest.git -b v1.12.0 && \
  mkdir -p googletest/build && cd googletest/build && \
  cmake .. -DCMAKE_BUILD_TYPE=Release && \
  make -j6 install

RUN git clone https://github.com/PointCloudLibrary/pcl.git -b pcl-1.13.1 && \
  mkdir -p pcl/build && cd pcl/build && \
  cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_visualization=OFF -DWITH_VTK=OFF -DBUILD_ml=OFF -DWITH_OPENGL=OFF && \
  make -j2 install

# Get ROS dependencies
COPY ros2.repos ${ROS_ROOT}

RUN mkdir -p ${ROS_ROOT}/src && \
  cd ${ROS_ROOT} && \
  vcs import src < ros2.repos && \
  apt-get upgrade -y && \
  rosdep init && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers libpcl-dev pcl_ros" && \
  apt-get clean

RUN apt-get remove -y libpcl-dev

# build it!
RUN cd ${ROS_ROOT} && \ 
  colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source it
RUN echo "source ${ROS_ROOT}/install/setup.bash" >> /root/.bashrc

# remove build files
RUN rm -rf ${ROS_ROOT}/src && \
rm -rf ${ROS_ROOT}/logs && \
rm -rf ${ROS_ROOT}/build && \
rm -rf ${DEPS_ROOT}/pcl && \
rm -rf ${DEPS_ROOT}/googletest && \
rm -rf /var/lib/apt/lists/* && \
apt-get autoremove -y && apt-get clean

WORKDIR /

