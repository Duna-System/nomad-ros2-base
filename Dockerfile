# This image already has ROS humble base installed.
FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

ENV ROS_EXTRA_ROOT=/ros_extra
ENV DEPS_ROOT=/deps

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash

WORKDIR ${ROS_EXTRA_ROOT}

SHELL ["/bin/bash", "-c"]

# Get dependencies for PCl and SLAM.
RUN apt update && apt install -y \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-good1.0-dev \
  gstreamer1.0-tools \ 
  libeigen3-dev \
  libboost-all-dev \
  libflann-dev \
  libjsoncpp-dev \
  python3-pip \
  libusb-1.0-0-dev \
  wget \
  zip

# install some pip packages needed for testing. SMBUS is for I2C
RUN python3 -m pip install -U \
  smbus

# Install new CMAKE
WORKDIR ${DEPS_ROOT}/cmake
RUN  wget https://github.com/Kitware/CMake/releases/download/v3.23.2/cmake-3.23.2-linux-aarch64.sh && \
  chmod +x cmake-3.23.2-linux-aarch64.sh && \
  yes | ./cmake-3.23.2-linux-aarch64.sh
ENV PATH=${DEPS_ROOT}/cmake/cmake-3.23.2-linux-aarch64/bin/:${PATH}

WORKDIR ${DEPS_ROOT}
RUN git clone https://github.com/google/googletest.git -b v1.12.0 && \
  mkdir -p googletest/build && cd googletest/build && \
  cmake .. -DCMAKE_BUILD_TYPE=Release && \
  make -j6 install

RUN git clone https://github.com/PointCloudLibrary/pcl.git -b pcl-1.13.1 && \
  mkdir -p pcl/build && cd pcl/build && \
  cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_visualization=OFF -DWITH_VTK=OFF -DBUILD_ml=OFF -DWITH_OPENGL=OFF && \
  make -j2 install

# Build extra ROS packages
COPY ros2.repos ${ROS_EXTRA_ROOT}

# Skip keys apparently does not work..
RUN mkdir -p ${ROS_EXTRA_ROOT}/src && \
  cd ${ROS_EXTRA_ROOT} && \
  vcs import src < ros2.repos && \
  source /opt/ros/humble/install/setup.bash && rosdep install --from-paths src --ignore-src -y --skip-keys "libpcl-dev" && \ 
  apt purge -y libpcl-dev && apt autoremove -y

RUN cd ${ROS_EXTRA_ROOT} && \ 
  source /opt/ros/humble/install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source it
RUN echo "source ${ROS_EXTRA_ROOT}/install/setup.bash" >> /root/.bashrc

# remove build files
RUN rm -rf ${ROS_EXTRA_ROOT}/src && \
  rm -rf ${ROS_EXTRA_ROOT}/logs && \
  rm -rf ${ROS_EXTRA_ROOT}/build && \
  rm -rf ${DEPS_ROOT}/pcl && \
  rm -rf ${DEPS_ROOT}/googletest && \
  rm -rf /var/lib/apt/lists/* && \
  apt-get autoremove -y && apt-get clean

WORKDIR /

