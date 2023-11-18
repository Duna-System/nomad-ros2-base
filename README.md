# Nomad ROS2 Base with Docker
This repository contains the tools for generating a docker image for developing NOMAD ROS2 applications. Requires NVIDIA AARCH64.

## Commands
`docker build . -t duna-nomad-base` for Jetson
`docker build . -t duna-nomad-base -f Dockerfile.amd64` for x86
`docker build . -t duna-nomad-base-testing -f Dockerfile.testing`


## Running the test container.
The test container has rviz2 installed. To use it, we have to stream GUI apps to the host.
Reference: https://janert.me/guides/running-gui-applications-in-a-docker-container/

Allow connection to Xserver
1. (on host): `xhost +local:`

Run image with bind mounts.

2. (on host) `docker run --rm --net host -v /tmp/.X11-unix:/tmp/.X11-unix -it duna-nomad-base-testing`

