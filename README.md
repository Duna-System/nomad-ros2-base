# Nomad ROS2 Base with Docker
This repository contains the tools for generating a docker image for developing NOMAD ROS2 applications. Requires NVIDIA AARCH64.

## Commands
`docker build . -t duna-nomad-base` for Jetson
`docker build . -t duna-nomad-base -f Dockerfile.amd64` for x86
