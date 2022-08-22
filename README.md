# Nomad ROS2 Base with Docker
This repository contains the tools for generating a docker image for developing NOMAD ROS2 applications. Requires NVIDIA AARCH64.

## Commands
`docker build . -t marcusforte/nomad-base` for Jetson (Default)
`docker build . -t marcusforte/nomad-base:x86 -f Dockerfile.x86` for X86
