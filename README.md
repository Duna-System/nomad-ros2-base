# Nomad ROS2 Base with Docker
This repository contains the tools for generating a docker image for developing NOMAD ROS2 applications. Requires NVIDIA AARCH64.

## Commands
`docker build . -t marcusforte/nomad-base:arm64` for Jetson
`docker build . -t marcusforte/nomad-base:amd64 -f Dockerfile.amd64` for x86

Then we create a manifest to group images into a single tag. The images have to be availabe in the registry
`docker manifest create marcusforte/nomad-base:latest --amend marcusforte/nomad-base:amd64 marcusforte/nomad-base:arm64`
`docker manifest push marcusforte/nomad-base:latest`
