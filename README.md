# CARLA Playground
This repo serves as a guide to installing and testing *CARLA* & *CARLA ROS Bridge* on Ubuntu 20.04.

## Overview
CARLA is an open-source simulator for autonomous driving research and is still in active development. However, when it comes to the brand-new Ubuntu 20.04, CARLA has not released an official Debian package for users to install conveniently. Thus, users who wish to install CARLA on Ubuntu 20.04 LTS have to build it from source. This article will go through the entire process of how to build CARLA from source on Ubuntu 20.04 LTS.

## System & Software Requirements
- **Ubuntu 20.04 LTS**
- **140 GB disk space.** Although the [official Wiki](https://carla.readthedocs.io/en/latest/build_linux/#linux-build-command-summary) says it only takes up to 30-50 GB disk space, unfortunately when I was compiling UnrealEngine I have spent over 100 GB disk space on it.
- **CARLA 0.9.10.1, UnrealEngine 4.24, ROS Noetic Ninjemys.** This article will focus on how to compile the current-latest CARLA 0.9.10.1 along with UnrealEngine 4.24.
- Python 3.8

## Dependencies
[Please refer to the official wiki here.](https://carla.readthedocs.io/en/latest/build_linux/#dependencies)

My experience is a little bit difference from the official wiki. Unfortunately I was not able to find and install Ubuntu Xenial's LLVM tool on 20.04 LTS, and thus I used Ubuntu Focal's default 

## References
_CARLA: An Open Urban Driving Simulator_<br>Alexey Dosovitskiy, German Ros,
Felipe Codevilla, Antonio Lopez, Vladlen Koltun; PMLR 78:1-16
[[PDF](http://proceedings.mlr.press/v78/dosovitskiy17a/dosovitskiy17a.pdf)]
[[talk](https://www.youtube.com/watch?v=xfyK03MEZ9Q&feature=youtu.be&t=2h44m30s)]
[[GitHub](https://github.com/carla-simulator/carla)]

*Official CARLA Documentation Website:* https://carla.readthedocs.io/en/latest/build_linux/

