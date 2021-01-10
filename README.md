# CARLA Playground
This repo serves as a guide to installing and testing *CARLA* & *CARLA ROS Bridge* on Ubuntu 20.04 LTS. 

1. **This repo itself contains several ROS packages** and can be placed in any Catkin workspace by
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:lb-robotics/carla-playground.git
cd ..
catkin init
catkin config --install
catkin build
```
Notice that **the use of `catkin_tools` is necessary to build the workspace.**

2. **This repo depends on self-developed LibCarla (CARLA C++ libraries, details see [here](https://github.com/lb-robotics/libcarla)).**
3. Due to the CMake structure of LibCarla, this repo can only be built wth `catkin_tools 0.5.0` with a proper **install space**:
```bash
catkin config --install
```



- [CARLA Playground](#carla-playground)
  - [Overview](#overview)
  - [System & Software Requirements](#system--software-requirements)
  - [CARLA Build Dependencies](#carla-build-dependencies)
  - [Unreal Engine 4.24](#unreal-engine-424)
  - [CARLA 0.9.10.1](#carla-09101)
    - [Clone Repository](#clone-repository)
    - [Get Assets](#get-assets)
    - [Set Environment Variable](#set-environment-variable)
    - [Building CARLA](#building-carla)
    - [Some Tweaks on CARLA Window](#some-tweaks-on-carla-window)
    - [Running Examples](#running-examples)
  - [CARLA ROS Bridge 0.9.10.1](#carla-ros-bridge-09101)
  - [References](#references)

## Overview
CARLA is an open-source simulator for autonomous driving research and is still in active development. However, when it comes to the brand-new Ubuntu 20.04 LTS, CARLA has not released an official Debian package for users to install conveniently. Thus, users who wish to install CARLA on Ubuntu 20.04 LTS have to build it from source. This article will go through the entire process of how to build CARLA from source on Ubuntu 20.04 LTS.

## System & Software Requirements
- **Ubuntu 20.04 LTS**
- **140 GB disk space.** Although the [official Wiki](https://carla.readthedocs.io/en/latest/build_linux/#linux-build-command-summary) says it only takes up to 30-50 GB disk space, unfortunately when I was compiling UnrealEngine I have spent over 100 GB disk space on it.
- **CARLA 0.9.10.1, UnrealEngine 4.24, ROS Noetic Ninjemys.** This article will focus on how to compile the current-latest *CARLA 0.9.10.1* along with *UnrealEngine 4.24.*
- Python 3.8

## CARLA Build Dependencies
[Please refer to the official wiki here.](https://carla.readthedocs.io/en/latest/build_linux/#dependencies)

My experience is a little bit difference from the official wiki. I was not able to find and install Ubuntu Xenial's LLVM tool on 20.04 LTS, and thus I used Ubuntu Focal's default apt repository for `clang-8` installation. What's more, ROS Noetic has removed all support for Python 2, so we don't need to build anything for Python 2 either.

```bash
sudo apt-get update &&
sudo apt-get install wget software-properties-common &&
sudo add-apt-repository ppa:ubuntu-toolchain-r/test &&
sudo apt-get update
```

```bash
sudo apt-get install build-essential clang-8 g++-7 cmake ninja-build libvulkan1 python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev &&
pip3 install --user -Iv setuptools==47.3.1 &&
pip3 install --user distro
```

## Unreal Engine 4.24
For CARLA 0.9.10.1, we are supposed to install UnrealEngine 4.24 only. Create an [UnrealEngine account](https://www.unrealengine.com/en-US/feed) and link your GitHub account to it to get access to the UnrealEngine developer GitHub repository. Then, clone UnrealEngine 4.24 release only onto your local machine. 
```bash
git clone --depth=1 -b 4.24 https://github.com/EpicGames/UnrealEngine.git ~/UnrealEngine_4.24
```

Download a patch for Unreal Engine. This patch fixes some Vulkan visualization issues that may occur when changing the map. Download and install it with the following commands.
```bash
cd ~/UnrealEngine_4.24
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/UE_Patch/430667-13636743-patch.txt 430667-13636743-patch.txt
patch --strip=4 < 430667-13636743-patch.txt
```

Build UnrealEngine 4.24 with the following commands. This may take up to 2 hours for the build.
```bash
./Setup.sh && ./GenerateProjectFiles.sh && make
```

Check your build of UnrealEngine 4.24 by trying to open UnrealEngine Editor.
```bash
cd ~/UnrealEngine_4.24/Engine/Binaries/Linux && ./UE4Editor
```

## CARLA 0.9.10.1
### Clone Repository
Building CARLA 0.9.10.1 requires to clone CARLA repository onto local machine. After clone, checkout the 0.9.10.1 tag.
```bash
git clone https://github.com/carla-simulator/carla
git checkout tags/0.9.10.1
```

### Get Assets
Download the assets that are necessary to build CARLA. 
```bash
cd ~/carla
./Update.sh
```

### Set Environment Variable
CARLA build tool finds the root folder of UnrealEngine 4.24 by setting the following environment variable:
```bash
export UE4_ROOT=~/UnrealEngine_4.24
```

If you wish to set the environment variable permanently, add it to your `~/.bashrc`.

### Building CARLA
CARLA build contains the following options:

- **make PythonAPI** compiles the API client, necessary to grant control over the simulation. It is only needed the first time. Remember to run it again when updating CARLA. Scripts will be able to run after this command is executed.
```bash
make PythonAPI
```

- **make launch** compiles the server simulator and launches Unreal Engine. Press **Play** to start the spectator view and close the editor window to exit. Camera can be moved with WASD keys and rotated by clicking the scene while moving the mouse around.
```bash
make launch
```

- **Creating a packaged version for CARLA** helps to start the server much faster and more conveniently. After creating the package, the starting script of CARLA can be found in `~/carla/Dist/CARLA_Shipping_0.9.10.1/LinuxNoEditor/CarlaUE4.sh`.
```bash
make package
```

A working version of CARLA 0.9.10.1 build can be found using this Google Link: https://drive.google.com/file/d/1XYc6xtr0X1xM0U4nHkGW5NnG_O0iSunv/view?usp=sharing. Users can download and extract the `tar` file, and run 
```bash
./CarlaUE4.sh
```

### Some Tweaks on CARLA Window
1. If CARLA consumes a lot of GPU resources and you would like to set limit on that, you can manually specify window size of CARLA window:
```bash
./CarlaUE4.sh -windowed -ResX=1080 -ResY=720
```
Thanks [this tutorial](https://silvamfpedro.github.io/thesis-blog/manual.html) for providing this information!

2. CARLA running in the background can be achieved by this command:
```bash
SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl
```

### Running Examples
There are a lot of useful examples of CARLA Python API under `~/carla/PythonAPI/examples`. Feel free to check them out!

## CARLA ROS Bridge 0.9.10.1
To install ROS Bridge 0.9.10.1, following the [official instructions](https://carla.readthedocs.io/en/latest/ros_installation/#b-using-source-repository).



## References
_CARLA: An Open Urban Driving Simulator_<br>Alexey Dosovitskiy, German Ros,
Felipe Codevilla, Antonio Lopez, Vladlen Koltun; PMLR 78:1-16
[[PDF](http://proceedings.mlr.press/v78/dosovitskiy17a/dosovitskiy17a.pdf)]
[[talk](https://www.youtube.com/watch?v=xfyK03MEZ9Q&feature=youtu.be&t=2h44m30s)]
[[GitHub](https://github.com/carla-simulator/carla)]

*Official CARLA Documentation Website:* https://carla.readthedocs.io/en/latest/build_linux/

