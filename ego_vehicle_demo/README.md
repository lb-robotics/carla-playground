# CARLA Ego Vehicle Demo in C++
This node demonstrates how to construct a CARLA vehicle with camera sensors in C++.

## Usage
In a new terminal, run
```bash
rosrun ego_vehicle_demo ego_vehicle_demo
```

**Notice.** This node must be built using `catkin_tools` with a specified install space:
```bash
catkin config --install
```

## Dependencies
This package depends on the following ROS packages:
- `carla-ros-bridge`, ver. 0.9.10.1, retrieved [here](https://github.com/carla-simulator/ros-bridge/releases/tag/0.9.10.1)
- LibCarla, ver. 0.9.10.1, retrieved [here](https://github.com/lb-robotics/libcarla/releases/tag/0.9.10.1)
- `geometry_msgs`
- `tf`
- `roscpp`

## Overview
CARLA uses "actors" to perceive and act in the world. Vehicles, Camera, LiDARs, are all actors. **Actors must be explicitly destroyed** (aka. call `carla::client::Actor.Destroy()`) upon node exit. When one create a vehicle in the world, it is recommended to specify its *role name*, which will be reflected in the ROS topics. All sensors attached to this vehicle will automatically publish ROS messages under `/carla/[<VEHICLE ROLE NAME>]/...`. For more details, please use `rostopic list` to examine when running this node.

This node also uses `CarlaEgoVehicleControl` to control the vehicle in the world. The node has already subscribed to the topic `/carla/ego_vehicle/vehicle_control_cmd` for this message.


