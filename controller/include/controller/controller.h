#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

// #include <carla/client/ActorBlueprint.h>
// #include <carla/client/BlueprintLibrary.h>
// #include <carla/client/Client.h>
// #include <carla/client/Map.h>
// #include <carla/client/Sensor.h>
// #include <carla/client/TimeoutException.h>
// #include <carla/client/World.h>
// #include <carla/geom/Transform.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
// #include <carla_msgs/CarlaWorldInfo.h>
#include <ros/ros.h>
#include <controller/control_info.h>

#include <memory>
#include <random>
#include <stdexcept>
#include <string>

namespace controller {

class Controller {
public:
    // ROS node handle (for subscribing messages)
    ros::NodeHandle _n;

    // Constuctor
    Controller();

    // Controller input calculator
    void calculate();

    // Subscriber handler
    void callback_egoVehicleControl(const controller::control_info &sensor_msg);

    // get the message
    carla_msgs::CarlaEgoVehicleControl get_msg();

private:
    controller::control_info sensor_info;
    carla_msgs::CarlaEgoVehicleControl msg;
};

}  // namespace controller

#endif /* __CONTROLLER_H__ */
