#include <controller/controller.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <ros/ros.h>
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller");

    controller::Controller vehicle_controller;

    ros::Subscriber sensor_subscriber = vehicle_controller._n.subscribe("/carla/ego_vehicle/vehicle_info",
                                                              1000,
                                                              &controller::Controller::callback_egoVehicleControl,
                                                              &vehicle_controller);

    while (ros::ok()) {
      vehicle_controller.calculate();

      ros::Publisher control_publisher = vehicle_controller._n.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1000);
      control_publisher.publish(vehicle_controller.get_msg());

      ros::spinOnce();
    }

    return 0;
}
