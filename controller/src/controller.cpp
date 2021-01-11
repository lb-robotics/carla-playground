#include <controller/controller.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <ros/ros.h>

namespace controller {

Controller::Controller(): sensor_info(), msg() {
    msg.throttle = 1.0f;
}

void Controller::calculate() {
  msg.throttle = 1.0f; // 0. <= throttle <= 1. float32 throttle
  msg.steer = 0.0f; // -1. <= steer <= 1. float32 steer
  msg.brake = 0.0f; // 0. <= brake <= 1. float32 brake
  msg.hand_brake = false; // hand_brake 0 or 1 bool hand_brake
  msg.reverse = false; // reverse 0 or 1 bool reverse
  msg.gear = 0; // gear int32 gear
  msg.manual_gear_shift = false; // manual gear shift bool manual_gear_shift
}

void Controller::callback_egoVehicleControl(const controller::control_info &sensor_msg) {
    sensor_info.velocity = sensor_msg.velocity;
    sensor_info.acceleration = sensor_msg.acceleration;
    sensor_info.orientation = sensor_msg.orientation;
    sensor_info.prev_distance = sensor_msg.prev_distance;
}

carla_msgs::CarlaEgoVehicleControl Controller::get_msg() {
    return msg;
}

}  // namespace controller
