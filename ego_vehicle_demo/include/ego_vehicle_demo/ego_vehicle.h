#ifndef __EGO_VEHICLE_H__
#define __EGO_VEHICLE_H__

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaWorldInfo.h>
#include <ros/ros.h>

#include <memory>
#include <random>
#include <stdexcept>
#include <string>

namespace ego_vehicle {

class EgoVehicle {
public:
    // ROS node handle (for subscribing messages)
    ros::NodeHandle _n;

    // Read-only reference for role name
    const std::string& role_name;

    /**
     * @brief Construct a new Ego Vehicle object with RNG (C++ random number generator) seed 
     * 
     * @param rng_seed: seed for std::mt19937_64 RNG
     */
    EgoVehicle(unsigned int rng_seed);

    // (Re)spawns ego vehicle
    void restart(carla::client::World& world);

    // Sets up the sensors upon restart
    void setupSensors(carla::client::World& world);

    // Ego vehicle runner
    void run();

    // Ego vehicle destroyer on node exit
    void destroy();

    // Subscriber handler
    void callback_egoVehicleControl(const carla_msgs::CarlaEgoVehicleControl& control_msg);

private:
    // Mersenne Twister 19937 generator (64 bit)
    //  Details see: https://www.cplusplus.com/reference/random/mt19937_64/
    //  Referenced at: cpp_client_demo/demo_main.cpp:69
    //
    std::mt19937_64 _rng;

    // CARLA utilities (host, port, timeout)
    std::string _role_name;
    std::string _host;
    uint16_t _port;
    ros::Duration _subscriberTimeOut;
    carla::time_duration _clientTimeOut;

    // CARLA Class Pointers
    carla::geom::Transform _spawn_point;
    boost::shared_ptr<carla::client::Vehicle> _vehicle_ptr;
    boost::shared_ptr<carla::client::Sensor> _camera_rgb_ptr;
    boost::shared_ptr<carla::client::Sensor> _camera_seg_ptr;
    boost::shared_ptr<carla::client::Sensor> _camera_depth_ptr;
    boost::shared_ptr<carla::client::Sensor> _lidar_ptr;
    carla::client::Vehicle::Control _control;
};

}  // namespace ego_vehicle

#endif /* __EGO_VEHICLE_H__ */
