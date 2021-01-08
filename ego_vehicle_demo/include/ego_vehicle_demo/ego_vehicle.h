#ifndef __EGO_VEHICLE_H__
#define __EGO_VEHICLE_H__

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla_msgs/CarlaWorldInfo.h>
#include <ros/ros.h>

#include <random>
#include <stdexcept>
#include <string>

namespace ego_vehicle {

class EgoVehicle {
public:
    /**
     * @brief Construct a new Ego Vehicle object with RNG (C++ random number generator) seed 
     * 
     * @param rng_seed: seed for std::mt19937_64 RNG
     */
    EgoVehicle(unsigned int rng_seed);

    // (Re)spawns ego vehicle
    void restart(const carla::client::World& world);

    // Ego vehicle runner
    void run();

private:
    // Mersenne Twister 19937 generator (64 bit)
    //  Details see: https://www.cplusplus.com/reference/random/mt19937_64/
    //  Referenced at: cpp_client_demo/demo_main.cpp:69
    //
    std::mt19937_64 _rng;

    // ROS node handle (for subscribing messages)
    ros::NodeHandle _n;

    // CARLA utilities (host, port, timeout)
    std::string _host;
    uint16_t _port;
    ros::Duration _subscriberTimeOut;
    carla::time_duration _clientTimeOut;
};

}  // namespace ego_vehicle

#endif /* __EGO_VEHICLE_H__ */
