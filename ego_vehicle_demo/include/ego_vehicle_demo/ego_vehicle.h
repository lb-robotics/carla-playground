#ifndef __EGO_VEHICLE_H__
#define __EGO_VEHICLE_H__

#include <carla/client/Client.h>
#include <carla_msgs/CarlaWorldInfo.h>
#include <ros/ros.h>

#include <stdexcept>
#include <string>

namespace ego_vehicle {

class EgoVehicle {
public:
    EgoVehicle();

    // Ego vehicle runner
    void run();

private:
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
