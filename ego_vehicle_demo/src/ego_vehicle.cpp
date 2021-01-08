#include <ego_vehicle_demo/ego_vehicle.h>
#include <ros/console.h>
#include <ros/exception.h>
#include <ros/ros.h>

using namespace std::chrono_literals;
namespace ego_vehicle {

EgoVehicle::EgoVehicle()
    : _n(),
      _host("localhost"),
      _port(2000u),
      _subscriberTimeOut(1.0),
      _clientTimeOut(10s) {
}

void EgoVehicle::run() {
    ROS_INFO("Waiting for CARLA world (topic: /carla/world_info)...");
    auto p_carlaWorldInfo =
        ros::topic::waitForMessage<carla_msgs::CarlaWorldInfo>("/carla/world_info", _n, _subscriberTimeOut);

    if (p_carlaWorldInfo == nullptr) {
        throw ros::Exception("Timed out waiting for CARLA world info");
    }

    ROS_INFO("CARLA world info received");

    carla::client::Client client = carla::client::Client(_host, _port);
    client.SetTimeout(_clientTimeOut);

    std::string town_name = p_carlaWorldInfo->map_name;
    ROS_INFO("Loading world %s", town_name.c_str());
    carla::client::World world = client.LoadWorld(town_name);
}

}  // namespace ego_vehicle
