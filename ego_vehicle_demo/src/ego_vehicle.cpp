#include <ego_vehicle_demo/ego_vehicle.h>
#include <ros/console.h>
#include <ros/exception.h>
#include <ros/ros.h>

#include <random>

#define EXPECT_TRUE(pred)            \
    if (!(pred)) {                   \
        throw ros::Exception(#pred); \
    }

using namespace std::chrono_literals;

namespace {
/**
 * @brief Picks a random element from a container list
 *      The container list must overload operator[]
 * 
 * @tparam RangeT:      container list type 
 * @tparam RNG:         random number generator type
 * @param range:        container list (iterable, operator[] overloaded)
 * @param generator:    random number generator
 * @return auto&:       an element in the container list
 */
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
    EXPECT_TRUE(range.size() > 0u);
    std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
    return range[dist(std::forward<RNG>(generator))];
}
}  // namespace

namespace ego_vehicle {

EgoVehicle::EgoVehicle(unsigned int rng_seed)
    : _rng(rng_seed),
      _n(),
      _host("localhost"),
      _port(2000u),
      _subscriberTimeOut(1.0),
      _clientTimeOut(10s) {
}

void EgoVehicle::restart(const carla::client::World &world) {
    /* Ego Vehicle Blueprint Configuration */
    // Get a random vehicle blueprint
    auto p_blueprintLibrary = world.GetBlueprintLibrary();
    auto p_listVehicles = p_blueprintLibrary->Filter("vehicle");
    auto blueprint = RandomChoice(*p_listVehicles, _rng);

    // Randomly config the blueprint
    if (blueprint.ContainsAttribute("color")) {
        auto &attribute = blueprint.GetAttribute("color");
        blueprint.SetAttribute("color", RandomChoice(attribute.GetRecommendedValues(), _rng));
    }
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

    this->restart(world);
}

}  // namespace ego_vehicle
