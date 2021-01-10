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
      _clientTimeOut(10s),
      _spawn_point(),
      _vehicle_ptr(),
      _control() {
    _control.throttle = 1.0f;
}

void EgoVehicle::restart(carla::client::World &world) {
    // if no vehicle is spawned (1st time in the loop), spawn a vehicle
    if (_vehicle_ptr == nullptr) {
        /* Ego Vehicle Blueprint Configuration */
        // Get a random vehicle blueprint
        auto p_blueprintLibrary = world.GetBlueprintLibrary();
        auto p_listVehicles = p_blueprintLibrary->Filter("vehicle");
        carla::client::ActorBlueprint blueprint = RandomChoice(*p_listVehicles, _rng);

        // Randomly config the blueprint
        if (blueprint.ContainsAttribute("color")) {
            auto &attribute = blueprint.GetAttribute("color");
            blueprint.SetAttribute("color", RandomChoice(attribute.GetRecommendedValues(), _rng));
        }

        /* Spawns Ego Vehicle */
        auto p_map = world.GetMap();
        carla::geom::Transform transform = RandomChoice(p_map->GetRecommendedSpawnPoints(), _rng);
        _spawn_point = transform;

        // spawns `actor`
        carla::traffic_manager::ActorPtr p_actor = world.SpawnActor(blueprint, transform);
        ROS_INFO("Spawned %d", p_actor->GetId());

        _vehicle_ptr = boost::static_pointer_cast<carla::client::Vehicle>(p_actor);

        /* Move Spectator in World */
        carla::traffic_manager::ActorPtr p_spectator = world.GetSpectator();
        transform.location += 32.0f * transform.GetForwardVector();
        transform.location.z += 2.0f;
        transform.rotation.yaw += 180.0f;
        transform.rotation.pitch = -15.0f;
        p_spectator->SetTransform(transform);
    }

    /* Apply control to vehicle */
    _vehicle_ptr->ApplyControl(_control);
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
