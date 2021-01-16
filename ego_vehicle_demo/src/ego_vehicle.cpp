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
    : _n(),
      role_name(_role_name),
      _rng(rng_seed),
      _role_name("ego_vehicle"),
      _host("localhost"),
      _port(2000u),
      _subscriberTimeOut(1.0),
      _clientTimeOut(10s),
      _spawn_point(),
      _vehicle_ptr(),
      _camera_rgb_ptr(),
      _camera_seg_ptr(),
      _camera_depth_ptr(),
      _lidar_ptr(),
      _control() {
    // _control.throttle = 1.0f;
}

void EgoVehicle::destroy() {
    if (_vehicle_ptr != nullptr) {
        _vehicle_ptr->Destroy();
        ROS_INFO("Destroyed vehicle on exit");
    }
    if (_camera_rgb_ptr != nullptr) {
        _camera_rgb_ptr->Destroy();
        ROS_INFO("Destroyed RGB camera on exit");
    }
    if (_camera_seg_ptr != nullptr) {
        _camera_seg_ptr->Destroy();
        ROS_INFO("Destroyed segmentation camera on exit");
    }
    if (_camera_depth_ptr != nullptr) {
        _camera_depth_ptr->Destroy();
        ROS_INFO("Destroyed depth camera on exit");
    }
}

void EgoVehicle::restart(carla::client::World &world) {
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
    if (blueprint.ContainsAttribute("role_name")) {
        blueprint.SetAttribute("role_name", _role_name);
    }

    /* Spawns Ego Vehicle */
    auto p_map = world.GetMap();
    carla::geom::Transform transform = RandomChoice(p_map->GetRecommendedSpawnPoints(), _rng);
    _spawn_point = transform;

    // spawns `actor`
    carla::traffic_manager::ActorPtr p_actor = world.SpawnActor(blueprint, transform);
    ROS_INFO("Spawned %s", _role_name.c_str());

    _vehicle_ptr = boost::static_pointer_cast<carla::client::Vehicle>(p_actor);

    /* Move Spectator in World */
    carla::traffic_manager::ActorPtr p_spectator = world.GetSpectator();
    transform.location += 32.0f * transform.GetForwardVector();
    transform.location.z += 2.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -15.0f;
    p_spectator->SetTransform(transform);

    this->setupSensors(world);

    /* Apply control to vehicle */
    _vehicle_ptr->ApplyControl(_control);
}

void EgoVehicle::setupSensors(carla::client::World &world) {
    auto p_blueprintLibrary = world.GetBlueprintLibrary();
    /* Spawns Front RGB Camera */
    auto p_listCameras = p_blueprintLibrary->Filter("sensor.camera.rgb");
    carla::client::ActorBlueprint cameraRGBBlueprint = RandomChoice(*p_listCameras, _rng);
    if (cameraRGBBlueprint.ContainsAttribute("role_name"))
        cameraRGBBlueprint.SetAttribute("role_name", "front");

    carla::geom::Transform cameraRGBTransform = carla::geom::Transform(
        carla::geom::Location(2.0f, 0.0f, 2.0f),  // x,y,z
        carla::geom::Rotation(0.0f, 0.0f, 0.0f)   // pitch, roll, yaw
    );
    carla::client::ActorPtr cameraRGBActor = world.SpawnActor(cameraRGBBlueprint, cameraRGBTransform, _vehicle_ptr.get());
    _camera_rgb_ptr = boost::static_pointer_cast<carla::client::Sensor>(cameraRGBActor);

    /* Spawns Front Semantic Segmentation Camera */
    p_listCameras = p_blueprintLibrary->Filter("sensor.camera.semantic_segmentation");
    carla::client::ActorBlueprint cameraSegBlueprint = RandomChoice(*p_listCameras, _rng);
    if (cameraSegBlueprint.ContainsAttribute("role_name"))
        cameraSegBlueprint.SetAttribute("role_name", "front");

    carla::geom::Transform cameraSegTransform = carla::geom::Transform(
        carla::geom::Location(2.0f, 0.0f, 2.0f),  // x,y,z
        carla::geom::Rotation(0.0f, 0.0f, 0.0f)   // pitch, roll, yaw
    );
    carla::client::ActorPtr cameraSegActor = world.SpawnActor(cameraSegBlueprint, cameraSegTransform, _vehicle_ptr.get());
    _camera_seg_ptr = boost::static_pointer_cast<carla::client::Sensor>(cameraSegActor);

    /* Spawns Front Depth Camera */
    p_listCameras = p_blueprintLibrary->Filter("sensor.camera.depth");
    carla::client::ActorBlueprint cameraDepthBlueprint = RandomChoice(*p_listCameras, _rng);
    if (cameraDepthBlueprint.ContainsAttribute("role_name"))
        cameraDepthBlueprint.SetAttribute("role_name", "front");

    carla::geom::Transform cameraDepthTransform = carla::geom::Transform(
        carla::geom::Location(2.0f, 0.0f, 2.0f),  // x,y,z
        carla::geom::Rotation(0.0f, 0.0f, 0.0f)   // pitch, roll, yaw
    );
    carla::client::ActorPtr cameraDepthActor = world.SpawnActor(cameraDepthBlueprint, cameraDepthTransform, _vehicle_ptr.get());
    _camera_depth_ptr = boost::static_pointer_cast<carla::client::Sensor>(cameraDepthActor);

    /* Spawns Lidar */
    auto p_listLidar = p_blueprintLibrary->Filter("sensor.lidar.ray_cast");
    carla::client::ActorBlueprint lidarBlueprint = RandomChoice(*p_listLidar, _rng);
    if (lidarBlueprint.ContainsAttribute("role_name"))
        lidarBlueprint.SetAttribute("role_name", "lidar0");
    carla::geom::Transform lidarTransform = carla::geom::Transform(
        carla::geom::Location(0.f, 0.f, 2.4f),
        carla::geom::Rotation(0.f, 0.f, 0.f));
    carla::client::ActorPtr lidarActor = world.SpawnActor(lidarBlueprint, lidarTransform, _vehicle_ptr.get());
    _lidar_ptr = boost::static_pointer_cast<carla::client::Sensor>(lidarActor);
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

void EgoVehicle::callback_egoVehicleControl(const carla_msgs::CarlaEgoVehicleControl &control_msg) {
    ROS_INFO("received vehicle_control_cmd");

    _control.throttle = control_msg.throttle;
    _control.steer = control_msg.steer;
    _control.brake = control_msg.brake;
    _control.hand_brake = control_msg.hand_brake;
    _control.reverse = control_msg.reverse;
    _control.gear = control_msg.gear;
    _control.manual_gear_shift = control_msg.manual_gear_shift;

    _vehicle_ptr->ApplyControl(_control);
}

}  // namespace ego_vehicle
