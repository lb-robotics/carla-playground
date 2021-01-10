#include <ego_vehicle_demo/ego_vehicle.h>
#include <ros/ros.h>

#include <random>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ego_vehicle_demo");

    ego_vehicle::EgoVehicle vehicle((std::random_device())());

    ros::Subscriber control_subscriber = vehicle._n.subscribe("/carla/ego_vehicle/vehicle_control_cmd",
                                                              1000,
                                                              &ego_vehicle::EgoVehicle::callback_egoVehicleControl,
                                                              &vehicle);

    try {
        vehicle.run();
    } catch (const ros::Exception& e) {
        ROS_FATAL("%s", e.what());
        ros::shutdown();
    }

    while (ros::ok()) {
        ros::spinOnce();
    }

    vehicle.destroy();

    return 0;
}
