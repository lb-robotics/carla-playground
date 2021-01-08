#include <ego_vehicle_demo/ego_vehicle.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ego_vehicle_demo");

    ego_vehicle::EgoVehicle vehicle;

    while (ros::ok()) {
        try {
            vehicle.run();
        } catch (const ros::Exception& e) {
            ROS_FATAL("%s", e.what());
            ros::shutdown();
        }

        ros::spinOnce();
    }

    return 0;
}
