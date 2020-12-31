#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>

void handleRgbCameraImage(const sensor_msgs::Image::ConstPtr& p_img) {
    ROS_INFO("Image height: %d, width: %d", p_img->height, p_img->width);

    // Referenced at https://answers.ros.org/question/304777/new-in-ros-sensor_msgsimage-in-opencv/
    try {
        cv::imshow("view image", cv_bridge::toCvShare(p_img, sensor_msgs::image_encodings::BGR8)->image);
        cv::waitKey(30);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", p_img->encoding.c_str());
    }
}

void handleDepthCameraImage(const sensor_msgs::Image::ConstPtr& p_img) {
    ROS_INFO("Image height: %d, width: %d", p_img->height, p_img->width);

    // Referenced at https://answers.ros.org/question/304777/new-in-ros-sensor_msgsimage-in-opencv/
    try {
        // FIXME: source image os of encoding TYPE_32FC1, but need to convert to mono8/mono16/bgr8 for display
        //      Seems problematic...
        //
        cv::imshow("view image", cv_bridge::toCvShare(p_img, sensor_msgs::image_encodings::TYPE_32FC1)->image);
        cv::waitKey(30);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", p_img->encoding.c_str());
    }
}

void handleSegmentationCameraImage(const sensor_msgs::Image::ConstPtr& p_img) {
    ROS_INFO("Image height: %d, width: %d", p_img->height, p_img->width);

    // Referenced at https://answers.ros.org/question/304777/new-in-ros-sensor_msgsimage-in-opencv/
    try {
        cv::imshow("view image", cv_bridge::toCvShare(p_img, "bgr8")->image);
        cv::waitKey(30);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", p_img->encoding.c_str());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_subscriber");
    ROS_INFO("Starting camera image subscriber");

    ros::NodeHandle n;

    ros::Subscriber rgbCameraSubscriber =
        n.subscribe("/carla/ego_vehicle/camera/rgb/front/image_color", 1000, handleRgbCameraImage);
    // ros::Subscriber depthCameraSubscriber =
    //     n.subscribe("/carla/ego_vehicle/camera/depth/front/image_depth", 1000, handleDepthCameraImage);
    // ros::Subscriber segmentationCameraSubscriber =
    //     n.subscribe("/carla/ego_vehicle/camera/semantic_segmentation/front/image_segmentation",
    //                 1000, handleSegmentationCameraImage);

    ros::spin();

    return 0;
}
