#ifndef __SWIFTROBOTM_ROSBRIDGE_NODE_
#define __SWIFTROBOTM_ROSBRIDGE_NODE_

#include "ros/ros.h"
#include "image_transport/image_transport.h"

#include "swiftrobotc/swiftrobotc.h"
#include <XmlRpcValue.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Imu.h"

#define SWIFTROBOT_PORT 2345

// add ros type for every swiftrobotm message
typedef enum ros_type {
    INVALID,
    IMAGE,
    IMU
} ros_type_t;

class Rosbridge {
public:
    Rosbridge(ros::NodeHandle &nh);
private:
    void parseInputBindings();
    void createRosPublisher();

    ros_type_t resolveRosType(std::string);
    // callbacks
    void deviceStatusUpdate(internal_msg::UpdateMsg msg);
    // subscriber
    void createSwiftSubscriberImage(int channel);
    void createSwiftSubscriberIMU(int channel);
private:
    SwiftRobotClient swiftrobot_client;

    ros::NodeHandle &nh_;
    image_transport::ImageTransport it_;
    XmlRpc::XmlRpcValue inputs;
    XmlRpc::XmlRpcValue outputs;

    std::map<int, ros::Publisher> ros_publisher;
    std::map<int, image_transport::Publisher> image_publisher;
};

#endif