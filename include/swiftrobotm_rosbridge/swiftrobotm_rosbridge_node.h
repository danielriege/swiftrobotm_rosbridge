#ifndef __SWIFTROBOTM_ROSBRIDGE_NODE_
#define __SWIFTROBOTM_ROSBRIDGE_NODE_

#include "ros/ros.h"
#include "image_transport/image_transport.h"

#include "swiftrobotc/swiftrobotc.h"
#include <XmlRpcValue.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#define SWIFTROBOT_PORT 2345

// add ros type for every swiftrobotm message
typedef enum ros_type {
    INVALID,
    IMAGE,
    IMU,
    ODOMETRY
} ros_type_t;

class Rosbridge {
public:
    Rosbridge(ros::NodeHandle &nh, uint16_t);
    Rosbridge(ros::NodeHandle &nh, std::string, uint16_t);
private:
    void initializeBridge();
    void parseInputBindings();
    void parseOutputBindings();
    void createRosPublisher();

    ros_type_t resolveRosType(std::string);
    // callbacks
    void deviceStatusUpdate(internal_msg::UpdateMsg msg);
    // subscriber swift
    void createSwiftSubscriberImage(int channel);
    void createSwiftSubscriberIMU(int channel);
    // subscriber ros
    void createRosSubscriberImage(std::string, int);
    void createRosSubscriberOdometry(std::string, int);
private:
    SwiftRobotClient swiftrobot_client;

    ros::NodeHandle &nh_;
    image_transport::ImageTransport it_;
    XmlRpc::XmlRpcValue inputs;
    XmlRpc::XmlRpcValue outputs;
    std::string test;

    std::map<int, ros::Publisher> ros_publisher;
    std::map<int, image_transport::Publisher> image_publisher;

    std::map<std::string, int> swift_publisher;
};

#endif