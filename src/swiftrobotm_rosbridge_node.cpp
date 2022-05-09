#include "swiftrobotm_rosbridge/swiftrobotm_rosbridge_node.h"

Rosbridge::Rosbridge(ros::NodeHandle &nh): nh_(nh), it_(nh), swiftrobot_client(SWIFTROBOT_PORT) {
    nh_.getParam("/input", inputs);
    nh_.getParam("/output", outputs);

    // to get information abount connected devices
    swiftrobot_client.subscribe<internal_msg::UpdateMsg>(0, std::bind(&Rosbridge::deviceStatusUpdate, this, std::placeholders::_1));
    swiftrobot_client.start();

    parseInputBindings();
}

// for every ros type create a new entry
ros_type_t Rosbridge::resolveRosType(std::string ros_type_str) {
    if (ros_type_str == "sensor_msgs/Image") return IMAGE;
    if (ros_type_str == "sensor_msgs/Imu") return IMU;
    return INVALID;
}

void Rosbridge::parseInputBindings() {
    ROS_ASSERT(inputs.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < inputs.size(); i++) {
        XmlRpc::XmlRpcValue binding = inputs[i];
        ROS_ASSERT(binding.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        std::string ros_topic = binding["ros_topic"];
        std::string ros_type = binding["ros_type"];
        int swift_channel = binding["swift_channel"];
        // create a publisher
        ros::Publisher tmp_pub;
        switch (resolveRosType(ros_type)) {
        case IMAGE: {
            // image is extra case
            image_transport::Publisher it_pub = it_.advertise(ros_topic, 1);
            image_publisher[swift_channel] = it_pub;
            createSwiftSubscriberImage(swift_channel);
            continue;
            break; }
        case IMU: {
            tmp_pub = nh_.advertise<sensor_msgs::Imu>(ros_topic, 1);
            createSwiftSubscriberIMU(swift_channel);
            break; }
        default:{
        // INVALID
            continue;
            break; }
        }
        ros_publisher[swift_channel] = tmp_pub;
    }
}

// callbacks
void Rosbridge::deviceStatusUpdate(internal_msg::UpdateMsg msg) {
    std::string status;
    switch (msg.status) {
    case internal_msg::ATTACHED: status = "attached"; break;
    case internal_msg::DETACHED: status = "detached"; break;
    case internal_msg::CONNECTED: status = "connected"; break;
    default: break;
    }
    ROS_INFO("Device %d is now %s", msg.deviceID, status.c_str());
}   

//provide subscribe for every possible message type
void Rosbridge::createSwiftSubscriberImage(int channel) {
    swiftrobot_client.subscribe<sensor_msg::Image>(channel, [=](sensor_msg::Image input) {
        image_transport::Publisher pub = image_publisher[channel];
        std::string encoding(input.pixelFormat, 4);
        if (encoding == "mono") {
            cv::Mat mono = cv::Mat(input.height , input.width, CV_8UC1, input.pixelArray.data.data());
            sensor_msgs::ImagePtr output = cv_bridge::CvImage(std_msgs::Header(), "mono8", mono).toImageMsg();
            pub.publish(output);
        } else {
            // color image
            // create cvMat image
            cv::Mat bgr;
            // image conversion
            if (encoding == "420v" || encoding == "420f") {
                // ros does not support YCbCr420, so we need to convert it
                int cbcr_offset = input.width * input.height ; //+ 4 * input.width;
                cv::Mat y = cv::Mat(input.height , input.width, CV_8UC1, input.pixelArray.data.data());
                cv::Mat cbcr = cv::Mat(input.height/2 , input.width/2, CV_8UC2, input.pixelArray.data.data() + cbcr_offset);
                cv::cvtColorTwoPlane(y, cbcr, bgr, cv::COLOR_YUV2BGR_NV12);
            } else if (encoding == "BGRA") {
                cv::Mat bgra = cv::Mat(input.height , input.width, CV_8UC4, input.pixelArray.data.data());
                cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
            } else {
                return;
            }
            sensor_msgs::ImagePtr output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr).toImageMsg();
            pub.publish(output);
        }
    });
}

void Rosbridge::createSwiftSubscriberIMU(int channel) {
    swiftrobot_client.subscribe<sensor_msg::IMU>(channel, [=](sensor_msg::IMU input) {
        ros::Publisher pub = ros_publisher[channel];
        sensor_msgs::Imu output;
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "imu";
        output.orientation.x = input.orientationX;
        output.orientation.y = input.orientationY;
        output.orientation.z = input.orientationZ;
        output.angular_velocity.x = input.angularVelocityX;
        output.angular_velocity.y = input.angularVelocityY;
        output.angular_velocity.z = input.angularVelocityZ;
        output.linear_acceleration.x = input.linearAccelerationX;
        output.linear_acceleration.y = input.linearAccelerationY;
        output.linear_acceleration.z = input.linearAccelerationZ;
        pub.publish(output);
    });
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "swiftrobotm_rosbridge_node");
    ros::NodeHandle nh;

    Rosbridge node(nh);

    ros::spin();

    return 0;
}