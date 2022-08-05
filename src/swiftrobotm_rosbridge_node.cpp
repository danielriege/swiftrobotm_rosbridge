#include "swiftrobotm_rosbridge/swiftrobotm_rosbridge_node.h"

// for USB
Rosbridge::Rosbridge(ros::NodeHandle &nh, uint16_t port): nh_(nh), it_(nh), swiftrobot_client(port), test("test") {
    initializeBridge();
}

Rosbridge::Rosbridge(ros::NodeHandle &nh, std::string device_ip, uint16_t port): nh_(nh), it_(nh), swiftrobot_client(device_ip, port), test("test") {
    initializeBridge();
}

void Rosbridge::initializeBridge() {

    // to get information abount connected devices
    swiftrobot_client.subscribe<internal_msg::UpdateMsg>(0, std::bind(&Rosbridge::deviceStatusUpdate, this, std::placeholders::_1));
    swiftrobot_client.start();

    if (nh_.hasParam("/outputs")) {
        nh_.getParam("/outputs", outputs);
        parseOutputBindings();
    }
    if (nh_.hasParam("/inputs")) {
        nh_.getParam("/inputs", inputs);
        parseInputBindings();
    }
}

// for every ros type create a new entry
ros_type_t Rosbridge::resolveRosType(std::string ros_type_str) {
    if (ros_type_str == "sensor_msgs/Image") return IMAGE;
    if (ros_type_str == "sensor_msgs/Imu") return IMU;
    if (ros_type_str == "nav_msgs/Odometry") return ODOMETRY;
    return INVALID;
}

void Rosbridge::parseOutputBindings() {
    ROS_ASSERT(outputs.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < outputs.size(); i++) { 
        XmlRpc::XmlRpcValue binding = outputs[i];
        ROS_ASSERT(binding.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        std::string ros_topic = binding["ros_topic"];
        int swift_channel = binding["swift_channel"];
        ros_type_t ros_type = resolveRosType(binding["ros_type"]);
        switch (ros_type) {
        case IMAGE: {
            createRosSubscriberImage(ros_topic, swift_channel);
            break;}
        case IMU: {
            break;}
        case ODOMETRY: {
            createRosSubscriberOdometry(ros_topic, swift_channel);
            break;}
        default: {
            continue;
            break; }
        }
    }
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

// ROS subscriber

void Rosbridge::createRosSubscriberImage(std::string topic, int channel) {
    static image_transport::Subscriber sub = it_.subscribe(topic, 1, [=](const sensor_msgs::ImageConstPtr& msg) {

        std::vector<uint8_t> pixelData = msg->data;
        sensor_msg::Image image_msg;
        image_msg.width = msg->width;
        image_msg.height = msg->height;
        char format[] = "BGRA";
        memcpy(image_msg.pixelFormat, format, 4);
        base_msg::UInt8Array pixelArrayMsg;
        pixelArrayMsg.data = pixelData;
        pixelArrayMsg.size = msg->step*msg->height;
        image_msg.pixelArray = pixelArrayMsg;
        
        swiftrobot_client.publish<sensor_msg::Image>(channel, image_msg);
    });
}

void Rosbridge::createRosSubscriberOdometry(std::string topic, int channel) {
    static ros::Subscriber sub = nh_.subscribe<nav_msgs::Odometry>(topic, 1, [=](const nav_msgs::Odometry::ConstPtr& msg) {
        nav_msg::Odometry odom_msg;
        odom_msg.positionX = msg->pose.pose.position.x;
        odom_msg.positionY = msg->pose.pose.position.y;
        odom_msg.positionZ = msg->pose.pose.position.z;

        tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        odom_msg.roll = roll;
        odom_msg.pitch = pitch;
        odom_msg.yaw = yaw;

        swiftrobot_client.publish<nav_msg::Odometry>(channel, odom_msg);
    });
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "swiftrobotm_rosbridge_node");
    ros::NodeHandle nh;

    Rosbridge *node;

    if (nh.hasParam("/ip_address")) {
        std::string device_ip;
        nh.getParam("/ip_address", device_ip);
        node = new Rosbridge(nh, device_ip, SWIFTROBOT_PORT);
    } else {
        node = new Rosbridge(nh, SWIFTROBOT_PORT);
    }
    ros::spin();

    return 0;
}