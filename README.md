# swiftrobotm Rosbridge

This package is a ros bridge for the [swiftrobotm](https://github.com/danielriege/swiftrobotm) middleware. It uses the [swiftrobotc](https://github.com/danielriege/swiftrobotc) client library to communicate with iOS devices via WiFi or USB. This package is still work in progress

## supported message types

#### swiftrobotm to ROS

| swiftrobotm       | ROS                | Description                                                  |
| ----------------- | ------------------ | ------------------------------------------------------------ |
| sensor_msg.Image | sensor_msgs::Image | possible pixel formats in the swiftrobotm message are `420v`, `420f` and  `bgra` for colored images. These will be converted to a bgr8 encoding for the ros message. When using `mono`, it will be converted to `mono8`. |
|sensor_msg.IMU | sensor_msgs::Imu | only the direct IMU values are supported in swiftrobotm. The covariance matrices are not. |

#### ROS to swiftrobotm

| swiftrobotm | ROS  | Description |
| ----------- | ---- | ----------- |
| sensor_msg.Image | sensor_msgs::Image | possible pixel formats in the swiftrobotm message are `420v`, `420f` and  `bgra` for colored images. These will be converted to a bgr8 encoding for the ros message. When using `mono`, it will be converted to `mono8`. |
| nav_msg.Odometry| nav_msgs::Odometry | Only the pose is supported. The twist will always be zero. |

