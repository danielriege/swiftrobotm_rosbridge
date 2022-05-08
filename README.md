# swiftrobotm Rosbridge

This package is a ros bridge for the swiftrobotm middleware. It uses the swiftrobotc client library to communicate with iOS devices via WiFi or USB. This package is still work in progress

## supported message types

#### swiftrobotm to ROS

| swiftrobotm       | ROS                | Description                                                  |
| ----------------- | ------------------ | ------------------------------------------------------------ |
| sensor_msg::Image | sensor_msgs::Image | possible pixel formats in the swiftrobotm message are `420v`, `420f` and  `bgra` for colored images. These will be converted to a bgr8 encoding for the ros message. When using `mono`, it will be converted to `mono8`. |
|                   |                    |                                                              |
|                   |                    |                                                              |

#### ROS to swiftrobotm

| swiftrobotm | ROS  | Description |
| ----------- | ---- | ----------- |
|             |      |             |
|             |      |             |
|             |      |             |

