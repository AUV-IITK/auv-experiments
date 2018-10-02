# hardware_camera

## Overview
This is a ROS package for getting feed from the camera using [usb_cam](http://wiki.ros.org/usb_cam) and publishing it as a [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) to the other layers.

The `hardware_camera` package has been tested under [ROS](http://www.ros.org) Kinetic and Ubuntu 16.04 LTS. The source code is released under a [BSD 3-Clause license](LICENSE.md).

The camera used is a [Microsoft LifeCam VX-700](https://www.cnet.com/products/microsoft-lifecam-vx-700-web-camera-series/specs/).

## Setting up hardware_camera

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- Following ROS Packages: [usb_cam](http://wiki.ros.org/usb_cam), [image_transport](http://wiki.ros.org/image_transport)

### Preparing the serial port
The camera will most like connect to your computer in `/dev/video1` (assuming your computer has a built-in camera and there is no other external camera connected to your computer).

### Building the package
Run the following command from the root of your catkin workspace:
```
catkin_make --pkg hardware_camera
```

## Usage
To launch the camera, run:
```
roslaunch hardware_camera cam_lifecam.launch
```

## Nodes

### usb_cam_node

The `usb_cam_node` interfaces with standard USB cameras (e.g. the Logitech Quickcam) using libusb_cam and publishes images as `sensor_msgs::Image`. The node belongs to the [usb_cam](http://wiki.ros.org/usb_cam) package.

#### Published Topics

* **`hardware_camera/cam_lifecam/image_raw`** ([sensor_msgs/Image])

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/AUV-IITK/fourtran/issues).

[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
