---
title: Practice
# icon: material/code-json
icon: material/code-braces-box # practical material
---

# Introduction Practice

During the practice, we will get to know the characteristic properties of autonomous vehicles and the characteristics of the recorded data.

## Foxglove Studio

As an introduction, let's look at the characteristic data of an autonomous vehicle. As an example, it is advisable to examine the data recorded with one of the **university's** vehicles. We will use [Foxglove Studio](https://foxglove.dev/download), as it is available without installation or as a ~150MB installable file, and it can visualize the data that is important to us. The examined data will show a similar picture:

![foxglove_a](foxglove04.png#only-light)
![foxglove_a](foxglove03.png#only-dark)

In class, download the visualized rosbag `.bag` / `.mcap` file and the Foxglove Studio layout from the `K:\` drive (`\\fs-kab.eik.sze.hu\C100\kozos\GKNB_AUTM078_Autonomous_robots_and_vehicles_programming`), or at home using the green button:

[Download MCAP :material-download: 553 MB](https://laesze-my.sharepoint.com/:u:/g/personal/herno_o365_sze_hu/Eclwzn42FS9GunGay5LPq-EBA6U1dZseBFNDrr6P0MwB2w?download=1){ .md-button .md-button--primary}
[Download Layout :material-auto-download:](https://raw.githubusercontent.com/sze-info/arj/main/docs/bevezetes/lexus01foxglove.json){ .md-button }

```sh
cd /mnt/c/temp
mkdir /mnt/c/temp # if it doesn't exist
rsync -avzh --progress /mnt/kozos/measurement_files/lexus3-2024-04-05-gyor.mcap /mnt/c/temp/
rsync -avzh --progress /mnt/kozos/measurement_files/lexus01foxglove.json /mnt/c/temp/
```
!!! tip Additional example data can be downloaded from the https://jkk-research.github.io/dataset page. `

## Introduction to Foxglove

While the `.mcap` is downloading, let's briefly introduce the Foxglove Studio program. Foxglove Studio is an open-source tool for visualizing and debugging robotics data. Specifically, it was open-source up to `v1.87.0`, and from `v2.0.0` it is free to use but closed-source. It is available in several ways:

- as a standalone desktop application
- accessible in a browser
- self-hosted on your own domain

Native robotics tools (such as parts of the ROS ecosystem) are generally only supported on Linux systems, but the Studio desktop application works on Linux, Windows, and macOS. Even if the ROS stack runs on another operating system, the Studio can communicate with the robot seamlessly.

The Studio offers rich visual elements and debugging panels - from interactive charts, 3D visual elements, camera images, and diagnostic data streams. Whether it's real-time robot tracking or debugging in a `.bag` / `.mcap` file, these panels help solve various common robotics tasks.

These panels can then be configured and assembled into custom layouts to suit the unique needs and workflows of the project.

## Description of the University's Nissan Measurement Data

In the ROS system (but also in other similar robotics solutions), individual data are published in [topics](http://wiki.ros.org/Topics). A topic can be, for example, the output of a sensor, the input of a controller, a visualization marker, etc. Topics have [types](http://wiki.ros.org/Messages), there are many predefined types, but we can create our own if these are not enough. For example, a few predefined types:

-  [`sensor_msgs/Image`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) - Uncompressed image information, typically from the camera, but can be processed data, such as marked pedestrians.
-  [`sensor_msgs/CompressedImage`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CompressedImage.html) - Compressed image information.
-  [`std_msgs/String`](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html) - Simple text message type.
-  [`std_msgs/Bool`](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html) - Simple binary message type.
-  [`geometry_msgs/Point`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html) - XYZ 3D point.
- [`geometry_msgs/Pose`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html) - 3D point and associated orientation.

As you can see, the types fall into different categories, such as `std_msgs`, `diagnostic_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, etc. Let's see what types of messages are found in the downloaded file:

| Topic | Type | Hz | Sensor |
| --- | --- | --- | --- |
/gps/duro/current_pose | `geometry_msgs/PoseStamped` | 10 | Duro GPS (UTM)
/gps/duro/imu | `sensor_msgs/Imu` | 200 | Duro GPS
/gps/duro/mag | `sensor_msgs/MagneticField` | 25 | Duro GPS
/gps/nova/current_pose | `geometry_msgs/PoseStamped` | 20 | Novatel GPS (UTM)
/gps/nova/imu | `sensor_msgs/Imu` | 200 | Novatel GPS
/left_os1/os1_cloud_node/imu | `sensor_msgs/Imu` | 100 | Ouster LIDAR
/left_os1/os1_cloud_node/points | `sensor_msgs/PointCloud2` | 20 | Ouster LIDAR
/right_os1/os1_cloud_node/imu | `sensor_msgs/Imu` | 100 | Ouster LIDAR
/right_os1/os1_cloud_node/points | `sensor_msgs/PointCloud2` | 20 | Ouster LIDAR
/velodyne_left/velodyne_points | `sensor_msgs/PointCloud2` | 20 | Velodyne LIDAR
/velodyne_right/velodyne_points | `sensor_msgs/PointCloud2` | 20 | Velodyne LIDAR
/cloud | `sensor_msgs/PointCloud2` | 25 | SICK LIDAR
/scan | `sensor_msgs/LaserScan` | 25 | SICK LIDAR
/zed_node/left/camera_info | `sensor_msgs/CameraInfo` | 30 | ZED camera
/zed_node/left/image_rect_color/compressed | `sensor_msgs/Image` | 20 | ZED camera
/vehicle_status | `autoware_msgs/VehicleStatus` | 100 | CAN data
/ctrl_cmd | `autoware_msgs/ControlCommandStamped` | 20 | Reference speed and steering angle
/current_pose | `geometry_msgs/PoseStamped` | 20 | Current GPS
/tf | `tf2_msgs/TFMessage` | 500+ | Transform