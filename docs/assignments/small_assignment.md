---
layout: default
title: Small Assignment
permalink: /small_assignment/
icon: material/math-compass
---

# Small Assignment

The purpose of the small assignment is for students to gain **practical** experience with ROS 2 and GitHub alongside the basic theoretical knowledge acquired in class. The small assignment can be completed in a relatively **short time**: an instructor can finish it in a few hours, and an average student can complete it in a few afternoons. It is important to note that the assignment is a **requirement for the signature**.

Expected qualities:

- One package, 1 or 2 nodes
- At least 1 publisher or 1 subscriber (more is allowed)
- Short documentation that includes the build process and node-topic relationships, with the level of detail as shown in the [examples](#examples)
- Correct [naming](#repo-name)
- Use of a template [recommended method for creating the small assignment repo](#recommended-method-for-creating-the-small-assignment-repo-template) or your own solution, but with the level of detail as shown in the [examples](#examples)
- Preferably compiles without errors, but `build warnings` are often acceptable, the main point is learning
- As many commits as possible to show the workflow
- Short length: 30-100 lines of code per node + CMakeLists.txt, package.xml, README.md, launch files (longer is acceptable but not required)
- Preferably illustrated with images (see [examples](#examples))
- Preferably a [mermaid](https://mermaid.js.org/intro/) diagram showing the relationships between nodes and topics (see [examples](#examples), [description](https://sze-info.github.io/arj/onallo/mermaid.html))

!!! danger
    The small assignment will be acceptable if the node can be built and produces the output as specified in the task! If this is not met, the student will have one week to correct it from the time the [issue](https://docs.github.com/en/issues) is posted!

## Examples

Example of a small assignment created by the instructors:

- [github.com/szepilot/sze_sw1_szinusz](https://github.com/szepilot/sze_sw1_szinusz): The package consists of two nodes. The `/gen_node` generates sine waves and random numbers, which are advertised in two `std_msgs/float32` topics. The `/sum_node` sums the generated topics and advertises them in a new `std_msgs/float32` topic. Implemented under `ROS 2 Humble`.
- [github.com/horverno/hor_d20_batman_turtle](https://github.com/horverno/hor_d20_batman_turtle): The package consists of one node, which can draw a "Batman logo" in the turtlesim simulator by plotting the trajectory. The advertised topic is of type `geometry_msgs/twist`. Implemented under `ROS 2 Humble`.
- The package consists of one node. This `/array_sorter` node subscribes to a `std_msgs/msg/float32_multi_array` type topic, then advertises the sorted version of the same type in ascending order. Implemented under `ROS 2 Humble`.
- The package consists of two nodes. The `/sensor_node` generates simulated sensor data: temperature and humidity, which are advertised in two separate `sensor_msgs/Temperature` and `sensor_msgs/RelativeHumidity` type topics. The `/monitor_node` monitors these data, and if the temperature exceeds a certain threshold or the humidity exceeds another, it sends an alert in a `std_msgs/String` type topic. Implemented under `ROS 2 Humble`.
- The package consists of one node. The `/minecraft_node` advertises a `visualization_msgs/Marker` type topic. By subscribing to the topic, a Minecraft character can be displayed in RViz2. Implemented under `ROS 2 Humble`.
- [github.com/umiklos/ung_isl_ajr_point_and_orientation](https://github.com/umiklos/ung_isl_ajr_point_and_orientation): The package consists of two nodes. One node generates a `geometry_msgs/Point` type, and the other node advertises a `geometry_msgs/Pose` type by adding orientation to it. Implemented under `ROS 2 Humble`.
- [github.com/umiklos/ung_isl_ajr_data_generation_and_control](https://github.com/umiklos/ung_isl_ajr_data_generation_and_control): The package consists of two nodes. The `/sensor_data_generator` generates simulated data with a fictitious sensor, such as distance and speed, which are advertised in two separate `sensor_msgs/Range` and `geometry_msgs/Twist` type topics. The other node, the `/control_node`, monitors these data and makes control decisions for the robot, which are printed in the terminal. Implemented under `ROS 2 Humble`.
- The package consists of two nodes. The `/imu_data_publisher` provides accelerometer and gyroscope sensor data, which are advertised in a `sensor_msgs/Imu` type topic. The other node, the `/imu_data_analyzer`, analyzes these IMU data and generates reports on the robot's status in a `diagnostic_msgs/DiagnosticArray` type topic. Implemented under `ROS 2 Humble`.

It is advisable, but not mandatory, to choose from `diagnostic_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `shape_msgs`, `std_msgs`, `trajectory_msgs`, `visualization_msgs`.

## Recommended Method for Creating the Small Assignment Repo: `Template`

We have created a so-called template repo in both C++ and Python, which makes it easier to create the first repository containing a package:

- [github.com/sze-info/ros2_cpp_template](https://github.com/sze-info/ros2_cpp_template)
- [github.com/sze-info/ros2_py_template](https://github.com/sze-info/ros2_py_template)

!!! tip

    You can read about it [here](https://sze-info.github.io/arj/onallo/ros2git.html).

<img src="https://raw.githubusercontent.com/sze-info/ros2_cpp_template/main/img/use_this_template01.png" width="60%" />

## Repo Name
- The repository name should follow this pattern: `VVV_NNN_optional`, where
  - `VVV` is the first 3 characters of the last name
  - `NNN` is the first 3 characters of the Neptun code
  - `optional` is an optional addition
  - The above should be separated by an underscore `_` and all lowercase
- For example: István Szabó, with Neptun code F99AXW, could have a URL for a small assignment dealing with random numbers like: `github.com/szaboistvan/sza_f99_random`.