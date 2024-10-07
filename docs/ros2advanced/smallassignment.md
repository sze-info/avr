---
layout: default
title: Small Assignment
permalink: /small_assignment/
icon: material/math-compass
---

# Small Assignment

The goal of the small assignment is for students to gain **practical** experience with ROS 2 and GitHub alongside the beginner-level theoretical knowledge acquired in class. The small assignment can be completed in a **short amount of time**: an instructor can finish it in a few hours, and an average student can complete it in a few afternoons. It is important to note that the assignment is a **requirement for a signature**.

Expected qualifications:

- One package, 1 or 2 nodes
- At least 1 publisher or 1 subscriber (more are allowed)
- Short documentation that includes the build process and node-topic relationships, detailed as in the [examples](#examples)
- Correct [naming](#repo-name)
- Use of a [template](https://sze-info.github.io/ajr/onallo/ros2git/#a-template-hasznalata) or a custom solution, but with the level of detail as in the [examples](#examples)
- Preferably compiles without errors, but `build warnings` are often acceptable; the key is learning
- As many commits as possible to show the workflow
- Short length: 30-100 lines of code per node + CMakeLists.txt, package.xml, README.md, launch files (longer is acceptable but not required)
- Preferably illustrated with images (see [examples](#examples))
- Preferably a [mermaid](https://mermaid.js.org/intro/) diagram showing the relationships between nodes and topics (see [examples](#examples), [description](https://sze-info.github.io/arj/onallo/mermaid.html))

!!! danger
    The small assignment will be acceptable if the node can be built and produces the output as specified in the task description! If this is not met, the student will have one week to correct it from the time the [issue](https://docs.github.com/en/issues) is posted!

## Examples

Example small assignments created by instructors:

- [github.com/szepilot/sze_sw1_szinusz](https://github.com/szepilot/sze_sw1_szinusz): The package consists of two nodes. The `/gen_node` generates sine waves and random numbers, which are advertised in two `std_msgs/float32` topics. The `/sum_node` sums the generated topics and advertises them in another `std_msgs/float32` topic. Implemented under `ROS 2 Humble`.
- [github.com/horverno/hor_d20_batman_turtle](https://github.com/horverno/hor_d20_batman_turtle): The package consists of one node, which can draw a "Batman logo" in the turtlesim simulator by plotting the trajectory. The advertised topic is of type `geometry_msgs/twist`. Implemented under `ROS 2 Humble`.
- [github.com/gfigneczi1/ign_b7e_array_sorter](https://github.com/gfigneczi1/ign_b7e_array_sorter): The package consists of one node. The `/array_sorter` node subscribes to a `std_msgs/msg/float32_multi_array` type topic, then advertises its sorted version in ascending order. Implemented under `ROS 2 Humble`.
- [github.com/gfigneczi1/ign_b7e_temp_sens](https://github.com/gfigneczi1/ign_b7e_temp_sens): The package consists of two nodes. The `/sensor_node` generates simulated sensor data: temperature and humidity, advertised in two separate `sensor_msgs/Temperature` and `sensor_msgs/RelativeHumidity` type topics. The `/monitor_node` monitors these data, and if the temperature exceeds a certain threshold or the humidity exceeds another, it sends an alert in a `std_msgs/String` type topic. Implemented under `ROS 2 Humble`.
- The package consists of one node. The `/minecraft_node` advertises a `visualization_msgs/Marker` type topic. Subscribing to the topic allows displaying a Minecraft character in RViz2. Implemented under `ROS 2 Humble`.
- [github.com/umiklos/ung_isl_ajr_point_and_orientation](https://github.com/umiklos/ung_isl_ajr_point_and_orientation): The package consists of two nodes. One node generates a `geometry_msgs/Point` type, and the other node advertises a `geometry_msgs/Pose` type by adding orientation. Implemented under `ROS 2 Humble`.
- [github.com/umiklos/ung_isl_ajr_data_generation_and_control](https://github.com/umiklos/ung_isl_ajr_data_generation_and_control): The package consists of two nodes. The `/sensor_data_generator` generates simulated data with a fictitious sensor, such as distance and speed, advertised in two separate `sensor_msgs/Range` and `geometry_msgs/Twist` type topics. The other node, the `/control_node`, monitors these data and makes control decisions for the robot, which are printed in the terminal. Implemented under `ROS 2 Humble`.
- The package consists of two nodes. The `/imu_data_publisher` provides accelerometer and gyroscope sensor data, advertised in a `sensor_msgs/Imu` type topic. The other node, the `/imu_data_analyzer`, analyzes these IMU data and creates reports on the robot's status in a `diagnostic_msgs/DiagnosticArray` type topic. Implemented under `ROS 2 Humble`.

It is recommended, but not mandatory, to choose from `diagnostic_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `shape_msgs`, `std_msgs`, `trajectory_msgs`, `visualization_msgs`.

## Recommended method for creating the small assignment repo: `template`

We have created a so-called template repo in both C++ and Python, which facilitates the creation of the first repository containing a package:

- [github.com/sze-info/ros2_cpp_template](https://github.com/sze-info/ros2_cpp_template)
- [github.com/sze-info/ros2_py_template](https://github.com/sze-info/ros2_py_template)

!!! tip

    Read more about it [here](https://sze-info.github.io/ajr/onallo/ros2git/#a-template-hasznalata).

<img src="https://raw.githubusercontent.com/sze-info/ros2_cpp_template/main/img/use_this_template01.png" width="60%" />

## Repo name
- The repository name should follow this pattern: `VVV_NNN_optional`, where
  - `VVV` is the first 3 characters of the last name, in lowercase
  - `NNN` is the first 3 characters of the Neptun code, in lowercase
  - `optional` is an optional addition, in lowercase
  - The above should be separated by an underscore `_` and all in lowercase
- Example: István Szabó, with Neptun code F99AXW, creating a small assignment about random numbers could have a URL like: `github.com/szaboistvan/sza_f99_random`.
