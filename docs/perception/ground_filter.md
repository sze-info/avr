---
title: Ground filter
icon: material/code-block-tags # supplementary material
---

# Ground filter

<img src="https://raw.githubusercontent.com/url-kaist/patchwork-plusplus/master/pictures/demo_000000.png" width="80%" />

Available:
- [url-kaist/patchwork-plusplus-ros/tree/ROS2](https://github.com/url-kaist/patchwork-plusplus-ros/tree/ROS2)
- [github.com/MohamedHussein736/patchwork-plusplus-ros/tree/ROS2](https://github.com/MohamedHussein736/patchwork-plusplus-ros/tree/ROS2)
- [github.com/jkk-research/patchwork-plusplus-ros](https://github.com/jkk-research/patchwork-plusplus-ros) a fork of the original repo that only contains the ROS 2 branch, with the SZE-JKK pull request merged into the original [Urban Robotics Lab](https://github.com/url-kaist/patchwork-plusplus-ros/tree/ROS2) repo.

```bash
cd ~/ros2_ws/src
```

```bash
git clone https://github.com/jkk-research/patchwork-plusplus-ros
```

```bash
cd ~/ros2_ws/src
```

```bash
colcon build --packages-select patchworkpp
```

```bash
ros2 launch patchworkpp demo.launch
```

TODO

```bash
ros2 bag play kitti_00_sample.db3
```