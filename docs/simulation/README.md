---
title: Theory - Simulation
permalink: /simulation/
icon: material/math-integral-box # theoretical material
---

# Simulation

During simulation, we study the expected behavior of the system using a computer model.

![](https://raw.githubusercontent.com/sze-info/arj/main/docs/_images/overview12.svg)

The essence of simulation is that we **do not** start testing our initial, possibly untested program code in the real world on our self-driving car/robot. This can have obvious disadvantages. However, it is important to note that the simulator always simulates a simplified model of reality, so code that works well in the simulator may not always work perfectly in real life.

So far, we have only used the 2D simulator called [Turtlesim](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html). It is popular due to its simplicity and educational nature, but the 3D world is naturally much more complex. Therefore, it may be advisable to use 3D simulators.

| 2D | 3D |
|:---:|:---:|
| <img src="https://docs.ros.org/en/foxy/_images/new_pen.png" width="50%"> | <img src="https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/Image_0.png" width="50%">  |
| Turtlesim  | Gazebo, Carla, SVL, AWSIM, MVsim |

The most supported simulator by ROS is Gazebo, but it is worth mentioning [SVL](https://github.com/lgsvl/simulator), which we have our own version optimized for [Nissan](https://github.com/szenergy/nissanleaf-lgsvl), [Carla](https://github.com/carla-simulator), or [CoppeliaSim](https://www.coppeliarobotics.com/).

<iframe width="560" height="315" src="https://www.youtube.com/embed/QD9iCauN0K8?rel=0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# Overview Video on Simulators

<div style="padding:56.25% 0 0 0;position:relative;"><iframe src="https://player.vimeo.com/video/879001753?h=80b62256e1" style="position:absolute;top:0;left:0;width:100%;height:100%;" frameborder="0" allow="autoplay; fullscreen; picture-in-picture" allowfullscreen></iframe></div><script src="https://player.vimeo.com/api/player.js"></script>
<p><a href="https://vimeo.com/879001753">Simulate robots like never before with Open 3D Engine</a> from <a href="https://vimeo.com/osrfoundation">Open Robotics</a> on <a href="https://vimeo.com">Vimeo</a>.</p>

# Additional Simulators

- [OSRF Gazebo](http://gazebosim.org/) - The most supported simulator by ROS, OGRE-based, ROS/ROS 2 compatible.
    - [GitHub repository](https://github.com/osrf/gazebo)
- [CARLA](https://carla.org/) - Unreal Engine-based automotive simulator. Compatible with Autoware, Baidu Apollo, ROS/ROS 2.
    - [GitHub repository](https://github.com/carla-simulator/carla)
    - [YouTube channel](https://www.youtube.com/channel/UC1llP9ekCwt8nEJzMJBQekg)
- [LGSVL / SVL](https://www.lgsvlsimulator.com/) - Unity Engine-based automotive simulator, compatible with Autoware, Baidu Apollo, ROS/ROS 2. *Note:* LG has [suspended](https://www.svlsimulator.com/news/2022-01-20-svl-simulator-sunset) active development of the SVL Simulator.
    - [GitHub repository](https://github.com/lgsvl/simulator)
    - [YouTube channel](https://www.youtube.com/c/LGSVLSimulator)
- [OSSDC SIM](https://github.com/OSSDC/OSSDC-SIM) - Unity Engine-based simulator for automotive applications, based on the suspended LGSVL simulator but actively developed. Compatible with Autoware, Baidu Apollo, and ROS/ROS 2.
     - [GitHub repository](https://github.com/OSSDC/OSSDC-SIM)
     - [YouTube video](https://www.youtube.com/watch?v=fU_C38WEwGw)
- [AirSim](https://microsoft.github.io/AirSim) - Unreal Engine-based simulator for drones and cars. ROS compatible.
     - [GitHub repository](https://github.com/microsoft/AirSim)
     - [YouTube video](https://www.youtube.com/watch?v=gnz1X3UNM5Y)
- [AWSIM](https://tier4.github.io/AWSIM) - Unity Engine-based simulator for automotive applications. Compatible with Autoware and ROS 2.
     - [GitHub repository](https://github.com/tier4/AWSIM)
     - [YouTube video](https://www.youtube.com/watch?v=FH7aBWDmSNA)
- [CoppeliaSim](https://www.coppeliarobotics.com/coppeliaSim) - Multi-platform, general-purpose robot simulator (formerly known as V-REP).
     - [YouTube channel](https://www.youtube.com/user/VirtualRobotPlatform)
- [MVSim](https://mvsimulator.readthedocs.io/)
     - [GitHub repository](https://github.com/MRPT/mvsim)
     - [YouTube video](https://www.youtube.com/watch?v=OzOG9V1h11g&list=PLOJ3GF0x2_eWvaxrKFb4BPzd4W9ss8jyc&index=6&ab_channel=JoseLuisBlanco)
- [AutoDRIVE](https://autodrive-ecosystem.github.io/)
    - [GitHub repository](https://github.com/Tinker-Twins/AutoDRIVE)
    - [YouTube channel](https://www.youtube.com/@AutoDRIVE-Ecosystem)

![](https://mrpt.github.io/mvsim-models/anims/warehouse-demo-mvsim.gif)

## Gazebo and ROS Compatibility

The selection of ROS and Gazebo versions is described based on the [Picking the "Correct" Versions of ROS & Gazebo](https://gazebosim.org/docs/garden/ros_installation) link.
*Note*: Versions of Gazebo 11 and earlier are referred to as Gazebo Classic, while those after are referred to as Ignition Gazebo or simply Ignition.

| | **GZ Citadel (LTS)**  | **GZ Fortress (LTS)**   | **GZ Garden**   |
|---|---|---|---|
| **ROS 2 Rolling**       | 🔴 | 🟢 | 🟡 |
| **ROS 2 Humble (LTS)**  | 🔴 | 🟢 | 🟡 |
| **ROS 2 Foxy (LTS)**    | 🟢 | 🔴 | 🔴 |
| **ROS 1 Noetic (LTS)**  | 🟢 | 🟡 | 🔴 |

* 🟢 - Recommended combination
* 🟡 - Possible, *but not recommended*. With extra work, the two software can be made to work together.
* 🔴 - Not compatible / not possible.
