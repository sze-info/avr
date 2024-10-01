---
# icon: material/home
hide:
  - toc
  - navigation
---

- Course name in Hungarian: **Autonóm járművek és robotok programozása**
- Course name in English: **Autonomous robots and vehicles software engineering**
- Neptun course code: `GKNB_AUTM078`

The course is offered in several engineering programs and builds on minimal existing programming knowledge. It starts with the following prerequisites: **Vehicle Engineering** (BSc) `GKNB_INTM023` Basics of Programming, **Software Engineering** (BSc) `GKNB_MSTM032` Python Programming, **Computer Engineering** (BSc) `GKNB_INTM114` Programming, **Mechatronics Engineering** (BSc) `GKNB_MSTM032` Python Programming, **Electrical Engineering** (BSc) `GKNB_INTM023` Basics of Programming.

!!! abstract "Course Objective"
    After successfully completing the course, students will have a comprehensive understanding of the software modules and other essential components of autonomous vehicles.

    They will be able to develop simpler related software modules in C++ and Python. Additionally, students will be capable of applying the appropriate code, interpreting its functions, and further developing it within the ROS 2 framework.

The course content is based on the curricula of internationally recognized universities such as MIT, ETH Zurich, TU Munich, University of Virginia, and Stanford University. Certain parts of the curriculum were adopted or used as inspiration, adhering to the appropriate licenses, [detailed here](https://github.com/sze-info/ajr#acknowledgement). In 2023, the course uses the ROS 2 Humble version, which is supported until May 2027.

Completing the course can be advantageous for employment at the following automotive/autonomous vehicle companies (in alphabetical order):

- [AiMotive Kft.](https://aimotive.com/) **(Budapest)**
- [AVL Hungary Kft.](https://www.avl.com/en-hu/locations/avl-hungary-kft) **(Budapest, Érd, Kecskemét)**
- [Bosch: Robert Bosch Kft.](https://www.bosch.hu/karrier/) **(Budapest, Győr, Zalaegerszeg)**
- [Continental Autonomous Mobility Hungary Kft.](https://www.continental.com/hu-hu/vallalat/continental-vallalatok-magyarorszagon/am-hungary/) **(Budapest, Győr, Debrecen)**
- [Vehicle Industry Research Center, SZE](https://jkk-web.sze.hu/szakmai-kompetenciak/autonom-kozlekedesi-rendszerek-kozpont/) **(Győr)**

Further career-related insights can be found on [statista](https://www.statista.com/outlook/tmo/robotics/worldwide), [glassdoor](https://www.glassdoor.com/Sueldos/robotics-developer-sueldo-SRCH_KO0,18.htm), and [linkedin](https://www.linkedin.com/posts/the-construct_robotics-developer-ros-activity-7208725277965787136-A_UQ/).

!!! note 
    Based on the knowledge presented in the course, students can prepare theses, dissertations, project work, and TDK papers, and there is also an opportunity to complete the mandatory internship.

<center> 

Instructors | | | .
-----|-----|-----|-----
<img src="https://raw.githubusercontent.com/sze-info/arj/main/docs/_images/okt_he01.png" width="80px"/> | Dr. Ernő Horváth <br/> <i>Course Leader</i> <br/>[github.com/horverno](http://github.com/horverno) | <img src="https://raw.githubusercontent.com/sze-info/arj/main/docs/_images/okt_ba01.png" width="80px"/>| Dr. Áron Ballagi <br/> <i>Curriculum, not teaching</i><br/> [github.com/aronball](http://github.com/aronball)
<img src="https://raw.githubusercontent.com/sze-info/arj/main/docs/_images/okt_kr01.png" width="80px"/> | Rudolf Krecht <br/> <i>Simulation, Robotics</i> <br/> [github.com/rudolfkrecht](http://github.com/rudolfkrecht) | <img src="https://raw.githubusercontent.com/sze-info/arj/main/docs/_images/okt_um01.png" width="80px"/> | Miklós Unger <br/> <i>Environmental Sensing</i> <br/> [github.com/umiklos](http://github.com/umiklos)
<img src="https://raw.githubusercontent.com/sze-info/arj/main/docs/_images/okt_ig01.png" width="80px"/>| Gergő Ignéczi <br/> <i>Control Engineering</i> <br/> [github.com/gfigneczi1](http://github.com/gfigneczi1)|<img src="https://raw.githubusercontent.com/sze-info/arj/main/docs/_images/okt_mn01.png" width="80px"/> | Norbert Markó <br/> <i>AI, Neural Networks</i>  <br/> [github.com/norbertmarko](http://github.com/norbertmarko)

</center>

In the fall semester of 2023/24, classes will be held in room `A2` and the computer lab `B6`.

## Theory

Class | Date | Material
-----|-----|-----
1 | Sep. 6 | [Introduction](https://sze-info.github.io/ajr/bevezetes/): Course structure. Knowledge of robotics and autonomous vehicles. Sensing, perception, planning, control, actuation.
2 | Sep. 13 | [ROS2 Concepts](https://sze-info.github.io/ajr/bevezetes/ros2/): Introduction to university robots and vehicles. Basics of `ROS 2`.
3 | Sep. 20 | [Sensing](https://sze-info.github.io/ajr/erzekeles/): Operation and signal processing of sensors such as cameras, LIDAR, GNSS (GPS), IMU, CAN. Main `ROS 2` topics, `ROS 2` time management.
4 | Sep. 27 | [Midterm Assignment](https://sze-info.github.io/ajr/feleves_beadando/): Introduction to the midterm assignment, grading criteria, ideas, Q&A.
5 | Oct. 4 | [Transformations](https://sze-info.github.io/ajr/transzformaciok/): Rigid body motion, matrix multiplication review, demonstration of homogeneous coordinates with short code snippets, concept of quaternions.
6 | Oct. 11 | [Perception](https://sze-info.github.io/ajr/eszleles/): Object recognition, object classification, object tracking and prediction, SLAM and LOAM.
7 | Oct. 25 | [Simulation](https://sze-info.github.io/ajr/szimulacio/): Overview of ROS 2 compatible simulators (e.g., [Gazebo](http://gazebosim.org/), [Carla](https://carla.org/), [SVL](https://www.lgsvlsimulator.com/), [OSSDC SIM](https://github.com/OSSDC/OSSDC-SIM), [AirSim](https://microsoft.github.io/AirSim), [AWSIM](https://tier4.github.io/AWSIM), [CoppeliaSim](https://www.coppeliarobotics.com/coppeliaSim), [MVSim](https://mvsimulator.readthedocs.io/)).
8 | Nov. 8 | [Planning](https://sze-info.github.io/ajr/tervezes/): Global planning, local planning. Local planning: lateral and longitudinal planning.
9 | Nov. 15 | [Control](https://sze-info.github.io/ajr/szabalyozas/): Vehicle control solutions (inverse models, predictive models, closed-loop models).
10 | Nov. 22 | [Artificial Intelligence](https://sze-info.github.io/ajr/mesterseges_intelligencia/): Neural networks with a focus on vehicles and robotics.

## Practice

Class | Date | Material
-----|-----|-----
1| Sep. 6 | [Introduction](https://sze-info.github.io/ajr/bevezetes/practice/) + [Linux](https://sze-info.github.io/ajr/bevezetes/linux/) + [Computer Lab Knowledge](https://sze-info.github.io/ajr/bevezetes/gepterem/): Using WSL2 on Windows OS. Basic computer lab knowledge. Linux commands that may be needed later.
2| Sep. 13 | [Installation](https://sze-info.github.io/ajr/telepites/ros_humble/) + [Setting Up Development Environment](https://sze-info.github.io/ajr/bevezetes/vscodegit/) + [ROS2 Communication](https://sze-info.github.io/ajr/bevezetes/ros2gyak/): First `ROS 2` nodes, using ROS commands, build and source.
3| Sep. 20 | [Sensing Practice](https://sze-info.github.io/ajr/erzekeles/practice/): Common formats of sensor data: `sensor_msgs/PointCloud2`, `sensor_msgs/Image`, `geometry_msgs/Pose`, etc. Handling and playing back bag `.mcap` files. Creating a simple package that subscribes to position data.
4| Sep. 27 | [Version Control, Git](https://sze-info.github.io/ajr/onallo/ros2git/), [Copilot](https://sze-info.github.io/ajr/bevezetes/copilot/), [VS Code](https://sze-info.github.io/ajr/bevezetes/vscodegit/), [ROS 2 Launch](https://sze-info.github.io/ajr/ros2halado/ros2launch/): Using Copilot for ROS 2 development, introduction to the template repo, using it, writing launch files in Python.
5| Oct. 4 | [Transformations Practice](https://sze-info.github.io/ajr/transzformaciok/practice/): Creating a node that advertises transformations. Displaying markers, independent task with launch.
6| Oct. 11 | [Perception Practice](https://sze-info.github.io/ajr/eszleles/practice/): Simple LIDAR filtering based on X, Y, and Z coordinates.
7| Oct. 25 | [Simulation Introduction](https://sze-info.github.io/ajr/szimulacio/gazebo_fortress/): Gazebo Fortress and ROS 2, [Simulation Practice](https://sze-info.github.io/ajr/szimulacio/gyakorlat/): Creating your own robot simulation.
8| Nov. 8 | [Planning Practice](https://sze-info.github.io/ajr/tervezes/practice/): Implementing a polynomial-based local planner. Using [Nav2](https://navigation.ros.org/) with a simulator.
9| Nov. 15 | [Control Practice](https://sze-info.github.io/ajr/szabalyozas/ros2practice/): PID tuning. Trajectory following with Gazebo simulator. Custom-developed controller and vehicle model.
10| Nov. 22 | [Artificial Intelligence Practice](https://sze-info.github.io/ajr/mesterseges_intelligencia/practice/): Neural networks practice.

## Grades

- **Signature**: Completion of a small assignment (simple home programming task)
- **1 (fail)**: $0\% \leq ZH_{average} < 60\%$
- **2 (pass)**: $60\% \leq ZH_{average} < 85\%$
- **3 (satisfactory)**: $85\% \leq  ZH_{average} \leq 100\%$  
- **4 (good)**: Large midterm assignment completed with major errors according to the [specified guidelines](https://sze-info.github.io/ajr/feleves_beadando/)
- **5 (excellent)**: Large midterm assignment completed with only a few minor errors according to the [specified guidelines](https://sze-info.github.io/ajr/feleves_beadando/)

## Conventions

- :material-code-braces-box:  practical material
- :material-math-integral-box:  theoretical material
- :material-code-block-tags:  supplementary material, may be necessary for a complete understanding

## Topics Covered (not in the order of processing)

- Introduction: Introduction to autonomous vehicles: current status, past, and future. Sensors, actuators, communication technologies. (LIDAR, radar, active and passive cameras, GPS, odometry, IMU, CAN) Foxglove studio and own measurements for illustration.
- Software System: Software for autonomous vehicles: sensing, perception, planning, tracking. Simulation technologies, user interfaces. Frameworks: ROS/ROS2/MATLAB/LabVIEW roles, real-time systems (FPGA, real-time operating systems).
- Sensing: SLAM, object detection, object tracking and prediction. Curb detection, lane detection, road defect detection, vehicle and pedestrian detection/tracking, etc. Advantages and disadvantages of artificial intelligence (especially neural networks) and traditional (e.g., C++ based) algorithms, their fusion.
- Technological Knowledge: Linux, Git: Linux knowledge: Terminal handling, Git handling, VS code, ROS installation.
- Technological Knowledge: ROS 2 Basics: topics and messages, MCAP (Rosbag) playback, handling topics, accessing topic content from Python, rviz, rqt_plot, creating MCAP (Rosbag). 
ROS 2 ecosystem and development: Creating ROS 2 nodes in Python and C++: ROS 2 nodes, rqt_graph, Publisher / Subscriber node in Python, Publisher / Subscriber node in C++. 
First consultation on the individual project task.
- ROS 2 Programming: Processing ROS sensor data with a C++ node: Writing ROS nodes, `visualization_msgs`, LIDAR sensor data: `sensor_msgs/PointCloud2`, `sensor_msgs/LaserScan`, etc.
- Simulation and Control: Simulation: Using ROS nodes for simulation (gazebo) F1/10, rviz, finalizing individual project tasks. Planning block: types of trajectory planners, kinematic challenges, Control block: vehicle modeling, introduction to controllers, vehicle and actuator-level control, implementing motion (braking systems, steering systems, etc.).
- Outlook, Doctoral Research, University Student Teams: Nissan Leaf, Lexus, and Szenergy autonomous projects, introduction with selected code snippets.
Sensing: point cloud handling or object detection based on camera.
Perception / Planning: route determination, free space determination, trajectory planning.
Control: closed-loop modeled vehicle, building a controller (e.g., PID or pure pursuit).
- Artificial Intelligence: Software for autonomous vehicles, summary, outlook on neural networks (artificial intelligence, AI).
- Technological Knowledge: Using ROS 2, its innovations compared to ROS.
- Project Work: Presentation of individual project tasks.

![](https://raw.githubusercontent.com/sze-info/arj/main/docs/_images/technology01.svg)

<br><br><br>
<center>
<h1><a href="https://sze-info.github.io/avr/">sze-info.github.io/avr</a></h1>
</center>

<br><br><br>