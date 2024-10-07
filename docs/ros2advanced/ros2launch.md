---
title: ROS 2 launch
icon: material/code-block-tags # supplementary material
---

# `ROS 2` Launch and Other Advanced Concepts

## Introduction

The ROS 2 launch system helps users define the configuration of their system and then execute it according to that configuration. The configuration step includes:
    - which programs to run,
    - what arguments the running programs should receive,
    - ROS-specific conventions that allow for easy reuse of components.

It is also important to mention that the solution supervises the launched processes and can react to changes in their runtime state.

Launch files can be created using Python, XML, or YAML.

## Preparations
### Create the `example_launch` package

If the `example_launch` package already exists, delete it. (In the lab, it is possible that someone created it in a previous semester.)

```bash
cd ~ && test -d "ros2_ws/src/example_launch" && echo Exists || echo Does not exist
```

```bash
rm -r ~/ros2_ws/src/example_launch
```

Open a new terminal and source the installation (if not in `bashrc`) to make the `ros2` commands work.

Navigate to the already created `ros2_ws` directory.

It is important to create packages in the `src` directory, not in the root of the workspace. So navigate to the `ros2_ws/src` folder and run the package creation command:

```bash
cd ~/ros2_ws/src
```

```bash
ros2 pkg create --build-type ament_cmake example_launch
```

The terminal will return a message confirming the creation of the `example_launch` package and all necessary files and folders.

### Launch Folder

Create a folder for the launch files:

```bash
cd ~/ros2_ws/src/example_launch
```

```bash
mkdir launch
```

## Creating a Launch File

```bash
cd launch
```

```bash
code turtlesim_mimc_launch.py
```

Create a launch file using the elements of the `turtlesim` package, using Python.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        return LaunchDescription([
                Node(
                        package='turtlesim',
                        namespace='turtlesim1',
                        executable='turtlesim_node',
                        name='sim'
                ),
                Node(
                        package='turtlesim',
                        namespace='turtlesim2',
                        executable='turtlesim_node',
                        name='sim'
                ),
                Node(
                        package='turtlesim',
                        executable='mimic',
                        name='mimic',
                        remappings=[
                                ('/input/pose', '/turtlesim1/turtle1/pose'),
                                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
                        ]
                )
        ])
```

The launch file created as described above launches three nodes from the `turtlesim` package. The goal is to open two turtlesim windows and replicate the movement of one turtle with the other. The only difference in launching the two turtlesim nodes is the namespace. Using unique namespaces allows launching two identical nodes simultaneously without name conflicts. Thus, both turtles will receive instructions on the same topic and report their positions on the same topic. The unique namespaces allow distinguishing the messages of the two turtles.

The last node is also from the `turtlesim` package, but the executable file is different: `mimic`. This node is supplemented with remappings. For example, the simple `/input/pose` corresponds to `/turtlesim1/turtle1/pose` and the previously known `/output/cmd_vel` is now `/turtlesim2/turtle1/cmd_vel`. This means that `mimic` subscribes to the `/turtlesim1/sim` pose topic and republishes it so that the `/turtlesim2/sim` velocity command subscribes to it. Thus, `turtlesim2` will mimic the movement of `turtlesim1`.

### Reviewing Code Details

These expressions import Python `launch` modules.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
```

Then the launch description begins:

```python
def generate_launch_description():
    return LaunchDescription([

    ])
```

The first two instructions in the launch description start the two turtlesim windows:

```python
Node(
        package='turtlesim',
        namespace='turtlesim1',
        executable='turtlesim_node',
        name='sim'
),
Node(
        package='turtlesim',
        namespace='turtlesim2',
        executable='turtlesim_node',
        name='sim'
),
```

Finally, the node that implements the movement mimicry is also started:

```python
Node(
        package='turtlesim',
        executable='mimic',
        name='mimic',
        remappings=[
            ('/input/pose', '/turtlesim1/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        ]
)
```

## Using `ROS2` Launch

The created launch file can be started as follows:

```bash
cd ~/ros2_ws/src/example_launch/launch # enter the folder containing the launch file
```
```bash
ros2 launch turtlesim_mimc_launch.py
```

Two turtlesim windows will open, and the following `[INFO]` output will be visible, listing the started nodes:
```bash
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [11714]
[INFO] [turtlesim_node-2]: process started with pid [11715]
[INFO] [mimic-3]: process started with pid [11716]
```

To test the system, publish a message to move `turtle1` in a new terminal:

```bash
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

Besides the direct method shown above, a launch file can also be run by a package:

```bash
ros2 launch <package_name> <launch_file_name>
```

For packages containing launch files, it is advisable to create an `exec_depend` dependency on the `ros2launch` package in the package's `package.xml` file:

```xml
<exec_depend>ros2launch</exec_depend>
```

This ensures that the `ros2 launch` command is available after building the package.

## Study the Launched System

With all previously launched nodes running, run the `rqt_graph` tool in a new terminal to graphically visualize the system created by the launch file:

```bash
rqt_graph
```

## Add to the Package to Launch from Anywhere

```bash
cd ~/ros2_ws/src/example_launch
```

```bash
code .
```
<figure markdown="span">
    ![Image title](vscode06.png){ width="80%" }
    <figcaption>Lexus</figcaption>
</figure>

Insert the following line before `<test_depend>` in `package.xml`:

```xml
<exec_depend>ros2launch</exec_depend>
```

Insert the following two lines before `ament_package()` in `CMakeLists.txt`:

```cmake
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
```

Build as usual:

```bash
cd ~/ros2_ws
```

```bash
colcon build --packages-select example_launch
```

```bash
source ~/ros2_ws/install/setup.bash
```

This command can now be issued from __anywhere__:

```bash
ros2 launch example_launch turtlesim_mimc_launch.py
```

# Sources
- [foxglove.dev/blog/how-to-use-ros2-launch-files](https://foxglove.dev/blog/how-to-use-ros2-launch-files)
- [youtube.com/watch?v=PqNGvmE2Pv4&t](https://www.youtube.com/watch?v=PqNGvmE2Pv4&t)
- [docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)