---
layout: default
title: ROS 2 Marker and Launch
parent: Independent Tasks
---

# Description

As an independent task, let's create a package named `my_launch_pkg`, in which a `run_transforms_and_markers.launch.py` will start the following:

- A node that publishes the `map`, `orbit1`, and `orbit2` frames (`ros2 run arj_transforms_cpp pub_transforms`)
- The `rqt_reconfigure` (`ros2 run rqt_reconfigure rqt_reconfigure`)
- The static `orbit3` frame (`ros2 run tf2_ros static_transform_publisher --x 1.0 --y 0.2 --z 1.4 --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 --frame-id orbit2 --child-frame-id orbit3`)
- And the launch that starts Rviz2 (`ros2 launch arj_transforms_cpp rviz1.launch.py`)

Verify the correct operation in rviz2.

So, at the end of the independent task, it should be possible to start it with the following command:

``` r
ros2 launch my_launch_pkg run_transforms_and_markers.launch.py
```

## Create the `my_launch_pkg` package

Prerequisite: the `arj_transforms_cpp` package must already be built and runnable.

If the `my_launch_pkg` package already exists, delete it. (In the lab, it is possible that someone created it in the previous semester.)

``` bash
cd ~ && test -d "ros2_ws/src/my_launch_pkg" && echo Exists || echo Does not exist
```

``` bash
rm -r ~/ros2_ws/src/my_launch_pkg
```

Open a new terminal and source the installation (if not in `bashrc`), so that the `ros2` commands work.

Navigate to the already created `ros2_ws` directory.

It is important to create the packages in the `src` directory, not in the root of the workspace. So navigate to the `ros2_ws/src` folder and run the package creation command:

``` bash
cd ~/ros2_ws/src
```

``` bash
ros2 pkg create --build-type ament_cmake my_launch_pkg
```

The terminal will return a message confirming the creation of the `my_launch_pkg` package and all necessary files and folders.

## Launch folder

Create a folder for the launch files:

``` bash
cd ~/ros2_ws/src/my_launch_pkg
```

``` bash
mkdir launch
```

## Create the launch file

``` bash
cd launch
```

``` bash
code run_transforms_and_markers.launch.py
```

Compose a launch file:

``` py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # ros2 run arj_transforms_cpp pub_transforms
        Node(
            package='arj_transforms_cpp',
            executable='pub_transforms',
        ),
        # ros2 run rqt_reconfigure rqt_reconfigure
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
        ),
        # ros2 run tf2_ros static_transform_publisher --x 1.0 --y 0.2 --z 1.4 --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 --frame-id orbit2 --child-frame-id orbit3
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1.0', '0.2', '1.4','0', '0', '0', '1', 'orbit2','orbit3'],
        ),     
        # ros2 launch arj_transforms_cpp rviz1.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("arj_transforms_cpp"), '/launch/', 'rviz1.launch.py'])
        ),
    ])
```

## Using `ROS2` launch

The created launch file can be started as follows:

``` bash
cd ~/ros2_ws/src/my_launch_pkg/launch # enter the folder containing the launch file
```
``` bash
ros2 launch run_transforms_and_markers.launch.py
```

In addition to the direct method shown above, a launch file can also be run by the package:

``` bash
ros2 launch <package_name> <launch_file_name>
```

For packages that contain launch files, it is advisable to create an ```exec_depend``` dependency on the ```ros2launch``` package in the package's ```package.xml``` file:

``` xml
<exec_depend>ros2launch</exec_depend>
```

This ensures that the ```ros2 launch``` command is available after the package is built.

## Add to the package to be runnable from anywhere

``` bash
cd ~/ros2_ws/src/my_launch_pkg
```

``` bash
code .
```

Insert the following line before `<test_depend>` in the package.xml:

``` xml
<exec_depend>ros2launch</exec_depend>
```

Insert the following 2 lines before `ament_package()` in the CMakeLists.txt:

``` cmake
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
```

Build in the usual way:

``` bash
cd ~/ros2_ws
```

``` bash
colcon build --packages-select my_launch_pkg
```

``` bash
source ~/ros2_ws/install/setup.bash
```

This command can now be issued from __anywhere__:

``` bash
ros2 launch my_launch_pkg run_transforms_and_markers.launch.py
```

# Sources
- [docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
