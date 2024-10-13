---
layout: default
title: Practice - Creating Your Own Robot Simulation in Gazebo
icon: material/code-braces-box # practical material
---

# Ignition Gazebo Practice

## Creating Your Own Robot Simulation

In the following steps, we will create a simple environment (world) and a robot model. To do this, create a `simulation` folder and within it, a file named `building_robot.sdf`. The folder can be created anywhere, and the file name is also freely selectable, but it is recommended to follow the given method for consistency.

```bash
mkdir simulation
cd simulation
touch building_robot.sdf
```

## Creating the Environment

Open the created file in Visual Studio Code and paste the following code snippet:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
    <world name="car_world">
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin
            filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
        </plugin>
        <plugin
            filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
        </plugin>
        <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
        </plugin>

        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>

        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    </plane>
                </geometry>
                </collision>
                <visual name="visual">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    <size>100 100</size>
                    </plane>
                </geometry>
                <material>
                    <ambient>0.8 0.8 0.8 1</ambient>
                    <diffuse>0.8 0.8 0.8 1</diffuse>
                    <specular>0.8 0.8 0.8 1</specular>
                </material>
                </visual>
            </link>
        </model>
    </world>
</sdf>
```

The above code snippet defines an empty environment with only a plane (ground) and default lighting (sunlight). Save the code and start the simulation as follows from the `simulation` folder:

```bash
cd ~/simulation
```
```
ign gazebo building_robot.sdf
```

After starting, you should see an empty environment as described:

![Alt text](empty_world.png)

## Creating the Robot Model

Continue editing the `building_robot.sdf` file after the `</model>` tag:

``` xml
<model name='vehicle_blue' canonical_link='chassis'>
    <pose relative_to='world'>0 0 0 0 0 0</pose>
</model>
A robot model name in this case is `vehicle_blue`. The name can be freely chosen, but it is important to ensure that the name is unique among the models used within the same environment.

The components that make up the model (e.g., chassis, wheels, etc.) will be referred to as links.

Each model can have a single `canonical_link` element. All other links used within the model will connect to this. If no `canonical_link` element is defined, the first link will be the default `canonical` type.

The `<pose>` tag is used to specify the position and orientation of a link. The `relative_to` attribute after the tag specifies what the link's position and orientation are defined relative to. Without the attribute, the position is given relative to the environment. The format for specifying position and orientation is `<pose>X Y Z R P Y</pose>`, where X, Y, and Z are the position coordinates within the frame, and R, P, and Y specify the orientation in radians. When defining the robot, we set all parameters to zero, so the robot and the environment frame coincide.

Each model (robot) consists of `links` connected by `joints`.

## Defining the Links that Make Up the Robot

When creating each link, the following must be specified:
1. Link name and position
2. Link inertial properties (mass and inertia matrix)
3. Visual and simplified (collision) geometry

- Chassis

Creating the link:

``` xml
<link name='chassis'>
    <pose relative_to='__model__'>0.5 0 0.4 0 0 0</pose>
</link>
```

Inertial properties (units are in SI):

``` xml
<inertial>
    <mass>1.14395</mass>
    <inertia>
    <ixx>0.095329</ixx>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyy>0.381317</iyy>
    <iyz>0</iyz>
    <izz>0.476646</izz>
    </inertia>
</inertial>
```

Specifying visual and simplified (collision) geometry:

``` xml
<visual name='visual'>
    <geometry>
    <box>
        <size>2.0 1.0 0.5</size>
    </box>
    </geometry>
    <material>
    <ambient>0.0 0.0 1.0 1</ambient>
    <diffuse>0.0 0.0 1.0 1</diffuse>
    <specular>0.0 0.0 1.0 1</specular>
    </material>
</visual>
```

```
<collision name='collision'>
    <geometry>
    <box>
        <size>2.0 1.0 0.5</size>
    </box>
    </geometry>
</collision>
```

Start the simulation again:

```
cd ~/simulation
```
```
ign gazebo building_robot.sdf
```

After this, you should see the robot chassis in the simulator:

![Alt text](chassis.png)

- Right and Left Wheel

Create the links for the robot's right and left (driven) wheels. This should be done within the `<model>` tags defining the robot, as all link definitions within the same model (robot) belong here.

We will create the wheels using cylinders. The wheels need to rotate around the Y-axis, so we must specify their correct orientation.

``` xml
<link name='left_wheel'>
    <pose relative_to="chassis">-0.5 0.6 0 -1.5707 0 0</pose>
    <inertial>
    <mass>1</mass>
    <inertia>
        <ixx>0.043333</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.043333</iyy>
        <iyz>0</iyz>
        <izz>0.08</izz>
    </inertia>
    </inertial>
```

Specifying visual and simplified (collision) geometry:

``` xml
<visual name='visual'>
    <geometry>
        <cylinder>
        <radius>0.4</radius>
        <length>0.2</length>
        </cylinder>
    </geometry>
    <material>
        <ambient>1.0 0.0 0.0 1</ambient>
        <diffuse>1.0 0.0 0.0 1</diffuse>
        <specular>0.0 0.0 1.0 1</specular>
    </material>
</visual>
    <collision name='collision'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
    </collision>
</link>
```
The caster wheel is defined in a similar manner, differing only in position (and of course the link name) from the right wheel definition:

```xml
<link name='right_wheel'>
    <pose relative_to="chassis">-0.5 -0.6 0 -1.5707 0 0</pose>
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.043333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.043333</iyy>
            <iyz>0</iyz>
            <izz>0.08</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
        <material>
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
            <specular>1.0 0.0 0.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
    </collision>
</link>
```
## Adding a Caster Wheel

We can also create custom frames, which we will do when building the caster wheel:

```xml
<frame name="caster_frame" attached_to='chassis'>
    <pose>0.8 0 -0.2 0 0 0</pose>
</frame>
```

The created frame is named `caster_frame`, which is attached to the `chassis` link. The `<pose>` tag specifies its position and orientation relative to this link, but the `relative_to` attribute is not needed for custom frames.

Continue defining the caster wheel:

```xml
<link name='caster'>
    <pose relative_to='caster_frame'/>
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.016</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.016</iyy>
            <iyz>0</iyz>
            <izz>0.016</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <sphere>
                <radius>0.2</radius>
            </sphere>
        </geometry>
        <material>
            <ambient>0.0 1 0.0 1</ambient>
            <diffuse>0.0 1 0.0 1</diffuse>
            <specular>0.0 1 0.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <sphere>
                <radius>0.2</radius>
            </sphere>
        </geometry>
    </collision>
</link>
```

## Defining Joints

We need to define the relationships between the previously defined links. These relationships will specify how the links can move relative to each other.

- Left Wheel Joint

Specify the joint name and type. The wheel needs to rotate, so we choose the `revolute` type.

```xml
<joint name='left_wheel_joint' type='revolute'>
    <pose relative_to='left_wheel'/>
    <parent>chassis</parent>
    <child>left_wheel</child>
    <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
        <limit>
            <lower>-1.79769e+308</lower>    <!--negative infinity-->
            <upper>1.79769e+308</upper>     <!--positive infinity-->
        </limit>
    </axis>
</joint>
```

- Right Wheel Joint

The right wheel joint is defined similarly to the left wheel joint:

```xml
<joint name='right_wheel_joint' type='revolute'>
    <pose relative_to='right_wheel'/>
    <parent>chassis</parent>
    <child>right_wheel</child>
    <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
        <limit>
            <lower>-1.79769e+308</lower>    <!--negative infinity-->
            <upper>1.79769e+308</upper>     <!--positive infinity-->
        </limit>
    </axis>
</joint>
```

- Caster Wheel Joint

Since the caster wheel is a sphere, it can rotate around all axes. Therefore, we use a different joint type:

```xml
<joint name='caster_wheel' type='ball'>
    <parent>chassis</parent>
    <child>caster</child>
</joint>
```

Start the simulation again:

```bash
cd ~/simulation
ign gazebo building_robot.sdf
```

After this, you should see the robot in the simulator:

![Alt text](robot.png)

The content of the XML descriptor file presented so far:
``` xml
<?xml version="1.0" ?>
<sdf version="1.8">
    <world name="car_world">
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin
            filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
        </plugin>
        <plugin
            filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
        </plugin>
        <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
        </plugin>

        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>

        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    </plane>
                </geometry>
                </collision>
                <visual name="visual">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    <size>100 100</size>
                    </plane>
                </geometry>
                <material>
                    <ambient>0.8 0.8 0.8 1</ambient>
                    <diffuse>0.8 0.8 0.8 1</diffuse>
                    <specular>0.8 0.8 0.8 1</specular>
                </material>
                </visual>
            </link>
        </model>

        <model name='vehicle_blue' canonical_link='chassis'>
            <pose relative_to='world'>0 0 0 0 0 0</pose>   <!--alapbeállítás szerint a megadott póz a világ koordinátáihoz képest értendő-->

            <!--karosszéria-->
            <link name='chassis'>
                <pose relative_to='__model__'>0.5 0 0.4 0 0 0</pose>
                <inertial>
                    <mass>1.14395</mass>
                    <inertia>
                        <ixx>0.095329</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.381317</iyy>
                        <iyz>0</iyz>
                        <izz>0.476646</izz>
                    </inertia>
                </inertial>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>2.0 1.0 0.5</size>
                        </box>
                    </geometry>
                    <!--Az összetevő anyagjellemzői (színe)-->
                    <material>
                        <ambient>0.0 0.0 1.0 1</ambient>
                        <diffuse>0.0 0.0 1.0 1</diffuse>
                        <specular>0.0 0.0 1.0 1</specular>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>2.0 1.0 0.5</size>
                        </box>
                    </geometry>
                </collision>
            </link>
            
            <!--Bal kerék-->
            <link name='left_wheel'>
                <pose relative_to="chassis">-0.5 0.6 0 -1.5707 0 0</pose>
                <inertial>
                    <mass>1</mass>
                    <inertia>
                        <ixx>0.043333</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.043333</iyy>
                        <iyz>0</iyz>
                        <izz>0.08</izz>
                    </inertia>
                </inertial>
                <visual name='visual'>
                    <geometry>
                        <cylinder>
                            <radius>0.4</radius>
                            <length>0.2</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <ambient>1.0 0.0 0.0 1</ambient>
                        <diffuse>1.0 0.0 0.0 1</diffuse>
                        <specular>1.0 0.0 0.0 1</specular>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <cylinder>
                            <radius>0.4</radius>
                            <length>0.2</length>
                        </cylinder>
                    </geometry>
                </collision>
            </link>

            <!--Jobb kerék (ugyanaz, mint a bal kerék, a pozíció tükrözésével)-->
            <link name='right_wheel'>
                <pose relative_to="chassis">-0.5 -0.6 0 -1.5707 0 0</pose>
                <inertial>
                    <mass>1</mass>
                    <inertia>
                        <ixx>0.043333</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.043333</iyy>
                        <iyz>0</iyz>
                        <izz>0.08</izz>
                    </inertia>
                </inertial>
                <visual name='visual'>
                    <geometry>
                        <cylinder>
                            <radius>0.4</radius>
                            <length>0.2</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <ambient>1.0 0.0 0.0 1</ambient>
                        <diffuse>1.0 0.0 0.0 1</diffuse>
                        <specular>1.0 0.0 0.0 1</specular>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <cylinder>
                            <radius>0.4</radius>
                            <length>0.2</length>
                        </cylinder>
                    </geometry>
                </collision>
            </link>

            <!--Tetszőleges frame-->
            <frame name="caster_frame" attached_to='chassis'>
                <pose>0.8 0 -0.2 0 0 0</pose>
            </frame>

            <!--Támasztógörgő-->
            <link name='caster'>
                <pose relative_to='caster_frame'/>
                <inertial>
                    <mass>1</mass>
                    <inertia>
                        <ixx>0.016</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.016</iyy>
                        <iyz>0</iyz>
                        <izz>0.016</izz>
                    </inertia>
                </inertial>
                <visual name='visual'>
                    <geometry>
                        <sphere>
                            <radius>0.2</radius>
                        </sphere>
                    </geometry>
                    <material>
                        <ambient>0.0 1 0.0 1</ambient>
                        <diffuse>0.0 1 0.0 1</diffuse>
                        <specular>0.0 1 0.0 1</specular>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <sphere>
                            <radius>0.2</radius>
                        </sphere>
                    </geometry>
                </collision>
            </link>

            <!--Bal kerék joint-->
            <joint name='left_wheel_joint' type='revolute'>
                <pose relative_to='left_wheel'/>
                <parent>chassis</parent>
                <child>left_wheel</child>
                <axis>
                    <xyz expressed_in='__model__'>0 1 0</xyz> 
                    <limit>
                        <lower>-1.79769e+308</lower>    <!--negatív végtelen-->
                        <upper>1.79769e+308</upper>     <!--pozitív végtelen-->
                    </limit>
                </axis>
            </joint>

            <!--Jobb kerék joint-->
            <joint name='right_wheel_joint' type='revolute'>
                <pose relative_to='right_wheel'/>
                <parent>chassis</parent>
                <child>right_wheel</child>
                <axis>
                    <xyz expressed_in='__model__'>0 1 0</xyz>
                    <limit>
                        <lower>-1.79769e+308</lower>    <!--negatív végtelen-->
                        <upper>1.79769e+308</upper>     <!--pozitív végtelen-->
                    </limit>
                </axis>
            </joint>

            <!--Támasztógörgő joint-->
            <joint name='caster_wheel' type='ball'>
                <parent>chassis</parent>
                <child>caster</child>
            </joint>
        </model>
    </world>
</sdf>
```
## Moving the Robot Platform

To move the robot we previously assembled, we will use a plugin, specifically the `diff_drive` plugin.

Open the previously created `building_robot.sdf` file and within the `vehicle_blue` `<model>` tags, invoke the plugin and define the basic parameters needed for its use:

```xml
<plugin
    filename="libignition-gazebo-diff-drive-system.so"
    name="ignition::gazebo::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
```

The `<plugin>` tag has two attributes. One is the name of the library from which the plugin originates (`filename`), and the other is the plugin name (`name`). The additional tags define the characteristics of the differential drive robot:
- `<left_joint>` and `<right_joint>`: the joints that define the connection between the robot's left and right wheels and the robot chassis.
- `<wheel_separation>`: the distance between the driven wheels, i.e., the track width. Since we previously specified that the positions of the right and left wheels along the Y-axis are -0.6m and 0.6m, the wheel distance is 1.2m.
- `<wheel_radius>`: the radius of the driven wheels.
- `<odom_publish_frequency>`: the frequency at which we want to publish the odometry calculated by the plugin.

With these parameters set, our model is ready to move. The next step is to send commands to it, which can be done using the `cmd_vel` topic.

1. Start the robot with manual command input

- In one terminal, start the simulation:
```bash
ign gazebo building_robot.sdf
```

- From another terminal, send a command to the robot:
```bash
ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
```

- Press the play button in the simulator.

Following these steps, the robot model should start moving.

2. Move the robot using the keyboard

Now we will control the robot by reading the keyboard, also using a ROS2 topic. This will require two additional plugins: `KeyPublisher` and `TriggeredPublisher`.

The `KeyPublisher` is an `ign-gui` plugin that reads keyboard key presses and sends them to the `/keyboard/keypress` topic. Let's try this plugin:

- Again, start the simulator in one terminal:

```bash
ign gazebo building_robot.sdf
```

- In the top right corner of the simulator window, click on the `plugins` dropdown list, then select the `Key Publisher` option.

- In another terminal, enter the following to print all keyboard presses:

```bash
ign topic -e -t /keyboard/keypress
```

The next step is to map the key presses to commands suitable for controlling the robot. We will use the `TriggeredPublisher` plugin for this.

The `TriggeredPublisher` plugin creates an output based on a given input in a way we define. In the `building_robot.sdf` file, within the `<world>` tags, specify the following mapping:

```xml
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777235</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.5}, angular: {z: 0.0}
    </output>
</plugin>
```

Try controlling the robot:

- Start the simulator again:

```bash
ign gazebo building_robot.sdf
```

- Select the `Key Publisher` plugin.

- Ensure the simulation is running, press the Play button if necessary.

- Press the `Up` (↑) arrow key. The robot should start moving forward.

## Independent Task

Create the functions for all arrow keys by extending the previous code snippet. The task can be solved analogously by modifying the parameters using the following mappings:

- Left arrow, value: `16777234`, parameters: `linear: {x: 0.0}, angular: {z: 0.5}`
- Up arrow, value: `16777235`, parameters: `linear: {x: 0.5}, angular: {z: 0.0}`
- Right arrow, value: `16777236`, parameters: `linear: {x: 0.0}, angular: {z: -0.5}`
- Down arrow, value: `16777237`, parameters: `linear: {x: 0.5}, angular: {z: 0.0}`

## Extending the Environment

The simulated environment created so far only contains a ground plane and sunlight. Let's create additional environmental elements by adding primitive static elements. Start by creating a single rectangular "wall":

```xml
<model name='wall'>
    <static>true</static>
    <pose>5 0 0 0 0 0</pose><!--pose relative to the world-->
    <link name='box'>
        <pose/>
        <visual name='visual'>
            <geometry>
                <box>
                    <size>0.5 10.0 2.0</size>
                </box>
            </geometry>
            <!--add material (color)-->
            <material>
                <ambient>0.0 0.0 1.0 1</ambient>
                <diffuse>0.0 0.0 1.0 1</diffuse>
                <specular>0.0 0.0 1.0 1</specular>
            </material>
        </visual>
        <collision name='collision'>
            <geometry>
                <box>
                    <size>0.5 10.0 2.0</size>
                </box>
            </geometry>
        </collision>
    </link>
</model>
```

## Independent Task

Add two more elements to the environment with the following parameters:
1. Element
    - name: wall1
    - pose: (0 12 0 0 0 1.5707)
    - size: (0.5 10.0 2.0)
    - material, color: optional

2. Element
    - name: wall2
    - pose: (0 -12 0 0 0 1.5707)
    - size: (0.5 10.0 2.0)
    - material, color: optional

## Adding a Sensor

In the previous sections, we created a movable robot simulation, but for autonomous operation, some sensor simulation is necessary. In the following steps, we will add an IMU (Inertial Measurement Unit) sensor and a LiDAR sensor to the previously created robot.

- IMU Sensor

The IMU sensor provides three distinct pieces of information:
- The sensor's orientation in quaternion format.
- The sensor's angular velocity around the (X, Y, Z) axes.
- The sensor's linear acceleration along the (X, Y, Z) axes.

The IMU sensor can also be added using a plugin. Define the IMU sensor by editing the previously created file within the `<world>` tags:

```xml
<plugin filename="libignition-gazebo-imu-system.so"
        name="ignition::gazebo::systems::Imu">
</plugin>
```

After defining the plugin, define the parameters for the sensor. The sensor will return the parameters of the link to which it is assigned. Since we want to measure the robot, or more specifically the robot chassis, we specify this link:

```xml
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>1</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
</sensor>
```

The applied parameters are:
- `<always_on>`: if the value is 1, the sensor will always update its output data according to the update rate.
- `<update_rate>`: the update frequency of the output data.
- `<visualize>`: if the value is 1, the sensor representation will be visually displayed.
- `<topic>`: the name of the topic containing the output data.

Try the created sensor.

- After saving, start the simulator:

```bash
ign gazebo building_robot.sdf
```

- In another terminal, print the IMU data. If you move the robot with the keyboard, changes will be observed in the sensor data:

```bash
ign topic -e -t /imu
```

# Sources
- [gazebosim.org/docs/fortress/sdf_worlds](https://gazebosim.org/docs/fortress/sdf_worlds)
- [gazebosim.org/docs/fortress/building_robot](https://gazebosim.org/docs/fortress/building_robot)
- [gazebosim.org/docs/fortress/moving_robot](https://gazebosim.org/docs/fortress/moving_robot)