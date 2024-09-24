---
layout: default
title: Installing ROS 2 Humble
icon: material/code-block-tags # supplementary material
---

# ROS 2 Humble

!!! tip "Simple Installation"

    The installation can be done step-by-step, but we have also prepared a [simple shell script-based installation](#home-lab-installation).

As mentioned in the introduction, there are basically four options for installing `ROS 2 Humble`:

1. Dual boot, native Linux (mostly Ubuntu) installed alongside Windows ✅ [description](https://sze-info.github.io/arj/installation/ubuntu.html)
2. Windows WSL2, lightweight Linux virtual machine ✅ [description](https://sze-info.github.io/arj/installation/win10.html)
3. Virtual machine for Windows 🟠
4. Windows build 🟠

We recommend the first two options out of these four, but the others are not prohibited either. Dual boot provides insight into the world of Linux, which is useful knowledge for an engineer nowadays. Care must be taken during installation, as a wrong setting can cause data loss, so a backup is also recommended. WSL (Windows Subsystem for Linux) is a lightweight compatibility layer for running Linux-based elements on Windows 10 or Windows 11 systems. As shown in the following figure, the Linux kernel can access hardware elements (CPU, memory, GPU, etc.) just as easily as the Windows kernel. In contrast, the virtual machine (option 3) is a much slower solution using more abstraction layers, recommended for those who either have a very modern, fast machine or have already installed such systems. The native Windows build (option 4) is theoretically available, but since most of the documentation is available for Linux, it will require a lot of extra work.

Illustration of the first three options:

![WSL Overview](wsl_overview01.svg)

## Installation

The following description applies to Ubuntu 22.04 Jammy. *Note* that other versions are also supported, and installation and descriptions for them are available here: [docs.ros.org/en/humble/Installation/Alternatives.html](https://docs.ros.org/en/humble/Installation/Alternatives.html)

The following description is based on [docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html).

## Setting the Locale

!!! note 
    This step is usually optional.

Ensure that you have a locale that supports UTF-8.

```sh
locale # Check UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale # Check settings
```

## Setting Up Sources

You need to add the ROS 2 apt repository to your system.

First, ensure that the Ubuntu Universe repository is enabled.

``` r
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Add the ROS 2 GPG key with `apt`.
``` r
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Then add the repository to the sources list.
``` r
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
## Installing ROS2 Packages

Update:

``` r 
sudo apt update
```
ROS 2 packages are often built on updated Ubuntu systems. It is always recommended to ensure your system is up-to-date before installing new packages.
``` r 
sudo apt upgrade
```
Desktop installation: Install ROS, RViz, demos, tutorials:
``` r 
sudo apt install ros-humble-desktop
```
Developer tools, compilers, and other tools for building ROS packages:
``` r 
sudo apt install ros-dev-tools
``` 
Set up your environment by sourcing the following file:

``` r 
source /opt/ros/humble/setup.bash
```
Tip: This can also be done in the `.bashrc` file with `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`.

# Verifying the Installation

Verify the correctness of the installation with the `ros2 topic list` command.

``` r
$ ros2 topic list

/parameter_events
/rosout 
```
If everything is fine, the above two topics should appear. Then you can get to know the use of simple example nodes:[docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)

# Recommended Post-Installation Settings

## Console Colors

By default, the console output is not colored, but it is advisable to set this with the `RCUTILS_COLORIZED_OUTPUT` environment variable (even in `bashrc`). For example

``` r 
export RCUTILS_COLORIZED_OUTPUT=1 
``` 

![RCUTILS_COLORIZED_OUTPUT](https://github-production-user-asset-6210df.s3.amazonaws.com/11504709/248783932-a71a5d37-d49b-4508-93db-2e74a3c24365.gif)
Details:[docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html#id14](https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html#id14)

## `colcon_cd`

It is also advisable to set up the `colcon_cd` command, so you can quickly switch your working directory to a package directory. For example, the `colcon_cd some_ros_package` command quickly jumps to the `~/ros2_ws/src/some_ros_package` directory.

Details:[docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#setup-colcon-cd](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#setup-colcon-cd)

# Windows /installation

[In the terminal `install_humble.sh`](https://github.com/sze-info/arj/blob/main/docs/telepites/install_humble.sh)

``` bash
wget https://raw.githubusercontent.com/sze-info/arj/main/docs/telepites/install_humble.sh
```
``` bash
sudo chmod +x install_humble.sh
```
Home:
``` bash
./install_humble.sh
```
In Lab:
``` bash
./install_humble.sh campus
```
# Workspace reset

If you want to delete the entire `ros2_ws`, then re-clone and build it (takes about 5 minutes), you can do it with the following single long command:
``` bash
cd ~ ; rm ws_reset.sh; wget https://raw.githubusercontent.com/sze-info/arj/main/docs/telepites/ws_reset.sh; sudo chmod +x ws_reset.sh; ./ws_reset.sh
```