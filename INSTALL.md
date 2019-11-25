# Installation
For the complete Phantomx installation, please refer first to the **ROS2Learn** installation instructions:  [github/kkonen/ros2learn/Install](https://github.com/kkonen/ros2learn/blob/master/Install.md).

## Table of Contents
- [ROS 2.0](#ros-20)
- [Dependent tools](#dependent-tools)
- [MARA](#mara)
  - [Create a ROS workspace](#create-a-ros-workspace)
  - [Compile the workspace](#compile-the-workspace)
    - [Ubuntu 18](#ubuntu-18)
  - [OpenAI Gym](#openai-gym)
  - [gym-gazebo2](#gym-gazebo2)
    - [Provisioning](#provisioning)

## ROS 2.0

- **ROS 2 Dashing**.
   - Ubuntu 18: Install ROS 2 Desktop following the official instructions, binaries recommended. [Instructions](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/).

## Dependent tools
- **Gazebo 9.9.0**.
   - Install the latest available version of Gazebo via [one-liner instructions](http://gazebosim.org/tutorials?tut=install_ubuntu#Defaultinstallation:one-liner).
   
     ```
     curl -sSL http://get.gazebosim.org | sh
     ```
- ROS 2 extra packages
```
sudo apt update && sudo apt install -y \
ros-dashing-action-msgs \
ros-dashing-message-filters \
ros-dashing-yaml-cpp-vendor \
ros-dashing-urdf \
ros-dashing-rttest \
ros-dashing-tf2 \
ros-dashing-tf2-geometry-msgs \
ros-dashing-rclcpp-action \
ros-dashing-cv-bridge \
ros-dashing-image-transport \
ros-dashing-camera-info-manager

# Install OpenSplice RMW implementation. Required for dashing until default FastRTPS is fixed.
sudo apt install ros-dashing-rmw-opensplice-cpp

sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  python3-sip-dev \
  python3-numpy \
  wget

# Install TensorFlow CPU. Feel free to get the GPU version at https://www.tensorflow.org/install/gpu.
pip3 install tensorflow

# Additional utilities
pip3 install transforms3d billiard psutil

# Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
```
## Gazebo Plugins
Go to [**gazebo_ros_pkgs**](https://github.com/kkonen/gazebo_ros_pkgs)
and follow the install instructions!

## Phantomx
### Create a ROS workspace

Create the workspace and download source files:

```
mkdir -p ~/ros2_phantomx_ws/src
cd ~/ros2_phantomx_ws/src
git clone https://github.com/kkonen/PhantomX.git
```

### Compile the workspace

Please make sure you are not sourcing ROS1 workspaces via `bashrc` or any other way. Also make sure you are not sourcing any provisioning script from other ROS2 distribution compliant gym-gazebo2 installation, e.g. gym-gazebo2 `crystal`.

#### Ubuntu 18

Build the workspace using the `--merge-install` flag.

```
source /opt/ros/dashing/setup.bash
cd ~/ros2_phantomx_ws
RMW_IMPLEMENTATION=rmw_opensplice_cpp colcon build --merge-install
# Remove warnings
```

### OpenAI Gym

It is recommended to install Gym's latest version, which means using the source code. If you already installed Gym via pip3, you can uninstall it via `pip3 uninstall gym` to avoid overlapping:

```
cd ~
git clone https://github.com/openai/gym
cd gym
pip3 install -e .
```

### gym-gazebo2

Install the gym-gazebo2 toolkit.

If you are using [**ros2learn**](https://github.com/kkonen/ros2learn/tree/master):
```
cd ~/ros2learn/environments/gym-gazebo2
pip3 install -e .
```

If not:
```
cd ~ && git clone -b phantomx https://github.com/kkonen/gym-gazebo2
cd gym-gazebo2
pip3 install -e .
```

#### Provisioning

First we need setup ROS2, Phantomx ROS2 workspace and Gazebo:

```
#Navigate to module's root directory
cd gym-gazebo2
source provision/phantomx_setup.sh
```
You probably want to create an alias for that.

**Note**: In Dashing we need to use opensplice implementation of DDS, since Fast-RTPS and others are still buggy and not supported well in this use case. Please export the OpenSplice DDS implementation manually or use the provisioning script before running/training any example of the Phantomx enviroment.

```
export RMW_IMPLEMENTATION=rmw_opensplice_cpp
```

