# LIAT DEMO ROBOT
## Introduction
This package enables the control of both a mobile robot and a robotic arm using a single joystick. It has been specifically tested with the HunterSE mobile robot and the ViperX robotic arm, model vx300s.

## Additional required ros2 packages for usage
- [Hunter Mobile Base](https://github.com/OctavioPinoRosas/hunter_ros2.git)
- [Interbotix_ros_xsarms](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)
- [Ugv_sdk](https://github.com/agilexrobotics/ugv_sdk.git)
- [Joy](https://github.com/ros-drivers/joystick_drivers)
- [Teleop_twist_joy](https://github.com/ros2/teleop_twist_joy)
- [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/realsense-ros.git)

## Installation
If you don´t have a directory, create one
(the following instructions assume your workspace is at: ~/ros2_ws)
```
mkdir -p ~/ros2_ws/src
```
inializace the workspace
```
cd ~/ros2_ws
colcon build --symlink-install
. ~/ros2_ws/install/setup.bash
```

### Instalation of [joy](https://github.com/ros-drivers/joystick_drivers) package
Verify that you have installed dependent packages joy
```
ros2 pkg list | grep joy
```
If you don't have it, then install it
```
sudo apt update
sudo apt install ros-DISTRO-joy
```
Change DISTRO with your ros2 version
Finally check if you have installed dependent packages joy

For more information visit the documentation [documentation](https://index.ros.org/p/joy/)

### Instalation of [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy.git) package
Verify that you have installed dependent packages joy
```
ros2 pkg list | grep teleop_twist_joy
```
If you don't have it then install it
```
sudo apt update
apt-get install ros-DISTRO-teleop-twist-joy
```
Change DISTRO with your ros2 version
Finally check if you have installed dependent packages teleop_twist_joy

For more information visit the documentation [documentation](https://index.ros.org/p/teleop_twist_joy/)

### Instalation of [interbotix_ros_xsarms](https://github.com/Interbotix/interbotix_ros_manipulators.git)
1. Software Installation

If your computer uses an Intel or AMD based processor (which is the case for NUCs, most laptops and desktop computers), follow the commands below to download and run the installation script. Specify the version of ROS 2 that you want to install using the -d flag followed by the distribution’s codename.
```
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d DISTRO
```
The install script provides more in-depth control of some installation options. Append the -h flag to see the help document like below:
```
`./xsarm_amd64_install.sh -h./xsarm_amd64_install.sh -h`
```
USAGE: ./xsarm_amd64_install.sh [-h][-d DISTRO][-p PATH][-n]
The commands below demonstrate the process of running the installation script for ROS 2 Humble and assume your workspace is at ros2_ws
```
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble -p ~/ros2_ws/src
```

2. Installation Checks

After running the installation script on the robot computer, we can verify that the script ran successfully.
- udev Rules
Check that the udev rules were configured correctly and that they are triggered by the U2D2. This can be done by checking that the port name shows up as ttyDXL when the U2D2 is plugged into a USB port. The command and the expected output are below:
```
ls /dev | grep ttyDXL
```
you can see:
`ttyDXL`
- Interbotix ROS Packages
Check that the Interbotix ROS packages were installed correctly. The command and example output are below:
```
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/interbotix_ws/install/setup.bash
ros2 pkg list | grep interbotix
```
You can see:
```
...
interbotix_common_modules
interbotix_common_sim
interbotix_common_toolbox
interbotix_perception_modules
interbotix_perception_msgs
interbotix_perception_pipelines
interbotix_perception_toolbox
interbotix_ros_xsarms
interbotix_ros_xsarms_examples
interbotix_ros_xseries
interbotix_tf_tools
interbotix_xs_driver
interbotix_xs_modules
interbotix_xs_msgs
interbotix_xs_ros_control
interbotix_xs_rviz
interbotix_xs_sdk
interbotix_xs_toolbox
interbotix_xsarm_control
interbotix_xsarm_descriptions
interbotix_xsarm_dual
interbotix_xsarm_joy
interbotix_xsarm_moveit
interbotix_xsarm_moveit_interface
interbotix_xsarm_perception
interbotix_xsarm_ros_control
interbotix_xsarm_sim
...
```
Specific packages you should confirm have been built are interbotix_xs_sdk, interbotix_xs_msgs, interbotix_common_modules, and interbotix_xs_modules. These serve as the fundamental core of the ROS 2 Interface and are required to use it. If these are missing, check the installation script’s output for errors.

For more information visit the documentation [documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)

### Instalation of [Hunter Mobile Base](https://github.com/OctavioPinoRosas/hunter_ros2.git) and  [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk.git) packages
1. Clone the packages into your catkin workspace and compile
```cd ~/ros2_ws/src
git clone https://github.com/agilexrobotics/ugv_sdk.git
git clone https://github.com/OctavioPinoRosas/hunter_ros2.git
cd ..
colcon build
```

3. Setup CAN-To-USB adapter
- Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
```
$ sudo modprobe gs_usb
```
- First time use hunter-ros2 package
```
cd ~/your_ws/src/ugv_sdk/scripts/
bash setup_can2usb.bash
```
- If not the first time use hunter-ros2 package(Run this command every time you turn off the power)
```
cd ~/catkin_ws/src/ugv_sdk/scripts/
bash bringup_can2usb_500k.bash
```
- Testing command
```
# receiving data from can0
candump can0
```

For more information visit the documentation documentation()

### Instalation of camera package [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/realsense-ros.git)
1. Install latest Intel® RealSense™ SDK 2.0
- Register the server's public key:
```
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```
- Make sure apt HTTPS support is installed:
`sudo apt-get install apt-transport-https`
- Add the server to the list of repositories:
```
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```
- Install the libraries (see section below if upgrading packages):
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.
- Optionally install the developer and debug packages:
`sudo apt-get install librealsense2-dev`
`sudo apt-get install librealsense2-dbg`
With dev package installed, you can compile an application with librealsense using `g++ -std=c++11 filename.cpp -lrealsense2` or an IDE of your choice.

Reconnect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.

Verify that the kernel is updated :
`modinfo uvcvideo | grep "version:"` should include `realsense` string

2. Install Intel® RealSense™ ROS2 wrapper

Option 1: Install debian package from ROS servers
- [Configure](http://wiki.ros.org/Installation/Ubuntu/Sources) your Ubuntu repositories
- Install all realsense ROS packages by ```sudo apt install ros-<ROS_DISTRO>-realsense2-*```
- For example, for Humble distro: ```sudo apt install ros-humble-realsense2-*```

Option 2: Install from source
```
cd ~/ros2_ws/src/
```

- Clone the latest ROS2 Intel&reg; RealSense&trade;  wrapper from [here](https://github.com/IntelRealSense/realsense-ros.git) into '~/ros2_ws/src/'
```bash
bashrc
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd ~/ros2_ws
```

- Install dependencies
```bash
sudo apt-get install python3-rosdep -y
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
```
- Build
```
colcon build
```

- Source environment
```
ROS_DISTRO=<YOUR_SYSTEM_ROS_DISTRO>  # set your ROS_DISTRO: iron, humble
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws
. install/local_setup.bash
```
For more information visit the documentation [documentation](https://github.com/IntelRealSense/realsense-ros.git)

## Usage
irst time use hunter-ros2 package
```
sudo ip link set can0 up type can bitrate 500000
```
Check the conexion with the arm. This can be done by checking that the port name shows up as ttyDXL when the U2D2 is plugged into a USB port.
```
ls /dev | grep ttyDXL
ttyDXL
```
Start the base node for the real robot
- Star the arm robot and camera installed in it
```
ros2 launch liat_demo_robot liat_demo_arm.launch.py
```
- Star the hunter mobile robot
```
ros2 launch liat_demo_robot liat_demo_hunter-teleop.launch.py
```

### Butons map
To understand how the joystick buttons map to controlling the robot, look at the diagram and table below. Note that while the PS has a slightly different naming convention, the button placement is essentially the same:

- **Buton map**

- **Table of buttons funtion to robot arm**

| LB + Buton | Action |
|:------------|:---------------|
| Star | move robot arm to its Home pose |
| Back | move robot arm to its Sleep pose |
| A | rotate the ‘waist’ joint clockwise |
| Y | rotate the ‘waist’ joint counterclockwise |
| B | open gripper |
| x | close gripper |
| D-pad Up | increase the control loop rate in 1 Hz step increments (max of 40)
| D-pad Down | decrease the control loop rate in 1 Hz step increments (min of 10)
| D-pad Left | coarse’ control - sets the control loop rate to a user-preset ‘fast’ rate
| D-pad Right | fine’ control - sets the control loop rate to a user-preset ‘slow’ rate
| Right stick Up/Down | increase/decrease pitch of the end-effector
| Right stick Left/Right | increase/decrease roll of the end-effector
| R3 | reverses the Right stick Left/Right control
| Left stick Up/Down | move the end-effector (defined at ‘ee_gripper_link’) vertically in Cartesian space
| Left stick Left/Right | move the end-effector (defined at ‘ee_gripper_link’) horizontally in Cartesian space
| L3 |reverses the Left stick Left/Right control
| RT | if the arm has 6dof, this moves the end-effector in a negative direction along its own ‘y’ axis
| LT | if the arm has 6dof, this moves the end-effector in a positive direction along its own ‘y’ axis
| Logitech | if torqued on, holding for 3 seconds will torque off the robot; if torqued off, tapping the button will torque on the robot

- **Table of buttons funtion to hunter robot**

| Combination of buttons | Action |
|:------------|:---------------|
| RB + left stick Up | move forward |
| RB + left stick Down | backward |
| RB + left stick up + right stick left | forward turn left|
| RB + left stick up + right stick right | forward turn right |
| RB + left stick Down + right stick left | backward turn right |
| RB + left stick Down + right stick right | backward turn left |