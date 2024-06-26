# LIAT DEMO ROBOT
## Introduction
This package enables the control of a mobile robot and a robotic arm using a single joystick. It has been specifically tested with the HunterSE mobile robot and the ViperX robotic arm, model vx300s. The arm uses inverse kinematics to get the end-effector of the robot. 

Note: This package was provided using ROS2 version humble and the control joystick Logitech F710 and a PS4 control.

In this package, you can launch the nodes for use only the arm, only the camera, the arm with the camera, only the Hunter robot mobile, or all nodes.

If you need to change the button control, you can do so in the xarm_joy.cpp node for the robot arm, but if you want to change the button on the control for the hunter robot, you do the change in the config file.

## Additional required ros2 packages for usage
- [Hunter Mobile Base](https://github.com/OctavioPinoRosas/hunter_ros2.git)
- [Interbotix_ros_xsarms](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)
- [Ugv_sdk](https://github.com/agilexrobotics/ugv_sdk.git)
- [Joy](https://github.com/ros-drivers/joystick_drivers)
- [Teleop_twist_joy](https://github.com/ros2/teleop_twist_joy)
- [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/realsense-ros.git)
- [Rqt_image_view](https://wiki.ros.org/rqt_image_view?distro=humble)

## Installation
If you don't have a directory, create one (the following instructions assume your workspace is at ~/ros2_ws):
```
mkdir -p ~/ros2_ws/src
```
Initialize the workspace:
```
cd ~/ros2_ws
colcon build --symlink-install
. ~/ros2_ws/install/setup.bash
```

### Installation of [joy](https://github.com/ros-drivers/joystick_drivers) package
Verify that you have installed dependent packages joy:
```
ros2 pkg list | grep joy
```
If you don't have it, then install it:
```
sudo apt update
sudo apt install ros-DISTRO-joy
```
Change DISTRO with your ros2 version.

Finally, check if you have installed dependent packages joy.

For more information visit the [documentation](https://index.ros.org/p/joy/)

### Installation of [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy.git) package
Verify that you have installed dependent packages joy:
```
ros2 pkg list | grep teleop_twist_joy
```
If you don't have it then install it:
```
sudo apt update
apt-get install ros-DISTRO-teleop-twist-joy
```
Change DISTRO with your ros2 version.

Finally, check if you have installed dependent packages teleop_twist_joy.

For more information visit the [documentation](https://index.ros.org/p/teleop_twist_joy/)

### Installation of [rqt-image-view](https://index.ros.org/p/rqt_image_view/#humble)
Verify that you have installed dependent packages rqt:
```
ros2 pkg list | grep rqt_image_view
```
If you don't have it then install it:
```
sudo apt update
sudo apt install ros-DISTRO-depth-image-view
```
Change DISTRO with your ros2 version.

Finally, check if you have installed dependent packages rqt.

For more information visit the [documentation](https://wiki.ros.org/rqt_image_view)

### Installation of [interbotix_ros_xsarms](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)
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
./xsarm_amd64_install.sh -h./xsarm_amd64_install.sh -h
```
USAGE: ./xsarm_amd64_install.sh [-h][-d DISTRO][-p PATH][-n]
2. Installation Checks

After running the installation script on the robot computer, we can verify that the script ran successfully.
- Udev rules

Check that the udev rules were configured correctly and that they are triggered by the U2D2. This can be done by checking that the port name shows up as ttyDXL when the U2D2 is plugged into a USB port. The command and the expected output are below:
```
ls /dev | grep ttyDXL
```
You can see:
`ttyDXL`

For more information visit the [documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)

### Installation of [Hunter Mobile Base](https://github.com/OctavioPinoRosas/hunter_ros2.git) and  [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk.git) packages
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
- First time using hunter-ros2 package
```
cd ~/ros2_ws/src/ugv_sdk/scripts/
bash setup_can2usb.bash
```
- If not the first time use hunter-ros2 package(Run this command every time you turn off the power)
```
cd ~/ros2_ws/src/ugv_sdk/scripts/
bash bringup_can2usb_500k.bash
```
- Testing command
Receiving data from can0
```
candump can0
```

For more information visit the [documentation](https://github.com/agilexrobotics/ugv_sdk.git)

### Installation of camera package [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/realsense-ros.git)
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
For more information visit the [documentation](https://github.com/IntelRealSense/realsense-ros.git)

## Usage
First time using hunter-ros2 package
```
sudo ip link set can0 up type can bitrate 500000
```
Check the connection with the Hunter robot.
Testing command:
Receiving data from can0
```
candump can0
```
Check the connection with the arm. This can be done by checking that the port name shows up as ttyDXL when the U2D2 is plugged into a USB port.
```
ls /dev | grep ttyDXL
```
you can see:
`ttyDXL`

To start the nodes for the robots, open a terminal window and run the first command in one window, and the second command in another window:
- **Start the arm robot and camera installed in it:**
```
ros2 launch liat_demo_robot liat_demo_arm-camera.launch.py
```
If you can't see the image in the interfaze of image view, select the camera and image type in the top left corner of the image view interface.

Note: To watch the image in the rviz do click in "Add", then en "By topic" and finally select image.

If only want to use the robotic arm without the camera: `ros2 launch liat_demo_robot demo_arm.launch.py`

- **Start the Hunter mobile robot:**
```
ros2 launch liat_demo_robot liat_demo_hunter-teleop.launch.py
```

### Buttons map
To understand how the joystick buttons map to control the robot, look at the diagrams and tables below.

Note: The following driamans and tables were made using the "xbox" option with the Logitech F710 controller, although the use of PS controllers has some differences, the button placement is essentially the same

- **Robotic arm button map**

![](https://github.com/OctavioPinoRosas/liat_demo_robot/blob/main/images/Control_Arm.jpeg)

- **Table of buttons function for the robotic arm**

| LB + Button | Action |
|:------------|:---------------|
| Star | move robot arm to its Home pose |
| Back | move robot arm to its Sleep pose |
| A | rotate the ‘waist’ joint clockwise |
| Y | rotate the ‘waist’ joint counterclockwise |
| B | open gripper |
| x | close gripper |
| D-pad Up | increase the control loop rate in 1 Hz step increments (max of 40) |
| D-pad Down | decrease the control loop rate in 1 Hz step increments (min of 10) |
| D-pad Left | go to position x=0.3, z=-0.2,pitch=1.5|
| D-pad Right | do the trayectory: open gripper; x=0.3, z=-0.26,pitch=1.5; close gripper; and x=0.55, z=0.4 |
| Right stick Up/Down | increase/decrease the pitch of the end-effector
| Right stick Left/Right | increase/decrease roll of the end-effector
| R3 | reverses the Right stick Left/Right control
| Left stick Up/Down | move the end-effector (defined at ‘ee_gripper_link’) vertically in Cartesian space
| Left stick Left/Right | move the end-effector (defined at ‘ee_gripper_link’) horizontally in Cartesian space
| L3 |reverses the Left stick Left/Right control
| RT | if the arm has 6dof, this moves the end-effector in a negative direction along its own ‘y’ axis
| LT | if the arm has 6dof, this moves the end-effector in a positive direction along its own ‘y’ axis
| Logitech | if torqued on, holding for 3 seconds will torque off the robot; if torqued off, tapping the button will torque on the robot

**Note:**

To move the arm robotic hold down the LB button always.

If when you move the joysticks the robotic arm't move, push back (Sleep) first.

After to push the button D-pad Left/ D-pad right to move the arm to a established position or trajectory, push Star (Home) or Back(Sleep) to unlock the arm and move normaly. To configure correctly the buttons check the section "Customize launches files and other settings".

- **Hunter robot button map**

![](https://github.com/OctavioPinoRosas/liat_demo_robot/blob/main/images/Control_Hunter.jpeg)

- **Table of buttons function for hunter robot**

| Combination of buttons | Action |
|:------------|:---------------|
| RB + left stick Up | move forward |
| RB + left stick Down | move backward |
| RB + left stick up + right stick left | forward turn left|
| RB + left stick up + right stick right | forward turn right |
| RB + left stick Down + right stick left | backward turn right |
| RB + left stick Down + right stick right | backward turn left |

Note: To move the Hunter robot hold down the RB button.
For turbo mode push L3 betwin RB in Xbox control, and R2 button in PS4 control.

If you push the LB and RB buttons to move the Hunter and the arm robotic, only the Hunter will move.

### Customize launches files and other settings

To customize launches files when you use the robots follow instructions below. 
Theres some intruction to modify the end position and trajectory estrablised.

#### Arm robot

- To further customize the launch file at run-time for the robotic arm, look at the table below:

| Argument | Description | Default | Choices |
|----------|-------------|---------|---------|
| `robot_model` | Model type of the Interbotix Arm such as wx200 or rx150. | `vx300s` | `px100`, `px150`, `rx150`, `rx200`, `wx200`, `wx250`, `wx250s`, `vx250`, `vx300`, `vx300s`, `mobile_px100`, `mobile_wx200`, `mobile_wx250s`|
| `controller` | Type of controller. | `xbox` | `ps4`, `ps3`, `xbox` |
| `use_rviz` | Launches RViz if set to true. | `true` | `true`, `false` |

For more customization option and information visit the [documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/joystick_control.html)

- Modification to can use the buttons to position and trajectory established

To should use the buttons to move the arm to the position and trajectory established you need to change the path where files when describe that.
In the file "src/xsarm_joy.cpp" change `/home/liat05/ros2_ws` with the path where is you workspace. The part of code to modify is:
```
static const std::string pose1_script = "/home/liat05/ros2_ws/src/liat_demo_robot/scripts/position.py";
static const std::string pose2_script = "/home/liat05/ros2_ws/src/liat_demo_robot/scripts/trajectory.py";
```

- Modify the move position established

To modify the position established go to the file "Scripts/position.py"

- Modify the trajectory established

To modify the trajectory established go to the file "Scripts/trajectory.py"

#### Hunter robot

- To configure the control in the Hunter robot launch file

| Argument | Description | Default | Choices |
|----------|-------------|---------|---------|
| `joy_config` | Type of controller. | `xbox` | `ps`, `xbox` |

Example:
`ros2 launch liat_demo_robot liat_demo_hunter-teleop.launch.py joy_config:=ps`

Note: the option `ps` works with PS3 and PS4, and the option `xbox` funtions with Xbox360 and similar, like the control Logitech F710.