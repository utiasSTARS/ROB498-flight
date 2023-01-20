# Getting Started With Your Jetson Nano Dev Kit

The provided NVIDIA Jetson Nano Dev Kit is a mini computer widely used amongst robotics enthusiasts. In this course, we will use the Jetson Nano as the computing unit for the drone to complete a set of autonomous flying challenges. Before starting, you should take a look at the official [NVIDIA Instruction](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) on how to set up your device.

NVIDIA provides an [SD card image](https://developer.nvidia.com/jetson-nano-sd-card-image) which installs a custom version of Ubuntu 18 on your Jetson. We also provide a modified version of the image that comes with a set of pre-installed packages for easier setup.

To use the pre-configured image, first download the zip file [here](https://drive.google.com/file/d/1c-AUyDF2ZgA6t0d_pnyBmTgDt-NZ41I6/view?usp=share_link) and unzip it on your host Ubuntu machine. Note that you will need at least 64GB of free disk space to store the image file. Next, connect the provided microSD card to your host computer and open the **Disks** app. Select the SD card from the drive list on the left and then use the **Restore Disk Image...** option in the menu to flash the disk image to the microSD, as shown in the figure below. The process will take about 20 minutes.

<img src = "https://github.com/utiasSTARS/ROB498-flight/blob/0f973e2960fc59ea7a5dc7dec1d59a7b93ecab13/instructions/images/flash_sd.png">

Once finished, insert the microSD into the Jetson and power it on. The username and password are both *rob498*. The system comes with the following packages:
- ROS Melodic
- MAVROS
- Intel Realsense SDK 2.0
- Auterion VIO

## ROS - Robot Operating System

ROS is a set of libraries and tools that help people build robotic software. If you have never used ROS before, we encourage you to go through the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials). In the following sections we will use ROS as the communication platform to talk to the sensors. However, this is not mandatory and you can use other software such as *MAVLink*.

## MAVROS

MAVROS is a ROS package that provides a wrapper for the MAVLink protocol. We use it as a bridge between the Jetson Nano and other devices such as the Pixhawk 4 Mini flight controller and the Intel Realsense T265 Tracking Camera.

## Intel Realsense VIO

The Intel Realsense T265 Tracking Camera provides onboard visual-interial odometry capability to your drone. We will use the estimated pose from the T265 camera to localize your drone when flying. The pre-installed [*Auterion VIO*](https://github.com/Auterion/VIO) package provides an easy way to communicate with the camera and obtain the estimated pose through *MAVROS*.

# Communication Between Jetson Nano and Pixhawk 4 mini

## Wiring Between Jetson Nano and Pixhawk 4 Mini

The Pixhawk 4 Mini's **Telem 1** port can be connected to Jetson Nano's **GPIO** pins using the provided JST-GH-to-Dupont jumper wire. This [video](https://www.youtube.com/watch?v=nIuoCYauW3s) provides a good instruction on how to connect the pins. Since the jumper wire is provided, you do not need to do any soldering. In summary, you will
- Connect **Pin 2** of **Telem 1** (TX/out) on Pixhawk 4 Mini to **Pin 10** of **GPIO** (UART_RX/in, /dev/ttyTHS1) on Jetson Nano
- Connect **Pin 3** of **Telem 1** (RX/in) on Pixhawk 4 Mini to **Pin 8** of **GPIO** (UART_TX/out, /dev/ttyTHS1) on Jetson Nano
- Connect **Pin 6** of **Telem 1** (Ground) on Pixhawk 4 Mini to **Pin 9** of **GPIO** (Ground) on Jetson Nano

## Configure Pixhawk 4 Mini

Before the two devices can communicate with each other, we need to set some parameters on Pixhawk 4 Mini. Download and install **QGroundControl (QGC)** on your host computer by following the instruction [here](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html). Connect the Pixhawk 4 Mini to the computer and once it shows up in QGC, set the following parameters ([tutorial](https://docs.qgroundcontrol.com/master/en/SetupView/Parameters.html))
- *SER_TEL1_BAUD* to 921600
- *MAV_0_RATE* to 921600

You may also [calibrate the sensors](https://docs.qgroundcontrol.com/master/en/SetupView/sensors_px4.html) in QGC.


## Configure Jetson Nano

The Jetson Nano will communicate with the Pixhawk 4 Mini through the */dev/ttyTHS1* terminal and we need to grant read and write permission. In a terminal on Jetson Nano, execute the following command

`sudo chmod 666 /dev/ttyTHS1`

## Test Connection - Get Pose From Pixhawk!

Now the two devices are ready to talk to each other! To test the communication, we can run the following commands on Jetson
- In the first terminal, launch ROS by running `roscore`
- In the second terminal, launch MAVROS by running `roslaunch mavros px4.launch`. The launch file already contains the necessary modifications to communicate with the Pixhawk
- MAVROS by default publishes information at a very low rate. We can increase the publish rate by running `rosservice call /mavros/set_message_interval TOPIC_ID DESIRED_RATE` in the third terminal. Topic IDs can be found [here](https://mavlink.io/en/messages/common.html). For example, if we want to publish odometry (pose) at 100 hz then we can run `rosservice call /mavros/set_message_interval 331 100` where 331 is the ID for odometry.
- To verify, we can check the publish rate by running `rostopic hz /mavros/odometry/in` in the fourth terminal. You can also use *rviz* to visualize the received poses.

# Use Realsense T265 Tracking Camera For Pose Estimation

The Pixhawk Mini only uses the internal IMU to estimate its pose, which could drift over time. Fortunately, the Realsense T265 camera can use both the IMU and viusal feature to provide more stable pose estimations. First, you need to connect the camera to the Jetson using the provided USB3.0 cable. The Auterion VIO package is installed at `~/thirdparty/vio_ws`. Source this ROS workspace and run `roslaunch px4_realsense_bridge bridge_mavros.launch` to launch the bridge. The estimated poses should be published under the `/mavros/odometry/out` topic.   
