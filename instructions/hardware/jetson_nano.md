# Getting Started With Your Jetson Nano

The NVIDIA Jetson Nano is a mini computer widely used amongst robotics enthusiasts. In this course, we will use the Jetson Nano as the computing unit for your drone, to complete a set of autonomous flight challenges. Before beginning, you should take a look at the official [NVIDIA Instructions](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) that explain how to set up your device.

The provided Jetson Nano computers should come preinstalled with Ubuntu 20.04 operating system. If this is not the case, please let the TA staff know.
<!-- NVIDIA provides an [SD card image](https://developer.nvidia.com/jetson-nano-sd-card-image) that contains a custom version of Ubuntu 18 for your Jetson. We also provide a modified version of the image that comes with a set of pre-installed packages for easier setup.

To use the pre-configured image, first download the zip file [here](https://drive.google.com/file/d/1c-AUyDF2ZgA6t0d_pnyBmTgDt-NZ41I6/view?usp=share_link) and unzip it on your Ubuntu host machine. Note that you will need at least 64 GB of free disk space to store the unzipped image file. Next, connect the provided microSD card via a reader to your host computer and open the **Disks** app. Select the SD card from the drive list on the left and then use the **Restore Disk Image...** option in the menu to flash the disk image to the microSD, as shown in the figure below. The process will take about 20 minutes.

<img src="../images/flash_sd.png"> -->

The username and password are both *jetson*. The system comes with the following packages installed:

- [ROS2 Foxy](https://docs.ros.org/en/foxy/Tutorials.html)
- [MAVROS] (http://wiki.ros.org/mavros)
- Intel Realsense SDK 2.0

## ROS2 - The Robot Operating System

ROS2 is a set of libraries and tools that help people build robot software. If you have never used ROS before, we encourage you to go through the [ROS tutorials](https://docs.ros.org/en/foxy/Tutorials.html) that are available online. In the following sections, we will use ROS as the communication platform to talk to various sensors and to the avionics. However, this is not mandatory and you can use other software such as MAVLink.

## MAVROS

[MAVROS](http://wiki.ros.org/mavros) is a ROS package that provides a wrapper for the MAVLink protocol. We use it as a bridge between the Jetson Nano and other devices such as the CubePilot Orange Cube+ flight controller.

To install and configure mavros please run the following commands

```
sudo apt remove modemmanager
sudo adduser ${USER} dialout

sudo apt install ros-foxy-mavros-extras

cd /opt/ros/foxy/lib/mavros/
sudo ./install_geographiclib_datasets.sh
```

Reboot the Jetson Nano computer after the above steps.

## Intel Realsense VIO

The Intel Realsense T265 Tracking Camera provides a fully self-contained visual-interial odometry capability to your drone. We will use the estimated pose from the T265 camera to localize the drone when flying.

The packages necessary to run the realsense camera can be installed by running the following commands:

```
sudo apt remove librealsense*
sudo apt install ros-foxy-realsense2-camera
```

To test your realsense camera, connect it to the Jetson Nano computer and launch the driver node:
```
ros2 launch realsense2_camera rs_launch.py
```

The VIO output can be accessed over the following topic:
```
ros2 topic echo /camera/pose/sample
```

If you get an error saying `RS2_USB_ACCESS_DENIED` or `No Realsense Device Found`, copy the UDEV rules [99-realsense-usb.rules](../../resources/configs/99-realsense-usb.rules) as follows

```
sudo cp 99-librealsense-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```

Reboot the jetson nano and reconnect the realsense camera and the above issues should go away.

# Communication Between the Jetson Nano and the flight controller

Setting up communication between the Jetson Nano in the flight controller requires two steps: first, wiring the two devices together, and second, configuring the appropriate communications parameters.

## Wiring Between the Jetson Nano and Cube

The Cube's **Telem 2** port can be connected to Jetson Nano's USB ports using the USB-UART converter provided in your kit. The USB-UART converter needs to be connected to the 6-pin JST-GH connector as shown in the following images. *Please follow the cable ordering when interfacing different cables*.

<p align="center">
<img src="../images/telem2_usb_uart2.jpg" width="300">
<img src="../images/telem2_usb_uart3.jpg" width="300">
</p>


<p align="center">
<img src="../images/telem2_usb_uart1.jpg" width="300">
<img src="../images/telem2_usb_uart4.jpg" width="300">
</p>

The bottom right figure shows the UWB-UART interfacae to JST-GH cable connected to TELEM2 port of the flight controller. The USB-UART interface can then be plugged into any of the USB ports on the jetson nano.

### Onboard Jetson Communication

Communication with the Jetson Nano is through the USB-UART module. The OrangeCube+ autopilot must be configured by changing a few parameters. The following sections mainly concern parameters that can be found in the **Parameters** section of QGroundControl (**Vehicle Setup/Parameters**).
Navigation to the `Parameters` section in the  First, enable Mavlink communication on TELEM2:

 - MAV_1_CONFIG      -> TELEM2

Reboot the flight controller. Next, you must configure the Cube to send and receive MAVLink messages at an acceptable rate. This can be done by setting the following parameters:

- MAV_1_MODE        -> Onboard
- SER_TEL2_BAUD     -> 921600 8N1

<p align="center">
    <img src = "../images/cube_telem2.png" width = "300">
</p>

The above figure shows JSG-GH cable connected to TELEM2 port of the cube.

<!-- ## Configure Orange Cube+

Before the two devices can communicate with each other, we need to set some parameters on Orange Cube+. Download and install **QGroundControl (QGC)** on your host computer by following the instructions [here](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html). Connect the Orange Cube+ to the computer and once it shows up in QGC, set the following parameters ([tutorial](https://docs.qgroundcontrol.com/master/en/SetupView/Parameters.html)):

- MAV_1_CONFIG -> TELEM2

Reboot the flight controller. Next, verify that the following parameter is set accordingly:

- SER_TEL2_BAUD to 921600

You may also [calibrate the sensors](https://docs.qgroundcontrol.com/master/en/SetupView/sensors_px4.html) in QGC. -->

## Configure the Jetson Nano

The Jetson Nano will communicate with the Cube through the `/dev/ttyUSB#` device (i.e.,a serial port). We need to identify the port of the USB-UART interface. To do this disconnect and reconnect the USB-UART interface and run the following command in a new terminal:

`dmesg`

At the bottom of the console log, the name of the interface should show up as `ttyUSB#`. In most cases this is `ttyUSB0`.

Next, we need to give persmissions to the user to talk to the port:

`sudo adduser ${USER}  dialout`

**NOTE** If you are using MAVROS to communicate with the Cube, make sure to modify the launch file to replace the default port `/dev/ttyACM0:57600` with `/dev/ttyUSB0:921600`.

## Receiving Vicon Poses via ROS Using Wifi

Vicon is a high precision tracking system that can provide real-time pose estimate to the drone during flight. We have step up the infrastructure so your Jetson Nano can access data from the Vicon sysetm using onboard Wifi in the format of ROS messages. This section provides the instruction on how to configure your Jetson in order to join the ROS network.

First, you need to connect your Jetson to our router. The SSID is `TP_LINK_ROB498` and the password is `rob498drones`. Once connected, you should set up static IP for the Jetson. To do so, open **Settings**, and navigate to the **Wi-Fi** tab. Click on the arrow beside the connection to open the network interface dialog box. Go to the **IPv4** tab, and change the **IPv4 Method** to **Manual**. In the **Address** box, enter **10.42.0.1xx** where *1xx* should be *100* + *your team number*. For example, if you were team 12 then your IP should be *10.42.0.112*. Next, in the **Netmask** box, enter **24**. Save and close the window. The setting will be applied after you reconnect to the network. Verify that your IP address has been updated using the **ifconfig** command.
  Now you should be able to subscribe to the Vicon pose messages using ROS. To verify, open a new terminal window and run the following command:

<!-- - `export ROS_MASTER_URI=http://10.42.0.100:11311`
- `export ROS_IP=*YOUR_IP*` -->
-  `ros2 topic echo /vicon/ROB498_Drone/ROB498_Drone`
 
 The Vicon poses will published as [PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) messages as well as in the `/tf` topic. If you do not see any messages being printed, reach out to your TA. Note that you must run the first two `export` commands in the terminal every time you want to launch a ROS node. 

## Interfacing with TA computer 

In order to interface with the TA computer (this is required for Vicon as well as running each of the challenges), you must set your ROS_DOMAIN_ID variable. This will ensure that other teams' nodes do not interfere with your flight. Set the ID to your team number to avoid collisions. You can add it to your .bashrc file to avoid needing to repeat the process each time. Specifcally, 
```
echo 'export ROS_DOMAIN_ID=<TEAM_NUMBER>' >> ~/.bashrc 
```

## Use Vicon motion capture for Pose Estimation

Now the two devices are ready to talk to each other! To test the comms, we can run the following commands on Jetson:

The general procedure, when using Vicon  is as follows:
1. The pose of the quadrotor will be published on a ROS2 topic `/vicon/ROB498_Drone/ROB498_Drone`.

2. In a terminal, launch MAVROS by running `ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600`. Alternatively, a launch file with the necessary parameters is provided here: [link](../../resources/code/ros2_ws/src/px4_autonomy_modules/launch/mavros.launch.py).

3. You can use [mavros](https://github.com/mavlink/mavros) package to *redirect* the pose data from Vicon to the flight controller. An option is to write a ROS node that subscribes to the pose data from the motion capture system and republishes it on the topic `/mavros/vision_pose/pose` (the topic might have a different name depending on your setup).

<!-- - MAVROS by default publishes information at a very low rate, or not publishes them at all. We can enable data streams, and set the publish rate by running `rosservice call /mavros/set_message_interval TOPIC_ID DESIRED_RATE` in the third terminal. Topic IDs can be found [here](https://mavlink.io/en/messages/common.html). For example, if we want to publish odometry (pose) at 100 Hz then we can run `rosservice call /mavros/set_message_interval 331 100` where 331 is the ID for odometry.  -->

**IMPORTANT:** The pose reported by the motion capture system depends on the markers mounted on the quadrotor. It is IMPERATIVE that the motion capture estimates be **aligned** with quadrotor body frame: If the quadrotor is manually moved forward, the pose reported by motion capture system should change accordingly (`translation.x` field of `/vicon/ROB498_Drone/ROB498_Drone` should increase). Similarly roll, pitch, and yaw angle changes need to be verified by manually moving the quadrotor.

## Use Realsense T265 Tracking Camera For Pose Estimation

In the absence of GPS, the Cube uses the internal IMU to estimate its pose, which will drift over time. Fortunately, the Realsense T265 camera can use both the IMU and viusal feature to provide more stable pose estimations. To use the Realsense VIO output
1. Launch the realsense camera driver:
```
ros2 launch realsense2_camera rs_launch.py
```

2. If the camera is working as expected (the console log should have the line `Realsense Node is up!`), there should be ROS2 topic with odometry output, which can be confirmed using
```
ros2 topic echo /camera/pose/sample
```

Follow a procedure similar to the motion capture setup to redirect the VIO output to `\mavros\vision_pose\pose`.

**IMPORTANT:** The reference frame for VIO output depends on several factors including (i) where the system (specifically the camera driver) was powered on, and more importantly (ii) the orientation of camera relative to the quadrotor body frame. It is IMPERATIVE that the VIO output be **aligned** with quadrotor body frame: If the quadrotor is manually moved forward, the VIO output should change accordingly (`pose.position.x` field of `\camera\pose\sample` should increase). Similarly roll, pitch, and yaw angle changes need to be verified by manually moving the quadrotor. If the VIO estimates do not align with its motion, then the quadrotor cannot maintain its position or attitude and can lead to unstable flight. 

<!-- The Auterion VIO package is installed at `~/thirdparty/vio_ws`. Source this ROS workspace and run `roslaunch px4_realsense_bridge bridge_mavros.launch` to launch the bridge. -->


# Wifi

If the Wifi adapter is not working, this section provides instruction on how to setup the wireless (wifi) interfact on your Jetson Nano.

- Boot your Jetson Nano and connect it to Ethernet. The TP-Link AC1300 dongle should *NOT* be plugged into any of the USB ports.
- Install *git* on your Jetson: `sudo apt install git`
- Clone the driver to your Jetson: `git clone https://github.com/RinCat/RTL88x2BU-Linux-Driver.git`
- Build and install the driver: `cd RTL88x2BU-Linux-Driver && make ARCH=arm64 && sudo make install`
- Reboot your Jetson
- Plug in the TP-Link dongle and your wifi connection should work (i.e., you should be able to choose a network to connect to)

**Alternatively**, you can download the driver repository onto a USB stick from another device and transfer it to the Jetson Nano. After transferring the repository, follow the build instructions as normal.

<!-- # Running Ubuntu and ROS in a Docker Container

NVIDIA currently supports Ubuntu 18.04 on the Jetson Nano. The compatible ROS version is Melodic Morenia. Both the OS and the ROS version are somewhat dated (as of 2023).

Thankfully, EngSci Robo Jonathan Spraggett has come to the rescue by providing an updated Docker container (image) that is set up to run Ubuntu 20.04 and ROS Noetic Ninjemys! The container supports access to the Jetson CUDA cores, too! -->

