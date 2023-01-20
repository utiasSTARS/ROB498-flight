# Getting Started With Your Jetson Nano Dev Kit

The provided NVIDIA Jetson Nano Dev Kit is a mini computer widely used amongst robotics enthusiasts. In this course, we will use the Jetson Nano as the computing unit for the drone to complete a set of autonomous flying challenges. Before starting, you should take a look at the official [NVIDIA Instruction](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) on how to set up your device.

NVIDIA provides an [SD card image](https://developer.nvidia.com/jetson-nano-sd-card-image) which installs a custom version of Ubuntu 18 on your Jetson. We also provide a modified version of the image that comes with a set of pre-installed packages for easier setup.

To use the pre-configured image, first download the zip file [here](https://drive.google.com/file/d/1c-AUyDF2ZgA6t0d_pnyBmTgDt-NZ41I6/view?usp=share_link) and unzip it on your host Ubuntu machine. Note that you will need at least 64GB of free disk space to store the image file. Next, connect the provided microSD card to your host computer and open the **Disks** app. Select **Restore Disk Image...** in the menu to flash the disk image to the microSD as shown in the figure below.

<img src = "instructions/images/flash_sd.png](https://github.com/utiasSTARS/ROB498-flight/blob/0f973e2960fc59ea7a5dc7dec1d59a7b93ecab13/instructions/images/flash_sd.png)">

# MAVROS

MAVROS is a ROS package which contains a communication node for the MAVLink protocol. It is used for this course as a bridge between the *Jetson Nano*, containing ROS-based drone commands, and the *Pixhawk 4 Mini*, the flight controller for the course drones. 

## Installation

The official MAVROS installation guide can be found here: 

> https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation

You can decide on whether or not to install the package using a binary installation for from source, however it is our recommendation that the MAVROS package be installed from source to your catkin workspace. Though this creates a very long (~45 minute) first build time, this allows the user to more easily modify the mavros launch files if need be. 

## Modifications

You must conduct a few small modifications to the launch and configuration files of the MAVROS node before using it with your drone. 

### Enable Rangefinder Measurements

To allow MAVROS to recieve rangefinder measurements from the Pixhawk, navigate to ``mavros/mavros/launch/px4_pluginlists.yaml`` and comment out or delete the "- rangefinder" and "- distance_sensor" lines under the **plugin_blacklist** heading.

## Usage Instructions

To start ROS-based communication between the *Jetson Nano* and the *Pixhawk 4 Mini*, run the following command:

```shell
$ roslaunch mavros px4.launch fcu_url:/dev/ttyTHS1:921600
```

This starts the mavros node using middleware to connect with the **PX4** firmware running on the Pixhawk. It also specifies the GPIO pins as the targeted communication port on the Jetson (``/dev/ttyTHS1``) and to send messages with a baudrate of 921600 B/s. 
