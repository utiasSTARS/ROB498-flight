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