# How to Install ROS2 Humble on a Jetson Nano

## Prerequisites

Follow these [instructions](./setup-ubuntu2004/setup_ubuntu_2004.md) to install **Ubuntu 20.04** on your Jetson Nanos.

Follow these [instructions](../instructions\hardware\jetson_nano.md) to set up your Jetson Nano and **the TP-Link Wifi dongle**.

## Considerations

The vicon system is currently configured to publish over ROS1. If you want to run your drone in ROS2, you would either need to reconfigure the vicon for ROS2 (some libraries exist), or run the [ROS bridge](https://github.com/ros2/ros1_bridge).

The instructions for setting up MAVROS in the ROB498 flight repo are for ROS1. MAVROS does [support ROS2](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md), but the ROB498 teaching team has not tested it, so we do not have set up instructions to provide.

PX4 provides two communication protocols to communicate with the flight controller: MAVROS and microDDS.

* The ROS2 Foxy version of MAVROS had a significant computational overhead. The overhead for ROS2 Humble is untested.
* The PX4 team recommends [microDDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html) with ROS2, but there have been some issues with other flight controllers and microDDS communication.

The ROS2 publisher-subscriber communication may need additional configuration when communicating over the TP-Link Wifi Modules, while the ROS1 defaults to `best_effort`.

Based on the above considerations, it is up to you to decide whether to use ROS1 or ROS2 for your project.

## Installing ROS2 Humble

To install ROS2 Humble, you need to install it from source, following the instructions at <https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html>

Note that at step [Install development tools and ROS tools](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools), you need to select the instructions for **Ubuntu 20.04 LTS**, and not Ubuntu 22.04 LTS and Later.

Installing ROS2 can take a long time, so set aside at least 1 hour that you can leave the Jetson powered on and periodically check in on the installation.