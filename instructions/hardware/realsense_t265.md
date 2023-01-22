# Intel RealSense T265 Tracking Camera Setup

The Intel RealSense T265 camera incorporates two fisheye lenses and image sensors, an IMU, and an Intel Movidius Myriad 2 VPU. Visual simultaneous localization and mapping (VSLAM) runs directly on the device, enabling real-time tracking in large-scale environments. The datasheet for the camera is available [here](https://www.intelrealsense.com/wp-content/uploads/2019/09/Intel_RealSense_Tracking_Camera_Datasheet_Rev004_release.pdf).

## Hardware Installation

The T265 is a high-bandwidth device that must be conntected to a USB 3.0 port. All USB ports on the 4 GB Jetson Nano support the 3.0 specification - simply plug in the blue USB cable (supplied) to get started.

## Software Installation

There are two methods for installing the ROS1 wrapper used for Intel Realsense devices like the T265. Both are detailed in the following installation guide:

https://github.com/IntelRealSense/realsense-ros

### Method 1: The ROS Distribution

This method is the simplest and most direct way to get the ROS wrapper installed, however it offers limited interaction with the camera node.

#### Verifying Camera Operation

After successfully installing the correct packages, you can verify operation of the camera by running the following command: 

```shell
$ roslaunch realsense2_camera demo_t265.launch
```

at a shell prompt. This will start a ROS node and open the RViz application.

### Method 2: The Realsense Distribution

Ths method allows you to operate the T265 using the ``librealsense`` SDK (provided by Intel). You can install the SDK by following the instructions provided here:

https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation

This method is more suitable if you wish to use the T265 with programs outside of ROS or if you want to more easily modify launch parameters for the camera node.

At some point, you may also wish to build the SDK library from source. We recommend building from source with the RSUSB backend to avoid patching the Linux kernel (but the choice is up to you).

#### Verifying Camera Operation

With the SDK successfully installed, you can verify operation of the camera by running the command:

```shell
$ realsense-viewer &
```

at a shell prompt. This will open the GUI viewer application, which also provides information about the current camera settings.

## ROS Usage Instructions

The T265 is primarily used for realtime VSLAM, which make it perfect for tracking the movements of your drone. If coupled to ROS, you will need to initialize the separate camera node at the start of every flight with the following:

```shell
$ roslaunch realsense2_camera rs_t265.launch
```

You can check what data is available in another terminal using 

```shell
$ rostopic list /camera
```

If you want to access the raw images from either T265, you must enable their output from the ``rs_t265.launch`` launch file. You can enable the raw camera outputs by changing the ``enable_fisheye1`` and ``enable_fisheye2`` parameters.