# Intel RealSense T265 Tracking Camera Setup

The Intel RealSense T265 camera incorporates two fisheye lenses and image sensors, an IMU, and an Intel Movidius Myriad 2 VPU. Visual simultaneous localization and mapping (VSLAM) runs directly on the device, enabling real-time tracking in large-scale environments. The datasheet for the camera is available [here](https://www.intelrealsense.com/wp-content/uploads/2019/09/Intel_RealSense_Tracking_Camera_Datasheet_Rev004_release.pdf).

## Hardware Installation

The T265 is a high-bandwidth device that must be conntected to a USB 3.0 port. All USB ports on the 4 GB Jetson Nano support the 3.0 specification - simply plug in the blue USB cable (supplied) to get started.

## Software Installation

The T265 requires the ``librealsense`` SDK (provided by Intel) to operate. You can install the SDK by following the instructions provided here:

https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation

At some point, you may also wish to build the SDK library from source. We recommend building from source with the RSUSB backend to avoid patching the Linux kernel (but the choice is up to you).

## Verifying Camera Operation

With the SKD successfully installed, you can verify the operation of the camera by running the command:

```shell
$ realsense-viewer &
```

at a shell prompt. This will open the GUI viewer application, which also provides information about the current camera settings. You may also want to watch [this](https://www.youtube.com/watch?v=EeT-pzM8n-o) YouTube video for more information.