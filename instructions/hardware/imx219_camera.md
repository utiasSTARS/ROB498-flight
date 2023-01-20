# IMX219 Monocular Camera Setup

The IMX219 is a diagonal 4.60 mm (Type 1/4.0) CMOS active-pixel image sensor with a square pixel array and 8.08 million effective pixels. This camera is popular in the Jetson and Arduino developer communities; the Jetson Nano has a special port and connector (the camera serial interface or CSI) for IMX219 cameras (or others). The datasheet for the camera is available [here](xxxx).

## Hardware Installation



## Software Installation




The T265 requires the ``librealsense`` SDK (provided by Intel) to operate. You can install the SDK by following the instructions provided here:

https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation

At some point, you also may wish to build the SDK library from source. We recommend building from source with the RSUSB backend to avoid patching the Linux kernel (but the choice is up to you).

## Verifying Camera Operation

With the SKD successfully installed, you can verify the operation of the camera by running the command:

```shell
$ realsense-viewer &
```

at a shell prompt. This will open the GUI viewer application, which also provides information about the current camera settings. You may also want to watch [this](https://www.youtube.com/watch?v=EeT-pzM8n-o) YouTube video for more information.