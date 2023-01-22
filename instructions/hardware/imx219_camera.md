# IMX219 Monocular Colour Camera Setup

The IMX219 is a diagonal 4.60 mm (Type 1/4.0) CMOS active-pixel image sensor with a square pixel array and 8.08 million effective pixels. This camera is popular in the Jetson and Arduino developer communities; the Jetson Nano has a special port and connector (the camera serial interface or CSI) for IMX219 cameras (or others). The datasheet for the camera is available [here](https://www.opensourceinstruments.com/Electronics/Data/IMX219PQ.pdf).

## Hardware Installation

Hardware installation is easy: lift the plastic tabs of the CSI connector that is closest to the barrel power jack (i.e., Camera 0). 

Slide the ribbon cable into the connector fully without tilting. The blue markings should face towards the *outside* of the board, away from the heat sink. The ribbon cable contacts need to face towards the heat sink.

Hold the ribbon cable still while carefully and gently pushing down on the plastic tabs to fasten the ribbon cable in place.

You should be able to pull gently on the camera without popping the cable out of the latch.

## Verifying Camera Operation

NVIDIA has already provided software to work with the camera. To verify that the camera is properly installed and connected, at a shell prompt type:

```shell
$ nvgstcapture-1.0
```

See the `nvgstcapture-1.0` man page for more information on various command options.