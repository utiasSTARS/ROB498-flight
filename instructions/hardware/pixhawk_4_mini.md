# Pixhawk 4 Mini

The Pixhawk 4 Mini is a flight controller developed by a collaboration involving Holybro and Auterion. It is configured with the open source PX4 firmware using the QGroundControl program. More information about the Pixhawk 4 Mini can be found here:

https://docs.px4.io/main/en/flight_controller/pixhawk4_mini.html

## Drone Setup with QGroundControl

Multiple Pixhawk internal sensors and external modules/connections require configuration to function properly. These actions can be carried out using the QGroundControl software on a separate setup machine. QGroundControl can be installed ontono Windows, MacOS, and Ubuntu systems - we recommends running it on MacOS or Ubuntu since the Windows version has some startup and video rendering issues. 

See the following link for installation instructions:

https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

Once QGroundControl is installed and running, you can connect the Pixhawk to your setup computer using a USB->microUSB cable (the microUSB port is located on the side of the Pixhawk next to the "Debug" port). The software should automatically detect it. By clicking the QGroundControl logo in the top left corner, you can enter either the **Vehicle Setup** or **Analyze Tools** screens, which will be relevant for later sections.

If you navigate the the **Vehicle Setup** section, you will see a summary of Pixhawk modules that must be configured before use.

### Firmware

Navigate to **Vehicle Setup/Firmware**. Unplug the Pixhawk and wait for the prompt to reconnect. Once connected, select the **PX4 Pro Stable Release** flight stack from the righthand menu and wait for it to install.

### Sensors

Navigate to **Vehicle Setup/Sensors**. Follow the instructions for calibrating the **Compass**, **Gyroscope**, **Accelerometer**, and **Level Horizon** modules by clicking on each tab in order.

### Flight Modes

Navigate to **Vehicle Setup/Flight Modes**. After completing the radio controller setup, you can bind specific hardware switches on the handheld RC (denoted as "channels" in QGroundControl) to specific actions. While you will be flying your drone in this course using commands sent from the Jetson Nano, it is useful for safety purposes to enable flight using the RC.

We recommend that you bind **Arm Switch Channel** (which readies the motors for use), **Emergency Kill Switch Channel** (which cuts power to the motors), and **Offboard Switch Channel** (which toggles control from the Jetson) to easily-accessible switches. 

You should also use a separate channel for the **Flight Mode Settings** to easily enter modes like **Manual** and **Stabilized**. You may also wish to control entry to **Offboard** mode from here.

### TELEM1 (Jetson Nano)

The Pixhawk's TELEM1 port is a dedicated port for communication with a ground station unit via a telemetry radio or an onboard computer. For use with the Jetson Nano, the TELEM1 port is wired up to the GPIO pins on the Jetson through a UART-based serial connection (see **Wiring and Connections** above).

The Jetson GPIO pinout configuration can be found here:

https://www.jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/

Pins **6**, **8**, and **10** in the Jetson Header board are used for the UART connection to the Pixhawk. Note that for UART connections, RX -> TX and TX -> RX from the Jetson to the Pixhawk. Pin **6** may be labeled as **GND** on the physical Jetson board, but it is in the space specified by the above link and should be wired to the GND pin of the TELEM1 port.

Note the address for these UART pins on the Jetson header board is listed as ``/dev/ttyTHS1``, which will be useful when setting up a MAVROS based connection for these devices.

## Other Parameters

The following sections mainly concern parameters that can be found in the **Parameters** section of QGroundControl (**Vehicle Setup/Parameters**). You can use the search bar at the top to quickly find these parameters and keep track of what has changed using the "Show modified only" search option. Modified parameters will be highlighted in red and many will have been set by the actions performed in the **Drone Setup with QGroundControl** section of this guide. Changing some parameters will require a reboot of the Pixhawk, which can be accomplished by disconnecting it from power and from the setup computer. 

### Onboard Jetson Communication

Communication with the Jetson Nano through the GPIO pins must be configured by changing a few parameters. First, you must configure the Pixhawk to send and receive MAVLink messages at an acceptable rate. This can be done by setting the following parameters:

- MAV_0_RATE        -> 921600 B/s
- SER_TEL1_BAUD     -> 921600 8N1

Even with this increased rate, a MAVROS node will only publish messages at 0.5 Hz by default. To fix this, you must configure the Pixhawk to recognise the Jetson as an onboard computer rather than a radio-based ground control unit. The Pixhawk will then automatically publish MAVROS messages at a faster rate. Set the following parameter as such:

- MAV_0_MODE        -> Onboard

### Offboard Position Estimation

Because the Pixhawk is normally configured to receive position information from a GPS module and this course is conducted indoors, you need to supply a position estimate from an external source. This can be done by passing odometry messages through MAVROS on the topic `/mavros/vision_pose/pose`.

The Pixhawk fuses position information through an EKF2 module, which has many configuration parameters. To set up the Pixhawk to fuse this external information, you need to set the following parameter:

- EKF2_AID_MASK     -> 286

This can also be done by enabling the **vision position fusion**, **vision yaw fusion**, and **vision velocity fusion** subparameters and disabling the **use GPS** and **GPS yaw fusion** subparameters in the EKF2_AID_MASK. 

To enable loopback of odometry measurements over MAVLink, set the following parameter:

- MAV_ODOM_LP       -> 1

### TeraRanger Evo Configuration

The Pixhawk is designed to recognize rangefinder devices like the TeraRanger Evo 60m. However, because of the wide range of devices that may be connected through the UART&I2C B port, you must specify that the Evo as the connected device. This can be done by setting the following parameter:

- SENS_EN_TRANGER   -> TREvo60m
  
## More Information

More information about the Pixhawk is avaiable in the official Getting Started guide:

https://docs.px4.io/main/en/getting_started/