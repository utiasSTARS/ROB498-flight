# Terabee TeraRanger Evo 60m

The *TeraRanger Evo 60m* is an infrared Time-of-Flight range sensor with a detection range of up to 60 meters. It comes with a main black module and 2 yellow backboards which provide the sensor with a communication and power link.

## Hardware Setup

To connect the Evo to your drone you can use either of the two communication backboards, which are configured with either a USB 2.0 cord or UART/I2C ribbon cable. This setup (along with the following sections and separate starter code files) uses the UART/I2C cable for connection to the *Pixhawk 4 Mini* as it provides an easy way of integrating measurements over ROS.

Because of specifications in the *Pixhawk 4 Mini*'s hardware, the I2C protocol must be followed when wiring up a connection to the Evo (SDA -> SDA, SCL -> SCL, GND -> GND, and VCC -> 5V from the Pixhawk to the Evo). Once the correct wires are connected, the Evo can be plugged directly into the **UART&I2C B** port of the Pixhawk. For help with wiring, see the following:

> https://www.generationrobots.com/media/User-Manual-for-TeraRanger-Evo-single-point-distance-sensors-and-backboards-1-2.pdf

> https://github.com/PX4/px4_user_guide/raw/master/assets/flight_controller/pixhawk4mini/pixhawk4mini_pinouts.pdf

## Software Setup

It should be first noted that the *Pixhawk 4 Mini* requires the correct driver to work with the Evo and communicate properly. This driver is usually present in the default PX4 firmware, however you may also need to add the driver (``distance_sensor/teraranger``) to the board configuration on your own (very unlikely). You will know if the driver is missing if the following Pixhawk parameters/shell commands do not exist for you. See the following link for more information:

> https://docs.px4.io/master/en/sensor/teraranger.html

In order for the Evo's measurements to be readable over Pixhawk (and eventually ROS), a short setup with Pixhawk is required. In QGroundControl, first change the parameter ``SENS_EN_TRANGER`` to **TREvo60m**, which tells the Pixhawk which sensor in the TeraRanger family is connected to the I2C port. Next, navigate to the **MAVLink Console** in **Analyze Tools/MAVLink Console** and enter the following command:

```shell
> teraranger start -X
```

which tells the Pixhawk to connect to the Evo over an external bus. You should only have to enter this command once every time the Pixhawk is powered on unless you disconnect the Evo.

To ensure proper communication, you can navigate to the **MAVLink Inspector** in **Analyze Tools/MAVLink Inspector** and read data through the ``DISTANCE_SENSOR`` topic. This can only be done after the previous commands have been executed.

## Usage Instructions

To recieve Evo measurements over ROS, you must first have the ``mavros`` and ``mavros_extras`` packages installed (see ``ROB498-flight/docs/mavros.md``). ``mavros_extras`` already contains the correct plugin that reads MAVLink distance sensor measurements, but you need to enable this plugin by commenting out or deleting the "- rangefinder" and "- distance_sensor" lines in ``mavros/mavros/launch/px4_pluginlists.yaml`` file under the **plugin_blacklist** heading. 

If this is done, then during execution of the ``mavros`` ROS node, you can view Evo measurements over the ``mavros/distance_sensor/hrlv_ez4_pub`` topic. Configuration of this topic (including configuring the position and orientation of the Evo with respect to the drone's base frame) can be accessed in ``mavros/mavros/launch/px4_config.yaml`` under the **distance sensor/hrlv_ez4_pub** heading.

### Height Estimation

Finally, it should be noted that Evo range measurements can be incorporated directly into the *Pixhawk 4 Mini*'s EKF2 module for position (height) estimation. THIS SHOULD ONLY BE DONE DURING OPERATION OVER FLAT SURFACES, AS CHANGES IN FLOOR HEIGHT/TEXTURE CAN CAUSE UNPREDICTABLE MOVEMENT.

To enable this function, you must open QGroundControl and change the ``EKF2_HGT_MODE`` parameter to "Range sensor", and configure multiple ``EKF2_RNG_ ...`` parameters for position/orientation of the Evo with respect to the Pixhawk, velocity sensitivity, etc. 