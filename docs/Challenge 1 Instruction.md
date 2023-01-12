# Challenge 1: Drone "Run Up" Detailed Instruction

Author: Xuan Wang

As has introduced in the Lecture 2 slides, the basic drone hardware (HW) kits are assigned to each team. The kit will be only prepared with soldering work compeleted,
and the task for the teams is to assemble the drone and pass the "run up" to prove that your aircraft is **air-worthy**. During the "run up" demo,
the teams should follow and pass all the [checklist items provided for the course](https://q.utoronto.ca/courses/299314) and interviewed by TAs, to show that they have **fully understood** all the safety guidelines and pre-flight/postflight procedures.

This manual will help you get started with the brief and core steps of assembling the hardware and configuring the basic software of the drone for this course. PX4 is the recommended open-source firmware for the pixhawk 4 mini autopilot flight controller, ad the **main reference is the [PX4 autopilot official documentation](https://docs.px4.io/main/en/)**, (many other languages are available!). It provides all the information you could find from knowing the basic concepts of drones to the advanced configurations. 

Note that the low-level drone controlling code are not expected to be written by students since the complexity is out-of-scope of undergraduate level. This is why the PX4 open-source firmware has been and is still being activly developed by thousands of outstanding engineers around the world. The work of this course could also make great contributions to this open-source community.

<img src = "img/px4-logo.svg" width = "200">

## Hardware Setup
Within the kit you will find that the power distribution board is already soldered with ESCs and motors, but you should also find out the rest correct avionics connections:
- Make sure the pixhawk 4 is powered properly from PDB
- ESC connections to the pixhawk for motor control
- RC (Remote control) receiver connection to the pixhawk
- TeraRanger EVO 60m connection to the pixhawk

Hint: check the datasheet and pinout documents of the avonic parts. The ESC & motor numbering and rotation direction as recongized by the flight controller should be as following figure:

<img src = "img/QuadRotorX.8e9a5495.svg" width = "200">


## Firmware/Ground Station Setup
In real life, the correct avonic connections cannot ensure that they will work properly at all. You also need to setup the firmware of the pixhawk 4. All the settings and parameters can be modified with the ground control station (GCS) software. It is recommended to use QGroundControl (referred to as **QGC** in the following sections), which supports most OS.

<img src = "img/QGC.png">

First open the QGC, click the logo on the top left to enter the vehicle setup page. Update the firmware to the latest version as instruced on the screen, make sure the airframe is selected as "Generic Quadcopter". Everytime you plug in the pixhawk again using micro USB cable, the QGC should auto detect it and connect.

If you go to vehicle setup after the firmware update, you will see a list of items to do on the left of the screen, please check in sequence and configure accordingly. The key settings are:

- Airframe (Generic Quadcopter)
- Sensors (Calibrate everything for the first setup, or whenever you think necessary)
- Radio (Calibration and confirmation of the remote control)
- Flight modes (to be explained more in detail in the next section)
- Motors testing (Make sure the rotation direction is correct)
- Parameters (Much more flexible settings!)

Try not to modify the PID parameters whcih would cause unstability.

## Radio Setup
As introduced above, radio setup and flight modes need more work and standard setups. It may also take some time for you to learn how to use the Taranis Remote Control Transmitter (will be referred to as "Taranis" in the following parts) since it provides a lot of flexibilities and is more advanced than the toy-level remote controllers!

To setup the radio, you to need first bind it with the RC receiver (the ARCHER R4) following the instructions on its manual (have a hardcopy with the module, you can also always find it online), and check in the "Radio" tab if you can see the reactions. If the receiver is stuck in the reg mode, contact TA. 

The attitude control is generally mapped as "American Hand" as shown in the figure below, if you would like to control in other fasion, please notify the Head TA. Remeber to also do the calibration for the first time setup, and make sure that the channel mappings are correct (for example, you do not want moving throttle on the RC and it appreas to be pitch on the flight controller!). If the mapping is incorrect, you can change the channel setting in Taranis.

<img src = "img/rc_basic.png" width = "500">

## Flight Mode Setup

In the Flight Mode section in QGC, you could assign channels for flight mode switch, arm switch, and **emergency kill switch**. The suggested channel/switch mapping for them is listed in the table below. The flight modes that we will be using in this course is "Stabilized", "Altitude", and "Offboard". They are explained well in the PX4 doc, Flying section. Please read carefully and make sure you understand well on both the behaviour and the sensors involved. 

| Switch Name (Function) | Hardware Switch | Channel |
|------------------------|-----------------|---------|
|Flight Mode Switch      |SB (longest stick on the front panel)| Channel 5 |
|Arm Switch              |SA/SC (shorter sticks on the front panel)| Channel 6 |
|Emergency Kill Switch   |SD (left top side switch, sorry for other-handed person)| Channel 7 |


To be able to switch the flight mode, you need to first setup the 




 
