# Challenge 1: Drone "Run Up" Detailed Instruction
As has introduced in the Lecture 2 slides, the basic drone hardware (HW) kits are assigned to each team. The kit will be only prepared with soldering work compeleted,
and the task for the teams is to assemble the drone and pass the "run up" to prove that your aircraft is **air-worthy**. During the "run up" demo,
the teams should follow and pass all the checklist items (>https://q.utoronto.ca/courses/299314) and interviewed by TAs, to show that they have **fully understood** all the safety guidelines and pre-flight/postflight procedures.

This manual will help you get started with the brief steps of assembling the hardware and configuring the basic software of the drone for this course. PX4 is the recommended open-source firmware for the pixhawk 4 mini autopilot flight controller: the **main reference is the PX4 autopilot official documentation** (>https://docs.px4.io/main/en/, and many other languages are available!). It provides all the information you could find from knowing the basic concepts of drones to the advanced configurations. 

Note that the low-level drone controlling code are not expected to be written by students since the complexity is out-of-scope of undergraduate level. This is why the PX4 open-source firmware has been and is still being activly developed by thousands of outstanding engineers around the world. The work of this course could also make great contributions to this open-source community.

<img src = "img/px4-logo.svg">

## Hardware Setup
Within the kit you will find that the power distribution board is already soldered with ESCs and motors, but you should also find out the rest correct avionics connections:
- Make sure the pixhawk 4 is powered properly from PDB
- ESC connections to the pixhawk for motor control
- RC (Remote control) receiver connection to the pixhawk
- TeraRanger EVO 60m connection to the pixhawk

## Firmware Setup
In real life, the correct avonic connections cannot ensure that they will work properly at all. You also need to setup the firmware of the pixhawk 4. All the settings and parameters can be modified with the ground control station (GCS) software. It is recommended to use QGroundControl.  


 
