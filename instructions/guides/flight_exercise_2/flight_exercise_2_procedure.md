**Preparation for Flight Exercise #2**

1) In order to obtain Vicon pose measurements and communicate with our ground control system (air traffic control), your drone must be connected to the course WiFi during flight. Please refer to the [Jetson instructions](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/hardware/jetson_nano.md) to set up wireless connectivity.

2) We will use ROS 2 as the communication middleware between your drone and our ground control server. Specifically, you must have the following ROS services running on your Jetson:
- `/comm/abort`: At any time during the flight, your drone could receive this command indicating that the test should be terminated immediately for safety reasons. For Flight Exercise #2, you may manually trigger the kill switch in case of emergency.
- `/comm/land`: At the end of your flight, your drone will receive this command and descend for a soft landing. For Flight Exercise #2, you may opt to land the drone manually.
- `/comm/launch`: at the beginning of your flight, your drone will receive this command and ascend to the desired altitude (1.5 m in Flight Exercise #2). For Flight Exercise #2, you may opt to take off manually.
- `/comm/test`: Once your drone has stabilized at the desired starting position and altitude, the TA will send the command to the drone to indicate the start of the scoring segment. For Flight Exercise #2, the drone just needs to hover at the starting pose and no additional action is required.
3) All services will accept a `std_srvs::TriggerRequest` and return a `std_srvs::TriggerResponse`. We require all four services to be running for the challenge. A [skeleton python node](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/guides/flight_exercise_2/templates/comm_node_skeleton.py) is provided for your reference. Note that you should not put any loops in the callback functions. The callbacks should just set some global flags and your main loop in the node is responsible for checking the state of the flags and execute the flight commands.

###### Hints

Example order of operations for Task 2 is as follows:

1. Pose of the quadrotor will be made available via the motion capture system or through onboard Realsense camera. As a first step, you should *redirect* this pose data to MAVROS topic `/mavros/vision_pose/pose`. 
2. Confirm your flight controller is getting this data. This can be done using QGroundControl by clicking on top left icon and following `Analyze Tools->Mavlink Inspector` and looking at `LOCAL_POSITION_NED` or `ODOMETRY`. NOTE: All calculations on the flight controller are done in NED (North-East-Down) frame, whereas pose reported by the motion capture system or the Realsense are in ENU (East-North-Up) frame. Conversion from NED to ENU is done internally by MAVROS. However, if you mount the realsense camera in a different orientation, please confirm that the data reported by realsense and seen by QGroundControl match (taking into account the frame conventions). More information on frames can be found here: https://docs.px4.io/main/en/ros/external_position_estimation.html#reference-frames
3. Once you are confident that data input pipeline is working as intended, you can proceed to waypoint publication. For this part, you will publish waypoints to `/mavors/setpoint_position/local`. Be cognizant of the `frame_id` field in each of the ROS messages that are published.
4. Note that the flight controller expects a steady stream of setpoint messages before you can switch to offboard mode. For instance, if you publish a single waypoint only once the flight controller won't transition or allow switching mode into `OFFBOARD` mode. ROS timers can be used to publish waypoints at a steady rate (typically 20 Hz). You could switch into offboard mode when the TA computer issues a `launch` command. Your code will recieve a service request for launch if you follow the guidelines and template in [](./templates/comm_node_skeleton.py).

5. Once the pose data **and** the setpoints are being sent to the flight controller via MAVROS, you will be able to transition to OFFBOARD mode by flippting the switch on your RC transmitter. You can enter OFFBOARD mode programmatically as shown here [offboard example](https://docs.px4.io/main/en/ros/mavros_offboard_python.html).

CAUTION: OFFBOARD mode can be dangerous as the autopilot is under the control of your script. Exercise extreme caution when operating in this mode. Practice taking control by switching into STABILIZED flight mode from OFFBOARD mode to recover your quadrotor in case things go wrong. 


**Flight Exercise #2 Test Procedure**

1. Make sure your drone is connected to the course WiFi network and you are able to `ping` the ground control server at 10.42.0.100 and receive Vicon pose messages.
2. The TA will send test service calls and check if your drone is able to receive the commands successfully.
3. Once ready, the TA will instruct you to arm the drone and then send the “launch” command. Your drone may take off autonomously or under manual control.
4. When you think your drone is ready for scoring, let the TA know and they will start collecting data. A “test” command will be sent to the drone. Vicon pose streaming will be stopped for the Vicon-free test.
5. Data collection will end after 30 seconds. The TA will instruct you to land the drone. Your drone may land autonomously or manually.
6. Flight data within the 30-second window will be saved and our scoring system will determine your score. You will receive the result (plots and numerical score) in a PDF file. A [sample result](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/guides/flight_exercise_2/sample_results/sample_result_stationary_drone.pdf) for a stationary drone is provided for your reference.