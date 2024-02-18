**Preparation for Flight Exercise #2**

1) In order to obtain Vicon pose measurements and communicate with our ground control system (air traffic control), your drone must be connected to the course WiFi during flight. Please refer to the [Jetson instructions](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/hardware/jetson_nano.md) to set up wireless connectivity.

2) We will use ROS 1 as the communication middleware between your drone and our ground control server. Specifically, you must have the following ROS services running on your Jetson:
- `/comm/abort`: At any time during the flight, your drone could receive this command indicating that the test should be terminated immediately for safety reasons. For Flight Exercise #2, you may manually trigger the kill switch in case of emergency.
- `/comm/land`: At the end of your flight, your drone will receive this command and descend for a soft landing. For Flight Exercise #2, you may opt to land the drone manually.
- `/comm/launch`: at the beginning of your flight, your drone will receive this command and ascend to the desired altitude (1.5 m in Flight Exercise #2). For Flight Exercise #2, you may opt to take off manually.
- `/comm/test`: Once your drone has stabilized at the desired starting position and altitude, the TA will send the command to the drone to indicate the start of the scoring segment. For Flight Exercise #2, the drone just needs to hover at the starting pose and no additional action is required.
3) All services will accept a `std_srvs::EmptyRequest` and return a `std_srvs::EmptyResponse`. We require all four services to be running for the challenge. A [skeleton python node](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/guides/flight_exercise_2/templates/comm_node_skeleton.py) is provided for your reference. Note that you should not put any loops in the callback functions. The callbacks should just set some global flags and your main loop in the node is responsible for checking the state of the flags and execute the flight commands.

**Flight Exercise #2 Test Procedure**

1. Make sure your drone is connected to the course WiFi network and you are able to `ping` the ground control server at 10.42.0.100 and receive Vicon pose messages.
2. The TA will send test service calls and check if your drone is able to receive the commands successfully.
3. Once ready, the TA will instruct you to arm the drone and then send the “launch” command. Your drone may take off autonomously or under manual control.
4. When you think your drone is ready for scoring, let the TA know and they will start collecting data. A “test” command will be sent to the drone. Vicon pose streaming will be stopped for the Vicon-free test.
5. Data collection will end after 30 seconds. The TA will instruct you to land the drone. Your drone may land autonomously or manually.
6. Flight data within the 30-second window will be saved and our scoring system will determine your score. You will receive the result (plots and numerical score) in a PDF file. A [sample result](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/guides/flight_exercise_2/sample_results/sample_result_stationary_drone.pdf) for a stationary drone is provided for your reference.