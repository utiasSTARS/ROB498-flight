**This document outlines the evaluation procedure of Challenge 3**

**Preparation**

1. In order to obtain the Vicon pose measurements and communicate with our ground control system, your drone must be connected to the course wifi during flight. Please refer to the [Github instructions](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/hardware/jetson_nano.md) to set up wireless connectivity.
2. We will use ROS 1 as the communication middleware between your drone and our ground control server. Specifically, you must have the following ROS services running on your Jetson:
- `/rob498_drone_ID/comm/abort`: at any time during the flight, your drone could receive this command indicating the test should be terminated for safety reasons. For Challenge 3, you may manually trigger the kill switch in case of emergency.
- `/rob498_drone_ID/comm/land`: at the end of your flight, your drone will receive this command and descend for a soft landing. For Challenge 3, you may opt to land the drone manually.
- `/rob498_drone_ID/comm/launch`: at the beginning of your flight, your drone will receive this command and ascend to a starting position. For Challenge 3, you may opt to take off manually.
- `/rob498_drone_ID/comm/test`: once your drone has stabilized, the TA will send the command to the drone to indicate the start of the scoring segment. The waypoints will also be published at `/rob498_drone_ID/comm/waypoints` as `PoseArray` messages. Upon receiving the waypoints, the drone should visit each waypoint in order within the 120s horizon.

3. All services will accept a `std_srvs::EmptyRequest` and return a `std_srvs::EmptyResponse`. We require all four services to be running for the challenge. An updated [python node](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/guides/Challenge%202/Scripts/comm_node_skeleton.py) is provided for your reference. Note that you should not put any loops in the callback functions. The callbacks should just set some global flags and your main loop in the node is responsible for checking the state of the flags and execute the flight commands.

**Challenge 3 Testing Procedure**
1. Make sure your drone is connected to the course wifi and you are able to `ping` the ground control server at 10.42.0.100 and receive the Vicon poses
2. The TA will send test service calls and check if your drone is able to receive the commands
3. Once ready, the TA will instruct you to arm the drone as well as send the “launch” command to your drone. Your drone may take off autonomously or manually
4. When you think your drone is ready for scoring, let the TA know and we will start collecting data. A “test” command as well as the waypoints will be sent to the drone. Vicon pose streaming will be stopped for the Vicon-free test.
5. Data collection will run for 120 seconds max. You could ask the TA to stop once your drone has visited all the waypoints. Your drone may land autonomously or manually
6. Flight data will be saved and our scoring system will determine your score. You will receive the result (plots and numerical score) in a PDF file. A [sample result](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/guides/Challenge%202/Sample_Results/sample_result_stationary_drone.pdf) is provided for your reference.

