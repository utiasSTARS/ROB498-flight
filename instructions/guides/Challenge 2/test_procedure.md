**This document outlines the evaluation procedure of Challenge 2**

**Preparation**

1. In order to obtain the Vicon pose measurements and communicate with our ground control system, your drone must be connected to the course wifi during flight. Please refer to the [Github instructions](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/hardware/jetson_nano.md) to set up wireless connectivity.
2. We will use ROS 1 as the communication middleware between your drone and our ground control server. Specifically, you must have the following ROS services running on your Jetson:
- /comm/abort: at any time during the flight, your drone could receive this command indicating the test should be terminated for safety reasons. For Challenge 2, you may manually trigger the kill switch in case of emergency.
- /comm/land: at the end of your flight, your drone will receive this command and descend for a soft landing. For Challenge 2, you may opt to land the drone manually.
- /comm/launch: at the beginning of your flight, your drone will receive this command and ascend to the desired altitude (1.5m for Challenge 2). For Challenge 2, you may opt to take off manually.
- /comm/test: once your drone has stabilized at the desired starting position and altitude, the TA will send the command to the drone to indicate the start of the scoring segment. For Challenge 2, the drone just needs to hover at the starting pose and no additional action is required.

3. All services will accept a `std_srvs::EmptyRequest` and return a `std_srvs::EmptyResponse`. We require all four services to be running for the challenge. A [skeleton python node]() is provided for your reference.

**Challenge 2 Testing Procedure**
1. Make sure your drone is connected to the course wifi and you are able to `ping` the ground control server at 10.42.0.100
2. The TA will send test service calls and check if your drone is able to receive the commands
3. Once ready, the TA will instruct you to arm the drone as well as send the “launch” command to your drone. Your drone may take off autonomously or manually
4. When you think your drone is ready for scoring, let the TA know and we will start collecting data. A “test” command will be sent to the drone
5. Data collection will end after 30 seconds. The TA will instruct you to land the drone. Your drone may land autonomously or manually
6. Flight data within the 30-second window will be saved and our scoring system will determine your score. You will receive the results in a PDF file.

