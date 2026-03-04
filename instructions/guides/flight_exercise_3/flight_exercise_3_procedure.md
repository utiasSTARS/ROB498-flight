**Preparation for Flight Exercise #3**

1. Flight Excercise #3 is very similar to Flight Exercise #2. There is an additional ROS subscriber that must be running on your Jetson:
- `/comm/waypoints`: This subscriber receives the set of waypoints as a PoseArray object.
2. All the other ROS services described for Flight Exercise #2 must also be running.
   
A [skeleton Python node](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/guides/flight_exercise_3/templates/comm_node_skeleton.py) is provided for your reference. Note that you should not put any loops in the callback functions. The callbacks should just set some global variables; your main loop in the node is responsible for checking the state of the variables/flags and executing the flight commands.

**Flight Exercise #3 Test Procedure**

1. Make sure your drone is connected to the course WiFi network and you are able to `ping` the ground control server at 10.42.0.100 and receive Vicon pose messages.
2. The TA will send test service calls and check if your drone is able to receive the commands successfully.
3. The TA will send the waypoints to your subscriber--the waypoints are sent *before* the other commands to allow you to carry out preprocessing (e.g., planning) if desired.
4. The TA will instruct you to arm the drone and then send the “launch” command. Your drone may take off autonomously or under manual control.
5. When you think your drone is ready for scoring, let the TA know and they will start collecting data. A “test” command will be sent to the drone. Vicon pose streaming will be stopped for the Vicon-free test.
6. Data collection will end after 90 seconds. The TA will instruct you to land the drone. Your drone may land autonomously or manually.
7. Flight data within the 90-second window will be saved and our scoring system will determine your score. You will receive the result (plots and numerical score) in a PDF file.