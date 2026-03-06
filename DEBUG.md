# Debugging Tips

Throughout the semester, you will certainly run into many issues where things just aren't working. This can be extremely frustrating and instill some hopelessness in the course. Not to worry though! Many of the issues you will see have been seen before! This document contains a list of the more common issues that may be run into throughout the year, lists probably causes and solutions, and points to tools that may help.

## Useful Tools

* [RVIZ](https://github.com/ros2/rviz): a visualization toolkit for ROS topics. Use this to confirm things like camera data being published, frame transforms, state estimates, etc.
* [RQt](https://docs.ros.org/en/humble/Concepts/Intermediate/About-RQt.html): a series of tools that let you analyze all things ROS. Use this to see node graphs, TF trees, topic subscribers, etc.
* [px4 Flight Review](https://review.px4.io/): A tool for visualizing OrangeCube flight logs. This is super useful for analyzing a flight that had weird behaviours. It shows your pose estimates, raw sensor data, any error flags / warning messages, vibrations, etc.

## Common Issues

| Category | Issue | Potential Causes & Solutions |
| :--- | :--- | :--- |
| **Battery** | Our battery is puffy | This comes from overuse. It is **extremely important** to not overfly your drone. Ensure that you have you battery management system setup (see instructions [here](instructions/hardware/orange_cube_plus.md#battery-management)) |
| **Data connectivity** | Cannot detect RGB camera | Likely, your Jetson has not enabled the CSI camera port. Follow the instructions [here](https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3275/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/hw_setup_jetson_io.html#wwpID0E02D0HA). |
| **Data connectivity** | Cannot see the Vicon topic | - Ensure the topic is being published (confirm with your TA)<br>- Check to make sure your `ROS_DOMAIN_ID` variable is set to your team number<br>- Check to make sure you can ping the TA machine (ask TA for IP address) |
| **State estimation** | "toilet-bowling" or rapid ascent | This behaviour, characterized by either lifting off and then the drone beginning to fly in a growing circle, or, rapidly flying too high, is almost always caused by a frame misalignment issue. In other words, Mavros thinks your camera is mounted in one orientation, and it is actually mounted in another.<br><br>To debug this, run your Mavros localization without the camera, and move your drone around. Get a feel for what your orange cube's coordinate frame is. Next, run only your camera node (**not with the orange cube**) and do the same. If these frames are not the same, you have to redefine the transform between the camera and the orange cube. Both translation and rotation matter here, be thorough. |
| **State estimation** | Large drifts | When this happens, it typically looks like your position estimate shooting off to large numbers (e.g. -80 in the z-direction). This is likely caused by vibration issues. The T265 cameras are extremely sensitive to vibration. To confirm this is the issue, test the localization node on a test bench without flying. Move your drone around. If the estimate looks good here, but breaks when flying, it's almost certainly a vibration issue.<br><br>Primary causes of this vibration are:<br>- Damage propellers<br>- Loose components on the drone (nuts, bolts, standoffs, cables, motors, etc.) |
| **State estimation** | Bias in the state estimate | This issue, typically discover during waypoint following, shows the drone missing the waypoints but thinking it hit them. This is often caused by a frame translation misalignment. The course testing code only cares about the position of the Vicon marker, which is **not** the same as your state estimate. This offset cannot be ignored. |

## Post-Crash steps

A bad crash can induce many issues with your drone. Here is a list of typical issues and things to check post-crash:

* Anything physically broken. Most often 3D printed parts and standoffs. These are **meant to break** as they act as a last-resort impact barrier before your sensors start taking impact. Simply replace the broken parts.
* A bad impact can cause damage to your Realsense. This most often manifests as an increased sensitivity to vibration. Test the camera-only localization on the test bench with manual drone movement, ensure it works. You may notice large drifts in estimates after flying that you weren't seeing before. See the "Large drifts" issue in the previous section.
* Similarly, bad crashes can cause OrangeCube damage. Follow a similar test-bench procedure as the previous step, this time using QCG to confirm the IMU is behaving as expected.
* Check for propeller damage. Even small breaks and bends in a propeller can introduce large vibrations and instabilities during flight.
