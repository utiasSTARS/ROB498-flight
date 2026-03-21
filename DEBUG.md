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
| **Time / offboard** | Offboard mode rejected immediately or after a few seconds; MAVROS commands seem ignored | Often **clock skew** between the Jetson and the Cube: the autopilot treats MAVROS messages as **stale** if their timestamps are too far from its own clock. Common when the Cube has accurate time (e.g. via QGroundControl on a laptop with internet) while the Jetson on the lab WiFi has **no working NTP** and drifts. See [Clock synchronization (Jetson vs. Cube)](#clock-synchronization-jetson-vs-cube) below. |

### Clock synchronization (Jetson vs. Cube)

**Lab context:** This has shown up during sessions when teams connect the Cube to QGroundControl on an internet-connected laptop while the Jetson is on a network (e.g. TP-Link / ROB498 WiFi) that **cannot reach default Ubuntu NTP servers**. The failure mode is **non-deterministic** because it depends on **which devices you connect, in what order**, and which clock ends up authoritative.

**What goes wrong**

- **Scenario A — QGC used on an internet-connected laptop:** The Cube’s MAVLink stack can sync to a good time source. The Jetson stays on an isolated WiFi segment without NTP and its clock **drifts**. The Cube then **rejects or ignores** MAVROS traffic whose timestamps look old compared to the autopilot clock. Symptom: **offboard transition fails** right away or after a short time.
- **Scenario B — No QGC:** If the autopilot is not getting a competing “good” time from elsewhere, MAVROS from the Jetson may align well enough that **hover / offboard works**, which can make the bug look intermittent.

**Fix (MY580 — TA NTP server):** An NTP server on the TA desktop should be available so drones on the lab network can stay aligned. On the **Jetson**, edit `/etc/systemd/timesyncd.conf` (requires `sudo`, e.g. `sudo nano /etc/systemd/timesyncd.conf`), set the NTP line to point at the TA machine, for example:

```ini
NTP=10.42.0.100
```

(Replaces or uncomments `#NTP=` as appropriate.) **Reboot the Jetson**, then run:

```bash
timedatectl status
```

You want **`System clock synchronized: yes`** (wording may vary slightly by distro).

**On the TA / Ubuntu side:** Checking `/etc/systemd/timesyncd.conf` on the station machine confirms whether an NTP server is configured there. If teams still drift, verify with the TA that the server is still enabled and that the Jetson is using the correct IP.

## Post-Crash steps

A bad crash can induce many issues with your drone. Here is a list of typical issues and things to check post-crash:

* Anything physically broken. Most often 3D printed parts and standoffs. These are **meant to break** as they act as a last-resort impact barrier before your sensors start taking impact. Simply replace the broken parts.
* A bad impact can cause damage to your Realsense. This most often manifests as an increased sensitivity to vibration. Test the camera-only localization on the test bench with manual drone movement, ensure it works. You may notice large drifts in estimates after flying that you weren't seeing before. See the "Large drifts" issue in the previous section.
* Similarly, bad crashes can cause OrangeCube damage. Follow a similar test-bench procedure as the previous step, this time using QCG to confirm the IMU is behaving as expected.
* Check for propeller damage. Even small breaks and bends in a propeller can introduce large vibrations and instabilities during flight.


