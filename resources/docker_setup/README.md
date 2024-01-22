[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Docker Image Size](https://badgen.net/docker/size/utrarobosoccer/rob498?icon=docker&label=image%20size)](https://hub.docker.com/r/utrarobosoccer/rob498/)
[![Docker Pulls](https://badgen.net/docker/pulls/utrarobosoccer/rob498?icon=docker&label=pulls)](https://hub.docker.com/r/utrarobosoccer/rob498/)



### Running Instructions for Computer
- docker-compose.yaml is for a docker container that runs on a normal computer
- docker-compose.robot.yaml  is for a docker container that runs on a Jetson Nano. Building from scratch will take more then an hour.
```bash
roslaunch drone mavros_posix_sitl.launch # run simulation
roslaunch drone gui.launch # For visualization

docker-compose -f docker-compose.yaml pull # Use docker-compose build if you want to build locally
docker-compose -f docker-compose.yaml up

```

### Pull and Run the ARM image for Jetson Nano
```bash
docker-compose -f docker-compose.robot.yaml pull
docker-compose -f docker-compose.robot.yaml up
```
