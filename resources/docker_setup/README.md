[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![CodeFactor](https://www.codefactor.io/repository/github/manx52/rob498/badge/main)](https://www.codefactor.io/repository/github/manx52/rob498/overview/main)
[![Docker Image CI](https://github.com/manx52/ROB498/actions/workflows/docker_image.yml/badge.svg)](https://github.com/manx52/ROB498/actions/workflows/docker_image.yml)
[![Docker Image Arm64 CI](https://github.com/manx52/ROB498/actions/workflows/docker_image_arm.yml/badge.svg)](https://github.com/manx52/ROB498/actions/workflows/docker_image_arm.yml)
[![Documentation Status](https://readthedocs.org/projects/rob498/badge/?version=latest)](https://manx52.github.io/ROB498/api.html)
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
