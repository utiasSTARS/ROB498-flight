
### Setup Instructions
The docker setup is only available for computers with NVIDIA-capable GPUs (Minimum 32 GB of free space). TO verify this, run ```nvidia-smi``` and verify if you have any problems. If you haven't installed it, go to Software & Updates >> Additional Drivers and install the recommended NVIDIA driver

#### Verify CUDA on Jetson Nano 
```bash
cd /usr/bin
sudo ./tegrastats
```

Create a catkin workspace:
```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
catkin init
cd src
git clone git@github.com:git@github.com:manx52/ROB498.git

```


#### Install necessary packages
```bash
sudo apt-get install apt-utils python3-pip
pip3 install --upgrade setuptools
sudo apt-get install -y curl

```

#### Install docker (https://docs.docker.com/engine/install/ubuntu/)
```bash
curl -sSL https://get.docker.com/ | sh
```

#### Install docker-compose using pip install method (https://docs.docker.com/compose/install/)
```bash
sudo python3 -m pip install docker-compose
```

#### Install docker NVIDIA tools
```bash
# Make sure you have nvidia drivers (nvidia-smi), if not https://linuxconfig.org/how-to-install-the-nvidia-drivers-on-ubuntu-20-04-focal-fossa-linux
curl -s -L https://nvidia.github.io/nvidia-container-runtime/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-container-runtime/$distribution/nvidia-container-runtime.list | sudo tee /etc/apt/sources.list.d/nvidia-container-runtime.list
sudo apt-get update
sudo apt-get install -y nvidia-container-runtime nvidia-container-toolkit mesa-utils
sudo systemctl restart docker
echo "xhost +local:docker" >> ~/.bashrc && source ~/.bashrc
```

### Testing Docker for CUDA on Jetson Nano
- Open up 2 terminals
- In one, run docker-compose -f docker-compose.robot.test.yaml up
- In the other run the code below
```bash
docker ps # list all of the docker containers that are running
docker exec -it <docker container id> bash
python3
import torch
torch.cuda.is_available()
```
If the output is true then success CUDA is enabled on the docker.

### Reference

Terminology
- image: a file representing an OS
- container: a running instance of an image

Basic docker commands
```bash
docker system prune # Clean everything

# Images
docker image ls # list all of the docker images that are built
docker system prune --all # Delete all docker images
docker pull <docker image name> # Pull a docker image
docker tag old_image_name new_image_name
docker run -it <docker image id> bash # Start a docker image as a container

# Containers
docker ps # list all of the docker containers that are running
docker stop $(docker ps -aq) # Stop all running docker containers
docker rm $(docker ps -aq) # Remove all not running docker containers
docker stop $(docker ps -aq) # Stop all not running docker containers
docker exec -it <docker image id> bash # SSH into a docker image to see whats going on inside it

# Identify service with port and kill it
ps -aux | grep <PORT> && sudo kill -9 <PID>
sudo netstat -tulpn | grep <PORT> && sudo kill -9 <PID>

# Docker compose
docker-compose build # Build all docker images
docker-compose pull # Takes images from dockerhub
docker-compose push # Push all docker
docker-compose up # Start the containers specified in this file

```

### Notes
- You never need to build the images a second time as all the code is mounted
- For modifications please refer to the dockerfile and docker-compose.yaml. They are commented and show which sections are important to change
- The initial docker containers are hosted on utrarobosoccer's docker hub here is a link to setup your own. https://docs.docker.com/docker-hub/quickstart/
- if the docker commands give a permission error put a sudo in front of the command

