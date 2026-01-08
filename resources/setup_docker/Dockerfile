# selectes which distribution of ROS and Ubuntu to use
ARG BASE_IMAGE=utrarobosoccer/noetic

FROM $BASE_IMAGE as dependencies
WORKDIR /root/src
RUN apt-get update && rosdep update --rosdistro noetic
ADD . .
RUN rosdep install --from-paths . --ignore-src -r -s  | grep 'apt-get install' | awk '{print $3}' | sort  >  /tmp/catkin_install_list
WORKDIR /root/dependencies

FROM $BASE_IMAGE as builder
SHELL ["/bin/bash", "-c"]

# Install dependencies
# Choose which packages to install in docker container
# Should modify and remove redundancies
RUN apt-get update && \
    apt-get install -q -y software-properties-common && \
    add-apt-repository ppa:apt-fast/stable -y && \
    echo debconf apt-fast/maxdownloads string 16 | debconf-set-selections && \
    echo debconf apt-fast/dlflag boolean true | debconf-set-selections && \
    echo debconf apt-fast/aptmanager string apt-get | debconf-set-selections && \
    apt-get install -q -y apt-fast && \
    apt-get clean
RUN apt-get update && apt-fast install -y --no-install-recommends \
    screen \
    vim \
    python3-pip \
    python3-catkin-tools \
    python3-protobuf \
    wget \
    ccache \
    dirmngr \
    gnupg2 \
    lsb-release \
    net-tools \
    iputils-ping \
    apt-utils \
    software-properties-common \
    sudo \
    unzip \
    ros-noetic-robot-state-publisher \
    ros-noetic-realsense2-camera \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-pcl-ros \
    ros-noetic-image-proc \
    ros-noetic-stereo-image-proc \
    ros-noetic-camera-calibration \
    curl \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-xkb1 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-render-util0 \
    libxcb-randr0 \
    libxcb-keysyms1 \
    libxcb-xinerama0 \
    qt5-default \
    qtbase5-dev \
    python3-pyqt5
RUN DEBIAN_FRONTEND=noninteractive apt-get -y --no-install-recommends install keyboard-configuration # This needs to be its own individual step

# CUDA Installation
# Architecture: Use sbsa for arm build
# CUDA Installation Ref: https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_network
# CUDNN (Ref: https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installlinux)
ARG ARCHITECTURE=x86_64
ARG OS=ubuntu2004
RUN wget --progress=dot:mega https://developer.download.nvidia.com/compute/cuda/repos/$OS/$ARCHITECTURE/cuda-$OS.pin && \
    mv cuda-$OS.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/${OS}/$ARCHITECTURE/3bf863cc.pub && \
    add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/${OS}/$ARCHITECTURE/ /" && \
    apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-fast -yq --no-install-recommends install cuda libcudnn8 libcudnn8-dev libnccl2 libnccl-dev

RUN curl -sSL https://get.docker.com/ | sh

# Important for Jetson Nano
RUN if [[ "$(dpkg --print-architecture)" == "arm64" ]] ; then \
    apt-get update && \
    apt-get install -y libomp5 libopenblas-dev && \
    apt-get update && \
    apt-get install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev && \
    apt-get update -y && \
    apt-get install -y  pkg-config zlib1g-dev  libwebp-dev libtbb2 libtbb-dev  libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libv4l-dev cmake && \
    pip install gdown && \
    gdown https://drive.google.com/uc?id=1AQQuBS9skNk1mgZXMp0FmTIwjuxc81WY && \
    gdown https://drive.google.com/uc?id=1BaBhpAizP33SV_34-l3es9MOEFhhS1i2 && \
    pip install torch-1.11.0a0+gitbc2c6ed-cp38-cp38-linux_aarch64.whl && \
    pip install torchvision-0.12.0a0+9b5a3fe-cp38-cp38-linux_aarch64.whl && \
    rm -rf torch-1.11.0a0+gitbc2c6ed-cp38-cp38-linux_aarch64.whl && \
    rm -rf torchvision-0.12.0a0+9b5a3fe-cp38-cp38-linux_aarch64.whl; fi



# Create User
ARG USER="drone"
RUN groupadd -g 1000 $USER && \
    useradd -u 1000 -g 1000 -mrs /bin/bash -b /home -p $(openssl passwd -1 $USER) $USER && \
    usermod -aG sudo $USER && \
    echo "$USER ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    usermod --append --groups 29,20,104,46,5,44 $USER

# Install apt dependencies
COPY --from=dependencies /tmp/catkin_install_list /tmp/catkin_install_list
RUN (apt-get update || echo "Apt Error") && apt-fast install -y --no-install-recommends $(cat /tmp/catkin_install_list)

# Install python dependencies
USER $USER
COPY requirements.txt /tmp/requirements.txt
ENV PATH=/home/$USER/.local/bin:$PATH
RUN pip3 install -r /tmp/requirements.txt -f https://download.pytorch.org/whl/torch_stable.html

RUN mkdir -p /home/$USER/catkin_ws/src
WORKDIR /home/$USER/catkin_ws

## Predownload neural networks
#RUN mkdir -p /home/$USER/.cache/torch/hub/ &&  \
#    cd /home/$USER/.cache/torch/hub/ && \
#    wget https://github.com/ultralytics/yolov5/archive/master.zip && \
#    unzip /home/$USER/.cache/torch/hub/master.zip && \
#    mv yolov5-master ultralytics_yolov5_master && \
#    rm -rf master.zip

# Install Geographiclib Datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh &&  \
    sudo bash ./install_geographiclib_datasets.sh

# Build Python ROS Packages
COPY --from=dependencies --chown=$USER /root/src src/ROB498
RUN source /opt/ros/noetic/setup.bash && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
RUN source /opt/ros/noetic/setup.bash && catkin build --no-status drone
RUN echo "source /home/$USER/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Necessary for using CUDA
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda/targets/aarch64-linux/lib/
