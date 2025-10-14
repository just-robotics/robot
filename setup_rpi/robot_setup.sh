#!/bin/bash

cd ~/

grep -qxF "alias l='clear'" ~/.bashrc || echo "alias l='clear'" >> ~/.bashrc

sudo apt update -y && sudo apt upgrade -y
sudo apt install -y vim nano gcc make cmake git

sudo sed -i 's/^\(en_GB.UTF-8 UTF-8\)/# \1/' /etc/locale.gen
sudo sed -i 's/^# *en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen
sudo locale-gen
sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
exec bash

### DOCKER INSTALLATION
if ! command -v docker &> /dev/null; then
    echo "Docker not found. Installing..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker pi
else
    echo "Docker is already installed. Skipping installation."
fi
sudo docker run --rm hello-world

### ROBOT REPO FROM GIT
if [ ! -d "robot" ]; then
    git clone https://github.com/just-robotics/robot.git
    git config pull.rebase false
else
    echo "Repository 'robot' already exists, skipping clone."
fi

### ROBOT DOCKER SETUP
cd ~/robot/docker
# sudo docker compose up -d --build

touch .env
echo "ROBOT_ID=$ROBOT_ID" > .env

grep -qxF "cd ~/robot/docker" ~/.bashrc || echo "cd ~/robot/docker" >> ~/.bashrc

grep -qxF "alias up='sudo docker compose up -d --build'" ~/.bashrc || \
echo "alias up='sudo docker compose up -d --build'" >> ~/.bashrc

grep -qxF "alias restart='sudo docker compose restart'" ~/.bashrc || \
echo "alias restart='sudo docker compose restart'" >> ~/.bashrc

### MICROROS SETUP
sudo docker pull microros/micro_ros_static_library_builder:humble
cd ~/robot/microros_ws/src/microros_stm32/
sudo docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
 microros/micro_ros_static_library_builder:humble
cd ~/robot/docker
