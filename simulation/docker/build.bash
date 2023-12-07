#!/bin/bash


docker build -t sim_robot --network=host .


### Check if NVIDIA GPU flag is needed ----------------------------------- #


if [ -n "$(which nvidia-smi)" ] && [ -n "$(nvidia-smi)" ]; then
    GPU_FLAG=(--gpus all)
    SOURCE_IMG_NAME="althack/ros2:galactic-cuda-gazebo-nvidia"
else
    SOURCE_IMG_NAME="althack/ros2:galactic-gazebo"
fi


### Docker build -------------------------------------------------------- #


docker build -t sim_robot --network=host --build-arg from=$SOURCE_IMG_NAME .

