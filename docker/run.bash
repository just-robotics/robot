#!/bin/bash


SIM_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

xhost +local:docker > /dev/null || true


### DOCKER RUN ----------------------------------------------------------- #

docker run  -d -ti --rm \
            -e "DISPLAY" \
            -e "QT_X11_NO_MITSHM=1" \
            -e XAUTHORITY \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v /etc/localtime:/etc/localtime:ro \
            -v ${SIM_ROOT}/workspace:/workspace \
            --net=host \
            --privileged \
            --name "robot" "robot"  \
            > /dev/null
