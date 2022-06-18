#!/usr/bin/env bash
#
# Purpose
#   This script is designed to run a docker image built using build.bash.
#   See README.md and build.bash for more information.

if [ $# -lt 1 ]
then
    echo "Usage: $0 <docker image> [optional arguments to gz-sim]"
    exit 1
fi

IMG=$(basename $1)

ARGS=("$@")

# Make sure processes in the container can connect to the x server.
# This is necessary so Gazebo can create a context for OpenGL rendering
# (even headless).
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run -it \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -e GZ_PARTITION \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev/input:/dev/input" \
  --rm \
  --gpus all \
  --security-opt seccomp=unconfined \
  $IMG \
  ${@:2}
